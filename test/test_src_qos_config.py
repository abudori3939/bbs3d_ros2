"""
Source cloud QoS config test: src_cloud_qos_* の yaml 設定が subscriber に反映されるか.

source (lidar) の sub は現状 ``KeepLast(50).best_effort()`` + volatile 固定で、
target 側 (``target_cloud_qos_*``) と非対称になっている。volatile な subscriber には
transient_local publisher の latch 済みサンプルが配送されないため、ノード起動前に
一発だけ publish された点群は永久に届かない。

fixture ``bbs3d_ros2_test_topic_mode.yaml`` を temp file に複製し、
``src_cloud_qos_reliability: "reliable"`` と ``src_cloud_qos_durability:
"transient_local"`` を指定して ``bbs3d_ros2_node`` を起動する。graph 上の
lidar sub の QoS が yaml と一致することを assert。

RED 時点(コード未実装):現状コードは yaml キーを読まず常に ``best_effort`` +
``volatile`` で sub を作るため、``RELIABLE`` / ``TRANSIENT_LOCAL`` を期待する
assertEqual が ``BEST_EFFORT`` / ``VOLATILE`` を観測して FAIL。ビルドは通り、
実行時の振る舞い不一致で正当な RED となる。

lidar topic 名はこのテスト専用の名前に差し替える(他テストのノードが同名 topic を
subscribe していると ``get_subscriptions_info_by_topic`` が複数件返るため)。
PCD ファイル不要(topic モード固定)で CI で常に実行される。
"""
import os
import re
import tempfile
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_YAML = (
    PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test_topic_mode.yaml"
)
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/src_qos_target"
LIDAR_TOPIC_NAME = "/_bbs3d_ros2_test/src_qos/points"

DISCOVERY_TIMEOUT_SEC = 20.0


def _launch_with_node():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text = re.sub(
        r"^lidar_topic_name:.*$",
        f'lidar_topic_name: "{LIDAR_TOPIC_NAME}"',
        text,
        flags=re.MULTILINE,
    )
    text += (
        '\n## Source cloud QoS (Step 13)\n'
        'src_cloud_qos_reliability: "reliable"\n'
        'src_cloud_qos_durability: "transient_local"\n'
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_src_qos_"
    )
    tmp.write(text)
    tmp.close()

    node = launch_ros.actions.Node(
        package="bbs3d_ros2",
        executable="bbs3d_ros2_node",
        name="bbs3d_ros2_node",
        parameters=[{"config": tmp.name}],
        output="screen",
    )
    ld = launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])
    return ld, {"node": node, "tmp_yaml": tmp.name}


@pytest.mark.launch_test
def generate_test_description():
    return _launch_with_node()


class TestSrcCloudQosConfig(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.observer_node = rclpy.create_node("test_src_qos_observer")

    def tearDown(self):
        self.observer_node.destroy_node()

    def _wait_for(self, predicate, timeout_sec):
        end_ns = (
            self.observer_node.get_clock().now().nanoseconds
            + int(timeout_sec * 1e9)
        )
        while self.observer_node.get_clock().now().nanoseconds < end_ns:
            if predicate():
                return True
            rclpy.spin_once(self.observer_node, timeout_sec=0.2)
        return predicate()

    def test_src_cloud_sub_qos_reflects_yaml(self, node, tmp_yaml):
        # graph に lidar sub が現れるまで待つ。
        def has_src_sub():
            infos = self.observer_node.get_subscriptions_info_by_topic(
                LIDAR_TOPIC_NAME
            )
            return any(i.node_name == "bbs3d_ros2_node" for i in infos)

        ok = self._wait_for(has_src_sub, DISCOVERY_TIMEOUT_SEC)
        self.assertTrue(
            ok,
            f"bbs3d_ros2_node sub on {LIDAR_TOPIC_NAME} not visible within "
            f"{DISCOVERY_TIMEOUT_SEC}s",
        )

        infos = self.observer_node.get_subscriptions_info_by_topic(
            LIDAR_TOPIC_NAME
        )
        src_infos = [i for i in infos if i.node_name == "bbs3d_ros2_node"]
        self.assertEqual(
            len(src_infos), 1,
            f"Expected exactly 1 bbs3d_ros2_node sub on {LIDAR_TOPIC_NAME}, "
            f"got {len(src_infos)}",
        )
        qos = src_infos[0].qos_profile

        # yaml 設定: reliable + transient_local(default の best_effort +
        # volatile から変えられることを見る)。
        self.assertEqual(
            qos.reliability, ReliabilityPolicy.RELIABLE,
            f"Expected RELIABLE reliability (yaml-configured), "
            f"got {qos.reliability!r}",
        )
        self.assertEqual(
            qos.durability, DurabilityPolicy.TRANSIENT_LOCAL,
            f"Expected TRANSIENT_LOCAL durability (yaml-configured), "
            f"got {qos.durability!r}",
        )


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):
    def test_remove_temp_yaml(self, tmp_yaml):
        if tmp_yaml is None:
            return
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
