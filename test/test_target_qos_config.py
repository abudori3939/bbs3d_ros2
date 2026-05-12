"""
Target cloud QoS config test: yaml の target_cloud_qos_reliability /
target_cloud_qos_durability が subscriber に反映されるか.

fixture ``bbs3d_ros2_test_topic_mode_volatile.yaml`` を temp file に複製し、
``target_cloud_qos_reliability: "best_effort"`` と
``target_cloud_qos_durability: "volatile"`` を設定して ``bbs3d_ros2_node``
を起動する。graph 上の sub の QoS が yaml と一致することを assert。

RED 時点(コード未実装):現状コードは yaml キーを読まず常に
``reliable``+``transient_local`` で sub を作るため、assertEqual で
``BEST_EFFORT`` / ``VOLATILE`` を期待する箇所が ``RELIABLE`` / ``TRANSIENT_LOCAL``
を観測して FAIL。ビルドは通り、実行時の振る舞い不一致で正当な RED となる。

PCD ファイル不要(topic モード固定)で CI で常に実行される。
"""
import os
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
    PROJECT_ROOT / "test" / "fixtures"
    / "bbs3d_ros2_test_topic_mode_volatile.yaml"
)
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/target_qos"

DISCOVERY_TIMEOUT_SEC = 20.0


def _launch_with_node():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_QOS_TOPIC__", TARGET_TOPIC_NAME
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_qos_"
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


class TestTargetCloudQosConfig(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.observer_node = rclpy.create_node("test_target_qos_observer")

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

    def test_target_cloud_sub_qos_reflects_yaml(self, node, tmp_yaml):
        # graph に target_cloud sub が現れるまで待つ。
        def has_target_sub():
            infos = self.observer_node.get_subscriptions_info_by_topic(
                TARGET_TOPIC_NAME
            )
            return any(i.node_name == "bbs3d_ros2_node" for i in infos)

        ok = self._wait_for(has_target_sub, DISCOVERY_TIMEOUT_SEC)
        self.assertTrue(
            ok,
            f"bbs3d_ros2_node sub on {TARGET_TOPIC_NAME} not visible within "
            f"{DISCOVERY_TIMEOUT_SEC}s",
        )

        # bbs3d_ros2_node が出している sub の QoS を取得して assert。
        infos = self.observer_node.get_subscriptions_info_by_topic(
            TARGET_TOPIC_NAME
        )
        target_infos = [i for i in infos if i.node_name == "bbs3d_ros2_node"]
        self.assertEqual(
            len(target_infos), 1,
            f"Expected exactly 1 bbs3d_ros2_node sub on {TARGET_TOPIC_NAME}, "
            f"got {len(target_infos)}",
        )
        qos = target_infos[0].qos_profile

        # yaml 設定: best_effort + volatile。
        self.assertEqual(
            qos.reliability, ReliabilityPolicy.BEST_EFFORT,
            f"Expected BEST_EFFORT reliability (yaml-configured), "
            f"got {qos.reliability!r}",
        )
        self.assertEqual(
            qos.durability, DurabilityPolicy.VOLATILE,
            f"Expected VOLATILE durability (yaml-configured), "
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
