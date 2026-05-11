"""
Target source mode test: topic モードで subscriber と未受信ガードが正しく動くか.

fixture yaml に ``target_source_mode: "topic"`` と
``target_cloud_topic_name: "/_bbs3d_ros2_test/target_renamed"`` を追記して
``bbs3d_ros2_node`` を起動し、以下 2 点を検証する:

(a) ``target_cloud_topic_name`` で指定した名前の subscription が ROS graph
    に現れること(= topic モードで対応する sub が作られていること)。
(b) ``target_clouds`` 未受信状態で Trigger service を呼ぶと ``success=false``
    かつ ``message == "target map not loaded"`` を返すこと(= localize 経路が
    新しいガードを通っていること)。

RED 時点(コード未実装): (a) 現状コードは ``target_source_mode`` を読まず、
subscriber も作らないため graph に現れない → ``assertTrue`` が timeout で
FAIL。(b) ``run_localization`` に target loaded ガードが無いので
``"point cloud not received"`` を返す → ``assertEqual`` が文字列不一致で
FAIL。ビルドは通り、実行時の振る舞い不一致で正当な RED となる。

Skipped when test data (data/target/*.pcd) is not present locally:
RED フェーズの現状コードは pcd モード固定で動くため ``target_clouds`` の
有効パスが必要(GREEN 後は topic モードで PCD ロード不要になるが、テスト
互換性を保つため他テストと同じ skip / fail パターンを使う)。
``BBS3D_REQUIRE_TEST_DATA=1`` switches skip → fail (CI / strict mode).
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
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "data" / "target"
FIXTURE_YAML = PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test.yaml"
SERVICE_NAME = "/bbs3d_ros2_node/localize"
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/target_renamed"

# 起動(PCD load + voxelmap 構築)と graph discovery の合計を吸収する。
DISCOVERY_TIMEOUT_SEC = 20.0
SERVICE_AVAILABLE_TIMEOUT_SEC = 15.0
RESPONSE_TIMEOUT_SEC = 10.0
EXPECTED_NOT_LOADED_MESSAGE = "target map not loaded"

DATA_AVAILABLE = DATA_DIR.exists() and any(DATA_DIR.glob("*.pcd"))
REQUIRE_DATA = os.environ.get("BBS3D_REQUIRE_TEST_DATA", "").lower() in (
    "1", "true", "yes",
)


def _launch_with_node():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUDS_PATH__", str(DATA_DIR)
    )
    text += '\ntarget_source_mode: "topic"\n'
    text += f'target_cloud_topic_name: "{TARGET_TOPIC_NAME}"\n'
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_target_mode_"
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


def _launch_empty():
    ld = launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    return ld, {"node": None, "tmp_yaml": None}


@pytest.mark.launch_test
def generate_test_description():
    if not DATA_AVAILABLE and REQUIRE_DATA:
        raise RuntimeError(
            f"BBS3D_REQUIRE_TEST_DATA=1 but test data missing at {DATA_DIR}. "
            "Download per 3d_bbs/ros2_test/ros2_test_code.md."
        )
    if DATA_AVAILABLE:
        return _launch_with_node()
    return _launch_empty()


def _make_dummy_pointcloud2() -> PointCloud2:
    # 50 点ほどの (x, y, z) float32 dummy 点群。voxelmap 再構築の最小入力。
    points = [(float(i), float(i) * 0.1, 0.0) for i in range(50)]
    header = Header()
    header.frame_id = "map"
    return point_cloud2.create_cloud_xyz32(header, points)


class TestTargetSourceModeTopic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.observer_node = rclpy.create_node("test_target_mode_observer")

    def tearDown(self):
        self.observer_node.destroy_node()

    def _wait_for(self, predicate, timeout_sec):
        # ROS 2 graph discovery が反映されるまで polling する。
        end_ns = (
            self.observer_node.get_clock().now().nanoseconds
            + int(timeout_sec * 1e9)
        )
        while self.observer_node.get_clock().now().nanoseconds < end_ns:
            if predicate():
                return True
            rclpy.spin_once(self.observer_node, timeout_sec=0.2)
        return predicate()

    def test_target_subscription_present_at_startup(self, node, tmp_yaml):
        if not DATA_AVAILABLE:
            self.skipTest(
                f"Test data not found at {DATA_DIR}. "
                "Download per 3d_bbs/ros2_test/ros2_test_code.md."
            )

        def target_sub_present():
            current = {
                t[0] for t in self.observer_node.get_topic_names_and_types()
            }
            return TARGET_TOPIC_NAME in current

        ok = self._wait_for(target_sub_present, DISCOVERY_TIMEOUT_SEC)
        current = {
            t[0] for t in self.observer_node.get_topic_names_and_types()
        }
        self.assertTrue(
            ok,
            f"Expected target subscription {TARGET_TOPIC_NAME} not visible "
            f"within {DISCOVERY_TIMEOUT_SEC}s. Topics found: {sorted(current)}",
        )

    def test_localize_rejected_before_target_received(self, node, tmp_yaml):
        if not DATA_AVAILABLE:
            self.skipTest(
                f"Test data not found at {DATA_DIR}. "
                "Download per 3d_bbs/ros2_test/ros2_test_code.md."
            )
        client = self.observer_node.create_client(Trigger, SERVICE_NAME)
        self.assertTrue(
            client.wait_for_service(timeout_sec=SERVICE_AVAILABLE_TIMEOUT_SEC),
            f"Service {SERVICE_NAME} not available within "
            f"{SERVICE_AVAILABLE_TIMEOUT_SEC}s",
        )
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.observer_node, future, timeout_sec=RESPONSE_TIMEOUT_SEC
        )
        response = future.result()
        self.assertIsNotNone(response, "Service did not respond")
        self.assertFalse(
            response.success,
            f"Expected failure with target not loaded, got success=true "
            f"(message={response.message!r})",
        )
        self.assertEqual(
            response.message,
            EXPECTED_NOT_LOADED_MESSAGE,
            f"Expected message {EXPECTED_NOT_LOADED_MESSAGE!r}, "
            f"got: {response.message!r}",
        )

    def test_target_publish_unblocks_localize_guard(self, node, tmp_yaml):
        # GREEN 確認用: target を transient_local QoS で publish した後、
        # `target map not loaded` ガードを抜けて別 reason("point cloud not
        # received" 等)を返すこと。RED 時点(現状コード)では guard 自体が
        # 無いため最初の Trigger 呼び出しから "point cloud not received" を
        # 返してしまい、`response_before` の assertEqual が FAIL する。
        if not DATA_AVAILABLE:
            self.skipTest(
                f"Test data not found at {DATA_DIR}. "
                "Download per 3d_bbs/ros2_test/ros2_test_code.md."
            )
        client = self.observer_node.create_client(Trigger, SERVICE_NAME)
        self.assertTrue(
            client.wait_for_service(timeout_sec=SERVICE_AVAILABLE_TIMEOUT_SEC),
            f"Service {SERVICE_NAME} not available within "
            f"{SERVICE_AVAILABLE_TIMEOUT_SEC}s",
        )

        # 受信前: "target map not loaded" が返ること(test 2 と同じ assert
        # を入れているが、本 test の前提条件 = ガードが効いていることを
        # 確認するため重複させる)。
        future_before = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.observer_node, future_before, timeout_sec=RESPONSE_TIMEOUT_SEC
        )
        response_before = future_before.result()
        self.assertIsNotNone(response_before, "Service did not respond before")
        self.assertEqual(
            response_before.message,
            EXPECTED_NOT_LOADED_MESSAGE,
            f"Pre-receive: expected {EXPECTED_NOT_LOADED_MESSAGE!r}, "
            f"got: {response_before.message!r}",
        )

        # target publish: transient_local QoS でないと subscriber 側に
        # 届かない可能性。明示的に latched 相当を指定。
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        target_pub = self.observer_node.create_publisher(
            PointCloud2, TARGET_TOPIC_NAME, qos
        )
        target_pub.publish(_make_dummy_pointcloud2())

        # voxelmap 再構築待ち(3-4s 程度 + 余裕)
        deadline_ns = (
            self.observer_node.get_clock().now().nanoseconds
            + int(15.0 * 1e9)
        )
        unblocked = False
        last_message = None
        while self.observer_node.get_clock().now().nanoseconds < deadline_ns:
            future_after = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                self.observer_node, future_after,
                timeout_sec=RESPONSE_TIMEOUT_SEC,
            )
            response_after = future_after.result()
            if response_after is None:
                continue
            last_message = response_after.message
            if response_after.message != EXPECTED_NOT_LOADED_MESSAGE:
                unblocked = True
                break
            rclpy.spin_once(self.observer_node, timeout_sec=0.5)

        self.assertTrue(
            unblocked,
            f"Localize guard never unblocked after target publish. "
            f"last message={last_message!r}",
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
