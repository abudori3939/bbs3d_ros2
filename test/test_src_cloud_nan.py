"""
Source cloud sanitize test: 非有限点 (NaN) を含む source を受けたときの扱い.

Step 12 で target 点群には NaN 除去 (`sanitize_cloud`) を入れたが、source 点群
(`run_localization` の前処理) には入っていない。`pcl::VoxelGrid` が非有限点を
除くのは `is_dense == false` のときだけで、`src_leaf_size: 0.0`(間引き無効)や
leaf size 過小で PCL が downsample をスキップした場合はそのまま通る。
crop (`min/max_scan_range`) も無効なら NaN がそのまま BBS3D に渡る。

検証項目:

(a) source が非有限点しか含まない場合、Trigger service が
    ``success=false`` かつ ``message == "source cloud has no finite points"``
    を返すこと(= BBS3D に空/NaN の点群を渡さずに早期 return する)。
(b) 有限点と非有限点が混ざった source を受けたとき、落とした点数を WARN で
    知らせること(target 側の "Dropped N non-finite points ..." と対称)。

fixture は topic モードのものを流用し、`src_leaf_size` と crop を無効化して
NaN が前処理を通り抜ける状況を作る。PCD ファイルは不要。
"""
import math
import os
import re
import tempfile
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_YAML = (
    PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test_topic_mode.yaml"
)
SERVICE_NAME = "/bbs3d_ros2_node/localize"
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/src_nan_target"
LIDAR_TOPIC_NAME = "/_bbs3d_ros2_test/src_nan/points"
IMU_TOPIC_NAME = "/_bbs3d_ros2_test/src_nan/imu"

NUM_FINITE_POINTS = 50
NUM_NAN_POINTS = 3
EXPECTED_NO_FINITE_MESSAGE = "source cloud has no finite points"
EXPECTED_DROPPED_SUBSTRING = (
    f"Dropped {NUM_NAN_POINTS} non-finite points"
)
EXPECTED_DROPPED_DETAIL = "source cloud"
EXPECTED_REBUILT_SUBSTRING = "Target voxelmap rebuilt"

DISCOVERY_TIMEOUT_SEC = 20.0
SERVICE_AVAILABLE_TIMEOUT_SEC = 15.0
RESPONSE_TIMEOUT_SEC = 30.0
LOG_WAIT_TIMEOUT_SEC = 20.0
SETTLE_SEC = 2.0


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text = re.sub(r"^lidar_topic_name:.*$",
                  f'lidar_topic_name: "{LIDAR_TOPIC_NAME}"', text,
                  flags=re.MULTILINE)
    text = re.sub(r"^imu_topic_name:.*$",
                  f'imu_topic_name: "{IMU_TOPIC_NAME}"', text,
                  flags=re.MULTILINE)
    # downsample と crop を無効化して、NaN が前処理を素通りする経路を作る。
    text = re.sub(r"^src_leaf_size:.*$", "src_leaf_size: 0.0", text,
                  flags=re.MULTILINE)
    text = re.sub(r"^min_scan_range:.*$", "min_scan_range: 0.0", text,
                  flags=re.MULTILINE)
    text = re.sub(r"^max_scan_range:.*$", "max_scan_range: 0.0", text,
                  flags=re.MULTILINE)
    # RED 時に NaN 点群がそのまま BBS3D へ渡るため、探索が長引かないよう
    # timeout を有効にしておく(テスト全体の実行時間を bound する)。
    text = re.sub(r"^timeout_msec:.*$", "timeout_msec: 2000", text,
                  flags=re.MULTILINE)
    # source を取りこぼさずに届けるため latched 相当にする(Step 13-1)。
    text += (
        '\nsrc_cloud_qos_reliability: "reliable"\n'
        'src_cloud_qos_durability: "transient_local"\n'
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_src_nan_"
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


def _make_target_cloud() -> PointCloud2:
    points = [(float(i), float(i) * 0.1, 0.0)
              for i in range(NUM_FINITE_POINTS)]
    header = Header()
    header.frame_id = "map"
    return point_cloud2.create_cloud_xyz32(header, points)


def _make_all_nan_cloud() -> PointCloud2:
    points = [(math.nan, 0.0, 0.0), (0.0, math.nan, 0.0),
              (0.0, 0.0, math.nan)]
    header = Header()
    header.frame_id = "map"
    return point_cloud2.create_cloud_xyz32(header, points)


def _make_mixed_cloud() -> PointCloud2:
    points = [(float(i), float(i) * 0.1, 0.0)
              for i in range(NUM_FINITE_POINTS)]
    points += [(math.nan, 0.0, 0.0), (0.0, math.nan, 0.0),
               (0.0, 0.0, math.nan)]
    header = Header()
    header.frame_id = "map"
    return point_cloud2.create_cloud_xyz32(header, points)


def _make_imu() -> Imu:
    # 地図座標系の「上」= +Z。重力整列行列が厳密に単位行列になる値。
    msg = Imu()
    msg.header.frame_id = "map"
    msg.linear_acceleration.x = 0.0
    msg.linear_acceleration.y = 0.0
    msg.linear_acceleration.z = 9.81
    return msg


class TestSourceCloudWithNaN(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.observer_node = rclpy.create_node("test_src_nan_observer")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_pub = self.observer_node.create_publisher(
            PointCloud2, TARGET_TOPIC_NAME, latched
        )
        self.src_pub = self.observer_node.create_publisher(
            PointCloud2, LIDAR_TOPIC_NAME, latched
        )
        self.imu_pub = self.observer_node.create_publisher(
            Imu, IMU_TOPIC_NAME, 10
        )

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

    def _spin_for(self, duration_sec):
        self._wait_for(lambda: False, duration_sec)

    def _topic_present(self, name):
        current = {
            t[0] for t in self.observer_node.get_topic_names_and_types()
        }
        return name in current

    def _call_localize(self, client):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.observer_node, future, timeout_sec=RESPONSE_TIMEOUT_SEC
        )
        response = future.result()
        self.assertIsNotNone(response, "Service did not respond")
        return response

    def test_source_non_finite_points_are_handled(self, proc_output, node,
                                                  tmp_yaml):
        self.assertTrue(
            self._wait_for(
                lambda: self._topic_present(TARGET_TOPIC_NAME),
                DISCOVERY_TIMEOUT_SEC),
            f"Target subscription {TARGET_TOPIC_NAME} not visible",
        )
        self.target_pub.publish(_make_target_cloud())
        proc_output.assertWaitFor(
            EXPECTED_REBUILT_SUBSTRING, process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC)

        # IMU は volatile sub なので、マッチ後に数発流す。
        self.assertTrue(
            self._wait_for(
                lambda: self._topic_present(IMU_TOPIC_NAME),
                DISCOVERY_TIMEOUT_SEC),
            f"IMU subscription {IMU_TOPIC_NAME} not visible",
        )
        for _ in range(5):
            self.imu_pub.publish(_make_imu())
            self._spin_for(0.2)

        client = self.observer_node.create_client(Trigger, SERVICE_NAME)
        self.assertTrue(
            client.wait_for_service(timeout_sec=SERVICE_AVAILABLE_TIMEOUT_SEC),
            f"Service {SERVICE_NAME} not available",
        )

        # (a) 非有限点しか無い source: BBS3D に渡さず reason を返すこと。
        self.src_pub.publish(_make_all_nan_cloud())
        self._spin_for(SETTLE_SEC)
        response = self._call_localize(client)
        self.assertFalse(
            response.success,
            "Expected failure for an all-NaN source cloud, got success=true",
        )
        self.assertEqual(
            response.message, EXPECTED_NO_FINITE_MESSAGE,
            f"Expected message {EXPECTED_NO_FINITE_MESSAGE!r}, "
            f"got: {response.message!r}",
        )

        # (b) 有限点と混在: 落とした点数を WARN で知らせること。
        self.src_pub.publish(_make_mixed_cloud())
        self._spin_for(SETTLE_SEC)
        self._call_localize(client)
        proc_output.assertWaitFor(
            EXPECTED_DROPPED_SUBSTRING, process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC)
        proc_output.assertWaitFor(
            EXPECTED_DROPPED_DETAIL, process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC)


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):
    def test_remove_temp_yaml(self, tmp_yaml):
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
