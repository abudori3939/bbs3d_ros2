"""
Source cloud leaf size test: src_leaf_size 過小を診断ログで知らせるか.

`pcl::VoxelGrid` はボクセル数が int32 を超える場合、`PCL_WARN` を stderr に
直接出したうえで **入力をそのまま出力して return する**(`voxel_grid.hpp` の
`output = *input_`)。target 側は Step 12 でノード自身の WARN を入れたが、
source 側には無い。地図同士の位置合わせのように source が広域点群になると、
数百万点が間引かれないまま BBS3D に渡り、候補変換ごとに O(N_src) のスコア
計算が走って実質終わらなくなる。しかも何の診断も出ない。

検証項目: `src_leaf_size` が過小なとき、ノード自身が WARN で
「leaf size が小さすぎて downsample がスキップされる」ことを知らせること。

fixture の `src_leaf_size` を 0.001 に置換し、3 軸それぞれ 2000 m 離れた点を
source として送ることで、PCL と同じ判定式 ((2000/0.001)+1)^3 ≈ 8.0e18 >
2^31-1 を確実に超えさせる。crop は無効化して点が残るようにする。
PCD ファイルは不要。
"""
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
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/src_leaf_target"
LIDAR_TOPIC_NAME = "/_bbs3d_ros2_test/src_leaf/points"
IMU_TOPIC_NAME = "/_bbs3d_ros2_test/src_leaf/imu"

TOO_SMALL_LEAF = 0.001
SPAN_M = 2000.0
# 部分一致を "src_leaf_size" と "too small" に分けると、後者は target 側の
# "tar_leaf_size ... is too small for this map ..." でも満たせてしまう。
# fixture が変わったときに黙って偽 GREEN 化しないよう、1 本の完全な文字列で待つ。
EXPECTED_WARN_LOG = (
    f"src_leaf_size {TOO_SMALL_LEAF:g} is too small for this cloud"
)
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
    text = re.sub(r"^src_leaf_size:.*$", f"src_leaf_size: {TOO_SMALL_LEAF}",
                  text, flags=re.MULTILINE)
    # crop を無効化しないと 2000 m 離れた点が max_scan_range で消える。
    text = re.sub(r"^min_scan_range:.*$", "min_scan_range: 0.0", text,
                  flags=re.MULTILINE)
    text = re.sub(r"^max_scan_range:.*$", "max_scan_range: 0.0", text,
                  flags=re.MULTILINE)
    text = re.sub(r"^timeout_msec:.*$", "timeout_msec: 2000", text,
                  flags=re.MULTILINE)
    text += (
        '\nsrc_cloud_qos_reliability: "reliable"\n'
        'src_cloud_qos_durability: "transient_local"\n'
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_src_leaf_"
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
    points = [(float(i), float(i) * 0.1, 0.0) for i in range(50)]
    header = Header()
    header.frame_id = "map"
    return point_cloud2.create_cloud_xyz32(header, points)


def _make_wide_source_cloud() -> PointCloud2:
    # 3 軸とも SPAN_M の広がりを持つ 4 点。leaf 0.001 ではボクセル数が
    # int32 を溢れるので PCL は downsample をスキップする。
    points = [
        (0.0, 0.0, 0.0),
        (SPAN_M, 0.0, 0.0),
        (0.0, SPAN_M, 0.0),
        (0.0, 0.0, SPAN_M),
    ]
    header = Header()
    header.frame_id = "map"
    return point_cloud2.create_cloud_xyz32(header, points)


def _make_imu() -> Imu:
    msg = Imu()
    msg.header.frame_id = "map"
    msg.linear_acceleration.x = 0.0
    msg.linear_acceleration.y = 0.0
    msg.linear_acceleration.z = 9.81
    return msg


class TestSrcLeafSizeTooSmallWarning(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.observer_node = rclpy.create_node("test_src_leaf_observer")
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

    def test_warns_that_src_leaf_size_is_too_small(self, proc_output, node,
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

        self.src_pub.publish(_make_wide_source_cloud())
        self._spin_for(SETTLE_SEC)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.observer_node, future, timeout_sec=RESPONSE_TIMEOUT_SEC
        )
        self.assertIsNotNone(future.result(), "Service did not respond")

        proc_output.assertWaitFor(
            EXPECTED_WARN_LOG, process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC)


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):
    def test_remove_temp_yaml(self, tmp_yaml):
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
