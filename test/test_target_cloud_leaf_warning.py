"""
Target cloud leaf size test: leaf size 過小を診断ログで知らせるか.

`pcl::VoxelGrid` はボクセル数が int32 を超える場合、`PCL_WARN` を stderr に
直接出したうえで **入力をそのまま出力して return する**(PCL 1.12〜1.14 で確認、
`voxel_grid.hpp` の `output = *input_`)。失敗にはならないが downsample が
黙ってスキップされ、意図せず全点が voxelmap に入る。実測では 9.2M 点・
404 x 430 x 70 m の地図に `tar_leaf_size: 0.1` を指定すると、間引かれずに
9,238,897 点(構築 2403 ms)が使われた(0.5 なら 982,869 点 / 872 ms)。

検証項目: `tar_leaf_size` が過小なとき、ノード自身が RCLCPP の WARN で
「leaf size が小さすぎて downsample がスキップされる」ことを知らせること。

fixture の `tar_leaf_size` を 0.001 に置換し、3 軸それぞれ 2000 m 離れた点を
含む小さな点群を送ることで、PCL と同じ判定式 ((2000/0.001)+1)^3 ≈ 8.0e18 >
2^31-1 を確実に超えさせる。PCD ファイルは不要。
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_YAML = (
    PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test_topic_mode.yaml"
)
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/leaf_warning_target"
TOO_SMALL_LEAF = 0.001
SPAN_M = 2000.0
NUM_POINTS = 4
EXPECTED_WARN_SUBSTRING = "tar_leaf_size"
EXPECTED_WARN_DETAIL = "too small"
# downsample がスキップされる = 入力点数のまま voxelmap が作られる。
EXPECTED_REBUILT_SUBSTRING = f"Target voxelmap rebuilt: {NUM_POINTS} points"
DISCOVERY_TIMEOUT_SEC = 20.0
LOG_WAIT_TIMEOUT_SEC = 20.0


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text = re.sub(r"^tar_leaf_size:.*$", f"tar_leaf_size: {TOO_SMALL_LEAF}",
                  text, flags=re.MULTILINE)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_leaf_warning_"
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


def _make_wide_pointcloud() -> PointCloud2:
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


class TestLeafSizeTooSmallWarning(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.observer_node = rclpy.create_node("test_leaf_warning_observer")

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

    def _publish_target(self):
        def target_sub_present():
            current = {
                t[0] for t in self.observer_node.get_topic_names_and_types()
            }
            return TARGET_TOPIC_NAME in current

        self.assertTrue(
            self._wait_for(target_sub_present, DISCOVERY_TIMEOUT_SEC),
            f"Target subscription {TARGET_TOPIC_NAME} not visible",
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        target_pub = self.observer_node.create_publisher(
            PointCloud2, TARGET_TOPIC_NAME, qos
        )
        target_pub.publish(_make_wide_pointcloud())

    def test_warns_that_leaf_size_is_too_small(self, proc_output, node,
                                               tmp_yaml):
        self._publish_target()
        proc_output.assertWaitFor(
            EXPECTED_WARN_SUBSTRING, process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC)
        proc_output.assertWaitFor(
            EXPECTED_WARN_DETAIL, process=node, timeout=LOG_WAIT_TIMEOUT_SEC)

    def test_downsample_is_skipped_by_pcl(self, proc_output, node, tmp_yaml):
        # 前提条件の確認: PCL は間引かずに素通しするため、voxelmap には
        # 入力と同じ点数が入る(この振る舞い自体は PCL 側の仕様)。
        self._publish_target()
        proc_output.assertWaitFor(
            EXPECTED_REBUILT_SUBSTRING, process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC)


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):
    def test_remove_temp_yaml(self, tmp_yaml):
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
