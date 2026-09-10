"""
Target cloud sanitize test: 非有限点 (NaN) を含む target を受けても TF が壊れないか.

`target_cloud_callback` は受信点群から NaN/Inf を落としていない。`pcl::VoxelGrid`
が非有限点を除くのは `is_dense == false` のときだけで、`tar_leaf_size: 0.0`
(間引き無効)や leaf size 過小で PCL が downsample をスキップした場合は
そのまま通る。その結果 `broadcast_viewer_frame` の centroid が NaN になり、
`map -> viewer` の TF に NaN が乗る(voxelmap にも NaN 点が入る)。

検証項目: NaN を含む target を publish した後に broadcast される
`map -> viewer` TF の translation が **すべて有限**であること。

fixture は topic モードのものを流用し `tar_leaf_size` を 0.0 に置換するため、
PCD ファイルは不要で test data の有無に関わらず実行できる。
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_YAML = (
    PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test_topic_mode.yaml"
)
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/nan_target"
VIEWER_FRAME_ID = "viewer"
NUM_FINITE_POINTS = 50
# 有限点 (i, 0.1*i, 0) i=0..49 の重心。ノードが NaN を落としていれば
# broadcast される viewer TF はこの値になる。同一 domain の別ノードが
# 流す viewer TF を誤って拾って GREEN になるのを防ぐため、値まで assert する。
EXPECTED_CENTROID = (
    sum(range(NUM_FINITE_POINTS)) / NUM_FINITE_POINTS,
    sum(i * 0.1 for i in range(NUM_FINITE_POINTS)) / NUM_FINITE_POINTS,
    0.0,
)
CENTROID_TOLERANCE = 1e-3
DISCOVERY_TIMEOUT_SEC = 20.0
TF_TIMEOUT_SEC = 20.0
PUBLISH_RETRIES = 5


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    # downsample 無効化: VoxelGrid を通さないことで NaN がそのまま前処理を
    # 通り抜ける状況を作る(leaf size 過小で PCL が素通しする場合と同じ経路)。
    text = re.sub(r"^tar_leaf_size:.*$", "tar_leaf_size: 0.0", text,
                  flags=re.MULTILINE)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_nan_target_"
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


def _make_pointcloud_with_nan() -> PointCloud2:
    # 有限点 50 点 + NaN 点 3 点。NaN が落とされれば centroid は有限になる。
    points = [(float(i), float(i) * 0.1, 0.0)
              for i in range(NUM_FINITE_POINTS)]
    points += [(math.nan, 0.0, 0.0), (0.0, math.nan, 0.0),
               (0.0, 0.0, math.nan)]
    header = Header()
    header.frame_id = "map"
    return point_cloud2.create_cloud_xyz32(header, points)


class TestTargetCloudWithNaN(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.observer_node = rclpy.create_node("test_nan_target_observer")

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

    def test_viewer_tf_is_finite(self, node, tmp_yaml):
        # TransformBroadcaster は volatile なので、publish する前に /tf を
        # 購読しておく必要がある。
        received = []
        self.observer_node.create_subscription(
            TFMessage, "/tf",
            lambda msg: received.extend(
                t for t in msg.transforms
                if t.child_frame_id == VIEWER_FRAME_ID
            ),
            10)

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

        # viewer TF は target 1 通につき 1 回しか broadcast されず、/tf は
        # volatile なので、subscription のマッチが間に合わないと取りこぼす。
        # target を撒き直して retry する(topic モードは何度受けても良い)。
        got_tf = False
        for _ in range(PUBLISH_RETRIES):
            target_pub.publish(_make_pointcloud_with_nan())
            got_tf = self._wait_for(
                lambda: bool(received), TF_TIMEOUT_SEC / PUBLISH_RETRIES
            )
            if got_tf:
                break
        self.assertTrue(
            got_tf,
            f"No {VIEWER_FRAME_ID} transform broadcast within "
            f"{TF_TIMEOUT_SEC}s",
        )

        translation = received[-1].transform.translation
        values = (translation.x, translation.y, translation.z)
        self.assertTrue(
            all(math.isfinite(v) for v in values),
            f"viewer TF translation must be finite, got {values} "
            "(non-finite points from the target cloud leaked into the "
            "centroid)",
        )
        # 値まで確認する: 同一 ROS_DOMAIN_ID の別ノードが流した viewer TF を
        # 拾っていた場合、有限ではあっても期待値と一致しない。
        for axis, actual, expected in zip("xyz", values, EXPECTED_CENTROID):
            self.assertAlmostEqual(
                actual, expected, delta=CENTROID_TOLERANCE,
                msg=f"viewer TF {axis} must be the centroid of the finite "
                    f"points ({expected}), got {actual}",
            )


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):
    def test_remove_temp_yaml(self, tmp_yaml):
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
