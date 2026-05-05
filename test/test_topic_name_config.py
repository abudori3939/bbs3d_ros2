"""
Topic name config test: yaml renames are honored by publishers.

fixture yaml の末尾に ``tar_points_topic_name: /_bbs3d_ros2_test/tar_points_renamed``
を追記して bbs3d_ros2_node を起動したとき、起動完了直後に default の
``/tar_points`` ではなくリネーム後の名前で ``PointCloud2`` メッセージが
publish されることを ``rclpy`` の subscriber で検証する。

RED 時点(コードがハードコードのまま): ``/tar_points`` でしか publish され
ないため、リネーム名 ``/_bbs3d_ros2_test/tar_points_renamed`` への
subscribe には何も届かず、``assertGreater(len(received), 0)`` が FAIL する。

Skipped when test data (data/target/*.pcd) is not present locally.
``BBS3D_REQUIRE_TEST_DATA=1`` switches skip → fail (CI / strict mode)。
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
from sensor_msgs.msg import PointCloud2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "data" / "target"
FIXTURE_YAML = PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test.yaml"
# 別ノード(他テスト)と衝突しない擬似プライベート名前空間。
RENAMED_TAR_TOPIC = "/_bbs3d_ros2_test/tar_points_renamed"
# load_tar_clouds + 1s sleep + publish の流れで余裕を見て 20s。
RECEIVE_TIMEOUT_SEC = 20.0

DATA_AVAILABLE = DATA_DIR.exists() and any(DATA_DIR.glob("*.pcd"))
REQUIRE_DATA = os.environ.get("BBS3D_REQUIRE_TEST_DATA", "").lower() in (
    "1", "true", "yes",
)


def _launch_with_node():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUDS_PATH__", str(DATA_DIR)
    )
    # fixture yaml に Step 9 の新キーを追記。これが反映されるかが本テストの観点。
    text += f'\ntar_points_topic_name: "{RENAMED_TAR_TOPIC}"\n'
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_topic_cfg_"
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


class TestRenamedTopicPublishes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.listener_node = rclpy.create_node("test_topic_name_listener")
        self.received = []
        self.listener_node.create_subscription(
            PointCloud2,
            RENAMED_TAR_TOPIC,
            lambda msg: self.received.append(msg),
            10,
        )

    def tearDown(self):
        self.listener_node.destroy_node()

    def test_tar_points_publishes_on_renamed_topic(self, node, tmp_yaml):
        if not DATA_AVAILABLE:
            self.skipTest(
                f"Test data not found at {DATA_DIR}. "
                "Download per 3d_bbs/ros2_test/ros2_test_code.md."
            )
        end_time = (
            self.listener_node.get_clock().now().nanoseconds
            + int(RECEIVE_TIMEOUT_SEC * 1e9)
        )
        while (
            not self.received
            and self.listener_node.get_clock().now().nanoseconds < end_time
        ):
            rclpy.spin_once(self.listener_node, timeout_sec=0.5)
        self.assertGreater(
            len(self.received),
            0,
            f"No msg received on {RENAMED_TAR_TOPIC} within "
            f"{RECEIVE_TIMEOUT_SEC}s — yaml の tar_points_topic_name "
            "が反映されていない可能性がある",
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
