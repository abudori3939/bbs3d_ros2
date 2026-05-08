"""
Topic name config test: yaml-configured names appear in the ROS graph.

fixture yaml の末尾に 6 個のリネーム指定(``tar_points_topic_name`` 等)を
追記して bbs3d_ros2_node を起動したとき、ROS グラフから取得したトピック /
サービス名一覧に「リネーム後の名前が存在する」「default 名は存在しない」を
検証する。Step 9 が変えるのは「publisher / subscription / service の名前
そのもの」なので、メッセージ流通経路ではなく、グラフ上の名前を直接 assert
する。

RED 時点(コードがハードコードのまま): publisher は ``/tar_points`` 等の
default 名で作られるため、renamed 名は graph に現れず、default 名が残る。
``assertTrue(ok)`` および ``assertFalse(leaked)`` がそれぞれ FAIL する
正当な RED(ビルドは通り、実行時の振る舞い不一致)。

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

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "data" / "target"
FIXTURE_YAML = PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test.yaml"

# Step 9 の各 yaml キーをこの値にリネームする。default (`/tar_points` 等)
# とは別の擬似プライベート名前空間を使い、他テスト / 外部 publisher と衝突
# させない。
RENAMED = {
    "tar_points_topic_name": "/_bbs3d_ros2_test/tar_points_renamed",
    "src_points_on_global_pose_topic_name":
        "/_bbs3d_ros2_test/src_points_renamed",
    "global_pose_topic_name": "/_bbs3d_ros2_test/global_pose_renamed",
    "score_topic_name": "/_bbs3d_ros2_test/score_renamed",
    "time_topic_name": "/_bbs3d_ros2_test/time_renamed",
    "localize_topic_name": "/_bbs3d_ros2_test/localize_renamed",
}
# yaml で全部リネームしたとき、これらの default 名は graph から消えていること。
# 5 publisher + 1 Bool subscription(`~/localize` をノード名前空間で解決した名前)。
# `get_topic_names_and_types()` は pub / sub を区別せず graph 全体を返すため、
# Bool sub の default もこの集合で leak 検査できる。
DEFAULT_TOPIC_NAMES = {
    "/tar_points",
    "/src_points_on_global_pose",
    "/global_pose",
    "/score",
    "/time",
    "/bbs3d_ros2_node/localize",
}
# rename 後に消えていることを確認したい default Service 名(`~/localize` 解決後)。
DEFAULT_SERVICE_NAME = "/bbs3d_ros2_node/localize"
# 起動(PCD load + voxelmap 構築)と ROS 2 graph discovery の両方を吸収する。
DISCOVERY_TIMEOUT_SEC = 20.0

DATA_AVAILABLE = DATA_DIR.exists() and any(DATA_DIR.glob("*.pcd"))
REQUIRE_DATA = os.environ.get("BBS3D_REQUIRE_TEST_DATA", "").lower() in (
    "1", "true", "yes",
)


def _launch_with_node():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUDS_PATH__", str(DATA_DIR)
    )
    rename_block = "\n".join(
        f'{key}: "{value}"' for key, value in RENAMED.items()
    )
    text += "\n" + rename_block + "\n"
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


class TestRenamedTopicNames(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.observer_node = rclpy.create_node("test_topic_name_observer")

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

    def test_topic_names_match_yaml(self, node, tmp_yaml):
        if not DATA_AVAILABLE:
            self.skipTest(
                f"Test data not found at {DATA_DIR}. "
                "Download per 3d_bbs/ros2_test/ros2_test_code.md."
            )
        expected = set(RENAMED.values())  # 5 pubs + 1 sub (localize Bool topic)

        def all_present():
            current = {
                t[0] for t in self.observer_node.get_topic_names_and_types()
            }
            return expected.issubset(current)

        ok = self._wait_for(all_present, DISCOVERY_TIMEOUT_SEC)
        current = {
            t[0] for t in self.observer_node.get_topic_names_and_types()
        }
        missing = expected - current
        self.assertTrue(
            ok,
            f"Renamed topics not all visible within {DISCOVERY_TIMEOUT_SEC}s. "
            f"Missing: {sorted(missing)}",
        )
        leaked = DEFAULT_TOPIC_NAMES & current
        self.assertFalse(
            leaked,
            f"Default topic names still present despite yaml renames: "
            f"{sorted(leaked)}",
        )

    def test_localize_service_name_matches_yaml(self, node, tmp_yaml):
        if not DATA_AVAILABLE:
            self.skipTest(
                f"Test data not found at {DATA_DIR}. "
                "Download per 3d_bbs/ros2_test/ros2_test_code.md."
            )
        expected = RENAMED["localize_topic_name"]

        def service_present():
            current = {
                s[0]
                for s in self.observer_node.get_service_names_and_types()
            }
            return expected in current

        ok = self._wait_for(service_present, DISCOVERY_TIMEOUT_SEC)
        current = {
            s[0] for s in self.observer_node.get_service_names_and_types()
        }
        self.assertTrue(
            ok,
            f"Renamed service {expected} not visible within "
            f"{DISCOVERY_TIMEOUT_SEC}s. Services found: {sorted(current)}",
        )
        self.assertNotIn(
            DEFAULT_SERVICE_NAME, current,
            f"Default service {DEFAULT_SERVICE_NAME} still present despite "
            f"yaml rename to {expected}.",
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
