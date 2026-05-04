"""
Smoke test: bbs3d_ros2_node initializes against a valid yaml.

Verifies that launching ``bbs3d_ros2_node`` with a fixture yaml whose
``target_clouds`` points at a real PCD directory results in the upstream
"[ROS2] 3D-BBS initialized" log line within 10 seconds.

Skipped when test data (data/target/*.pcd) is not present locally.
Place test data per 3d_bbs/ros2_test/ros2_test_code.md.

Set ``BBS3D_REQUIRE_TEST_DATA=1`` to fail (instead of skip) when the test
data is missing -- useful for CI / strict gating.
"""
import os
import tempfile
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "data" / "target"
FIXTURE_YAML = PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test.yaml"
EXPECTED_LOG = "[ROS2] 3D-BBS initialized"
LOG_WAIT_TIMEOUT_SEC = 10.0

DATA_AVAILABLE = DATA_DIR.exists() and any(DATA_DIR.glob("*.pcd"))
REQUIRE_DATA = os.environ.get("BBS3D_REQUIRE_TEST_DATA", "").lower() in (
    "1", "true", "yes",
)


def _launch_with_node():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUDS_PATH__", str(DATA_DIR)
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_test_"
    )
    tmp.write(text)
    tmp.close()

    node = launch_ros.actions.Node(
        package="bbs3d_ros2",
        executable="bbs3d_ros2_node",
        name="bbs3d_ros2_node",
        parameters=[{"config": tmp.name}],
        output="both",
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


class TestNodeInit(unittest.TestCase):
    def test_initialized_log_appears(self, proc_output, node, tmp_yaml):
        if not DATA_AVAILABLE:
            self.skipTest(
                f"Test data not found at {DATA_DIR}. "
                "Download per 3d_bbs/ros2_test/ros2_test_code.md."
            )
        # stream を指定せず、stdout / stderr のいずれにマッチすれば OK にする
        # (Step 8 で std::cout → RCLCPP_INFO に移行し、init ログは stderr に出る)
        proc_output.assertWaitFor(
            EXPECTED_LOG,
            process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC,
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
