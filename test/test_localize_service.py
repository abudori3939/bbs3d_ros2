"""
Localize service test: /bbs3d_ros2_node/localize returns failure messages.

Verifies that calling the Service with no /livox/points or /livox/imu
published returns ``success=false`` with a message containing
``"not received"``. This exercises the input-not-yet-received path of
the shared run_localization() helper.

The other 3 failure cases (``"localization timed out"``,
``"score below threshold"``, plus success path) require either an
engineered fixture (timeout) or the rosbag pipeline (score / success),
and are deferred to Step 8 of docs/IMPLEMENTATION_PLAN.md.

Skipped when test data (data/target/*.pcd) is not present locally,
since the node still needs PCD to initialize before the Service becomes
responsive. ``BBS3D_REQUIRE_TEST_DATA=1`` switches skip → fail (CI).
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
from std_srvs.srv import Trigger

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "data" / "target"
FIXTURE_YAML = PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test.yaml"
SERVICE_NAME = "/bbs3d_ros2_node/localize"
SERVICE_AVAILABLE_TIMEOUT_SEC = 15.0
RESPONSE_TIMEOUT_SEC = 10.0

DATA_AVAILABLE = DATA_DIR.exists() and any(DATA_DIR.glob("*.pcd"))
REQUIRE_DATA = os.environ.get("BBS3D_REQUIRE_TEST_DATA", "").lower() in (
    "1", "true", "yes",
)


def _launch_with_node():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUDS_PATH__", str(DATA_DIR)
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_localize_test_"
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


class TestLocalizeService(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.client_node = rclpy.create_node("test_localize_client")

    def tearDown(self):
        self.client_node.destroy_node()

    def test_service_returns_failure_without_inputs(self, node, tmp_yaml):
        if not DATA_AVAILABLE:
            self.skipTest(
                f"Test data not found at {DATA_DIR}. "
                "Download per 3d_bbs/ros2_test/ros2_test_code.md."
            )
        client = self.client_node.create_client(Trigger, SERVICE_NAME)
        self.assertTrue(
            client.wait_for_service(timeout_sec=SERVICE_AVAILABLE_TIMEOUT_SEC),
            f"Service {SERVICE_NAME} not available within "
            f"{SERVICE_AVAILABLE_TIMEOUT_SEC}s",
        )
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.client_node, future, timeout_sec=RESPONSE_TIMEOUT_SEC
        )
        response = future.result()
        self.assertIsNotNone(response, "Service did not respond")
        self.assertFalse(
            response.success,
            f"Expected failure with no inputs, got success=true (message={response.message!r})",
        )
        self.assertIn(
            "not received",
            response.message,
            f"Expected 'not received' substring, got: {response.message!r}",
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
