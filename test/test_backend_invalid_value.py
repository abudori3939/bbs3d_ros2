"""
Backend config test (異常系): 不正な ``backend`` 値で clean に起動失敗するか.

``backend: "tpu"`` のような未知の値を黙って無視して起動してしまうと、ユーザは
意図と違う実装で走っていることに気付けない。``target_source_mode`` / QoS の
文字列検証と同じく、``load_config`` で弾いて exit code 1 で終了することを
検証する。

構成は ``test_bad_pcd_path.py`` に合わせている(active テストで ERROR ログを
待ってから post_shutdown で exit code を見る)。topic モードの fixture を使う
ため PCD ファイルは不要。
"""
import os
import tempfile
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_YAML = (
    PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test_topic_mode.yaml"
)
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/backend_invalid_target"
INVALID_BACKEND = "tpu"
EXPECTED_ERROR_SUBSTRING = "backend must be"
LOG_WAIT_TIMEOUT_SEC = 15.0


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text += f'\n## Step 11: 不正値(テスト用)\nbackend: "{INVALID_BACKEND}"\n'
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_backend_bad_"
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


class TestInvalidBackendErrorLog(unittest.TestCase):
    def test_error_log_appears(self, proc_output, node, tmp_yaml):
        proc_output.assertWaitFor(
            EXPECTED_ERROR_SUBSTRING,
            process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC,
        )


@launch_testing.post_shutdown_test()
class TestInvalidBackendOutcome(unittest.TestCase):
    def test_exit_code_is_one(self, proc_info, node, tmp_yaml):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[1],
            process=node,
        )

    def test_remove_temp_yaml(self, tmp_yaml):
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
