"""
Config validation test (異常系): yaml の不正値で clean に起動失敗するか.

``load_config`` は不正値を ERROR ログ + ``return false`` で弾き、ノードは
exit code 1 で終了する仕様になっている。``backend`` と負の leaf size には
専用テストがあるが、以下は検証コードだけあってテストが無かった。

- ``cpu_num_threads`` が 1 未満
- ``src_cloud_qos_reliability`` / ``src_cloud_qos_durability`` の未知の文字列
- ``target_cloud_qos_reliability`` / ``target_cloud_qos_durability`` の未知の文字列

``launch_testing.parametrize`` で 1 ファイルから設定ごとに起動し、各ケースで
「ERROR ログが出る → ノードが自分で終了する → exit code 1」を確認する。
既存の検証を守るためのテストなので、追加時点から PASS する(RED ではない)。

終了待ちと ``keep_alive`` の理由は ``test_backend_invalid_value.py`` と同じ。
topic モードの fixture を使うため PCD ファイルは不要。
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
import launch_testing.markers
import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_YAML = (
    PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test_topic_mode.yaml"
)
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/invalid_config_target"
LOG_WAIT_TIMEOUT_SEC = 15.0
SHUTDOWN_WAIT_TIMEOUT_SEC = 10.0

# (yaml に追記する 1 行, 期待する ERROR ログの部分文字列)
CASES = [
    ("cpu_num_threads: 0",
     "cpu_num_threads must be 1 or greater"),
    ('src_cloud_qos_reliability: "fast"',
     "src_cloud_qos_reliability must be"),
    ('src_cloud_qos_durability: "forever"',
     "src_cloud_qos_durability must be"),
    ('target_cloud_qos_reliability: "fast"',
     "target_cloud_qos_reliability must be"),
    ('target_cloud_qos_durability: "forever"',
     "target_cloud_qos_durability must be"),
]


@pytest.mark.launch_test
@launch_testing.parametrize("invalid_line, expected_error", CASES)
@launch_testing.markers.keep_alive
def generate_test_description(invalid_line, expected_error):
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text += f"\n## 不正値(テスト用)\n{invalid_line}\n"
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_invalid_config_"
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
    return ld, {
        "node": node,
        "tmp_yaml": tmp.name,
        "expected_error": expected_error,
    }


class TestInvalidConfigErrorLog(unittest.TestCase):
    def test_error_log_appears(self, proc_output, proc_info, node,
                               expected_error):
        proc_output.assertWaitFor(
            expected_error,
            process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC,
        )
        proc_info.assertWaitForShutdown(
            process=node, timeout=SHUTDOWN_WAIT_TIMEOUT_SEC)


@launch_testing.post_shutdown_test()
class TestInvalidConfigOutcome(unittest.TestCase):
    def test_exit_code_is_one(self, proc_info, node):
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
