"""
Backend config test: yaml の ``backend`` で実装(GPU / CPU)を選べるか.

Step 11 で追加した ``backend: "auto" | "gpu" | "cpu"`` のうち、どの環境でも
必ず利用できる ``"cpu"`` を指定して起動し、``3D-BBS backend: CPU`` の起動ログが
出ることを検証する(= yaml の指定が実際に選ばれた実装に反映されている)。

``"gpu"`` 指定の検証は「GPU 実装を含むビルドかどうか」に依存し、CPU のみの
ビルドでは常に起動失敗、GPU 入りビルドでは常に成功と結果が反転してしまうため、
自動テストには含めない(手動確認に留める)。

topic モードの fixture を使うため PCD ファイルは不要で、test data の有無に
関わらず CI で常に実行できる。
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
FIXTURE_YAML = (
    PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test_topic_mode.yaml"
)
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/backend_target"
EXPECTED_LOG = "3D-BBS backend: CPU"
LOG_WAIT_TIMEOUT_SEC = 15.0


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text += '\n## Step 11: 実装の明示指定\nbackend: "cpu"\n'
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_backend_cpu_"
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


class TestCpuBackendSelected(unittest.TestCase):
    def test_backend_log_reports_cpu(self, proc_output, node, tmp_yaml):
        proc_output.assertWaitFor(
            EXPECTED_LOG,
            process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC,
        )

    def test_remove_temp_yaml(self, tmp_yaml):
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
