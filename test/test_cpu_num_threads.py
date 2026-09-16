"""
CPU num_threads config test: yaml の ``cpu_num_threads`` が CPU 実装に届くか.

上流 ``cpu::BBS3D`` のスレッド数は ctor で 4 固定 (``num_threads_(4)``) で、
スコア計算の ``#pragma omp parallel for num_threads(num_threads_)`` に効く。
CPU バックエンドで実機性能を出すには設定できる必要があるが、現状 yaml から
触る手段が無い。

検証項目: ``cpu_num_threads: 3`` を指定して起動したとき、起動ログが
``3D-BBS backend: CPU (num_threads=3)`` になること(スレッド数は外から観測
できないため、適用結果をログに出すことを仕様に含める)。

RED 時点(コード未実装):ログは ``3D-BBS backend: CPU`` までしか出ないため
``assertWaitFor`` がタイムアウトして FAIL。ビルドは通り、起動そのものは成功
した上での振る舞い不一致なので正当な RED となる。

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
import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_YAML = (
    PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test_topic_mode.yaml"
)
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/num_threads_target"
NUM_THREADS = 3
EXPECTED_LOG = f"3D-BBS backend: CPU (num_threads={NUM_THREADS})"
LOG_WAIT_TIMEOUT_SEC = 15.0


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text += (
        '\n## Step 13: CPU 実装のスレッド数\n'
        'backend: "cpu"\n'
        f'cpu_num_threads: {NUM_THREADS}\n'
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_num_threads_"
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


class TestCpuNumThreadsConfig(unittest.TestCase):
    def test_backend_log_reports_num_threads(self, proc_output, node,
                                             tmp_yaml):
        proc_output.assertWaitFor(
            EXPECTED_LOG,
            process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC,
        )


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):
    def test_remove_temp_yaml(self, tmp_yaml):
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
