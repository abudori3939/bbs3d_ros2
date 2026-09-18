"""
CPU num_threads warning test: コア数を超える ``cpu_num_threads`` を WARN で知らせるか.

``cpu_num_threads`` はスコア計算の ``#pragma omp parallel for num_threads(...)``
に渡る。マシンのハードウェアスレッド数を超える値は OpenMP がそのまま
スレッドを作ってコアを奪い合うだけで速くならないが、現状は黙って受け付ける。

検証項目: ``cpu_num_threads`` が ``std::thread::hardware_concurrency()`` を
超えるとき、WARN を出したうえで**起動は継続する**こと(ERROR にはしない。
コンテナの CPU 制限などでハードウェアスレッド数が実態とずれる場合があるため)。

どのマシンでも確実に超える値として 9999 を使う。``backend: "cpu"`` を明示する
ので GPU 機でも同じ結果になる。topic モードの fixture を使うため PCD 不要。
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
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/num_threads_exceeds_target"
NUM_THREADS = 9999
EXPECTED_WARN = f"cpu_num_threads {NUM_THREADS} exceeds"
EXPECTED_STARTED = "[ROS2] 3D-BBS initialized"
LOG_WAIT_TIMEOUT_SEC = 15.0


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text += (
        '\n## コア数を確実に超えるスレッド数(テスト用)\n'
        'backend: "cpu"\n'
        f'cpu_num_threads: {NUM_THREADS}\n'
    )
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False,
        prefix="bbs3d_num_threads_exceeds_"
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


class TestCpuNumThreadsExceedsWarning(unittest.TestCase):
    def test_warns_and_keeps_running(self, proc_output, node, tmp_yaml):
        proc_output.assertWaitFor(
            EXPECTED_WARN, process=node, timeout=LOG_WAIT_TIMEOUT_SEC)
        # WARN であって ERROR ではない: 起動は最後まで進む。
        proc_output.assertWaitFor(
            EXPECTED_STARTED, process=node, timeout=LOG_WAIT_TIMEOUT_SEC)


@launch_testing.post_shutdown_test()
class TestCleanup(unittest.TestCase):
    def test_remove_temp_yaml(self, tmp_yaml):
        try:
            os.unlink(tmp_yaml)
        except FileNotFoundError:
            pass
