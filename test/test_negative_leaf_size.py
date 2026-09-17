"""
Leaf size config test (異常系): 負の leaf size で clean に起動失敗するか.

``src_leaf_size`` / ``tar_leaf_size`` は「0.0 で off、正の値で間引き」という
仕様で、負値に意味は無い。にもかかわらず source 側の分岐は上流由来の
``if (src_leaf_size != 0.0f)`` なので**負値も間引き経路に入る**。

Step 13 で入れた ``min_feasible_leaf_size()`` は PCL と同じ
``(int64)(extent * (1/leaf)) + 1`` の積でボクセル数を見積もるため、
``1/leaf`` が負になると 1 軸目で ``d == 0`` になり ``product`` が 0 に潰れ、
次の軸で ``kMaxCells / product`` が **整数ゼロ除算 (SIGFPE)** を起こす。
``~/localize`` を叩いた瞬間にプロセスが落ち、Service の response も返らない。

検証項目: 負の ``src_leaf_size`` は ``load_config`` の段階で ERROR を出して
exit code 1 で終了すること(localize まで到達させない)。

構成は ``test_backend_invalid_value.py`` に合わせている(active テストで
ERROR ログを待ってから post_shutdown で exit code を見る)。topic モードの
fixture を使うため PCD ファイルは不要。
"""
import os
import re
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
TARGET_TOPIC_NAME = "/_bbs3d_ros2_test/negative_leaf_target"
NEGATIVE_LEAF = -1.0
EXPECTED_ERROR_SUBSTRING = "src_leaf_size must be"
LOG_WAIT_TIMEOUT_SEC = 15.0


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace(
        "__TARGET_CLOUD_TOPIC__", TARGET_TOPIC_NAME
    )
    text = re.sub(r"^src_leaf_size:.*$", f"src_leaf_size: {NEGATIVE_LEAF}",
                  text, flags=re.MULTILINE)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_leaf_negative_"
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


class TestNegativeLeafSizeErrorLog(unittest.TestCase):
    def test_error_log_appears(self, proc_output, node, tmp_yaml):
        proc_output.assertWaitFor(
            EXPECTED_ERROR_SUBSTRING,
            process=node,
            timeout=LOG_WAIT_TIMEOUT_SEC,
        )


@launch_testing.post_shutdown_test()
class TestNegativeLeafSizeOutcome(unittest.TestCase):
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
