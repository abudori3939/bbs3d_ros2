"""
Bad PCD path test: bbs3d_ros2_node terminates cleanly with an ERROR log.

target_clouds が存在しないパスを指している状態で起動したときに:
- ``Couldn't load target clouds`` を含む ERROR ログを残す
- exit code 1 で clean exit する(segfault -11 ではなく)

を検証する。テストデータ不要(意図的に bad path を作る)。

post_shutdown_test に寄せている理由: 不正パスでは Bbs3dNode が起動直後に
例外/exit する想定で、active テスト中(プロセス生存中)に assertWaitFor を
呼ぶと "Launch stopped before active tests finished" になりやすいため。
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
FIXTURE_YAML = PROJECT_ROOT / "test" / "fixtures" / "bbs3d_ros2_test.yaml"
BAD_PATH = "/tmp/bbs3d_ros2_nonexistent_target_dir"
EXPECTED_ERROR_SUBSTRING = "Couldn't load target clouds"


@pytest.mark.launch_test
def generate_test_description():
    text = FIXTURE_YAML.read_text().replace("__TARGET_CLOUDS_PATH__", BAD_PATH)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="bbs3d_bad_path_"
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


@launch_testing.post_shutdown_test()
class TestBadPathOutcome(unittest.TestCase):
    def test_exit_code_is_one_not_segfault(self, proc_info, node, tmp_yaml):
        # exit code の解釈:
        #   1   -> clean error exit(GREEN の正解)
        #   0   -> success(誤り、ノードは失敗すべき)
        #   -11 -> SIGSEGV(現状の RED、bad path 後の null deref)
        #   その他負値 -> シグナル(framework cleanup の SIGINT 等)
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
