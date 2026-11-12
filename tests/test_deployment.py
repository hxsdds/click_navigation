"""Regressions for failures observed during first deployment."""
import hashlib
import importlib.util
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class DeploymentTests(unittest.TestCase):
    def check_path(self, path):
        return subprocess.run(
            ['bash', '-c', 'root="$1"; source "$2"', '--', path,
             str(ROOT/'scripts/check_build_path.sh')], capture_output=True, text=True)

    def test_ascii_path_accepted(self):
        self.assertEqual(self.check_path('/home/user/work/click_navigation').returncode, 0)

    def test_chinese_path_rejected_before_build(self):
        result = self.check_path('/mnt/c/Desktop/新建文件夹/click_navigation')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('ASCII path without spaces', result.stderr)

    def test_spaces_rejected_before_build(self):
        self.assertNotEqual(self.check_path('/home/user/new folder/project').returncode, 0)

    def test_partial_install_is_not_reported_as_built(self):
        spec = importlib.util.spec_from_file_location('deployment_doctor', ROOT/'scripts/doctor.py')
        doctor = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(doctor)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'config').mkdir()
            (root/'install').mkdir()
            (root/'install/local_setup.bash').touch()
            (root/'scripts').mkdir()
            (root/'scripts/plan_3d.py').touch()
            (root/'src/message_pkg').mkdir(parents=True)
            (root/'src/message_pkg/package.xml').write_text('<package><name>message_pkg</name></package>')
            (root/'map').write_bytes(b'map')
            entry = {'file': 'map', 'sha256': hashlib.sha256(b'map').hexdigest()}
            (root/'config/static_navigation_map.json').write_text(json.dumps(
                {key: entry for key in ('known_grid', 'ground_grid', 'world', 'octomap')}))
            output = io.StringIO()
            with patch.object(doctor, 'ROOT', root), patch.object(sys, 'argv', ['doctor.py']), redirect_stdout(output):
                self.assertNotEqual(doctor.main(), 0)
            self.assertIn('FAIL: workspace packages installed', output.getvalue())
            self.assertNotIn('OK: workspace built', output.getvalue())
