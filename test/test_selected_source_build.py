"""Exercise the product's selective source entry without compiling other products."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / '.xgc2/scripts/build_source.sh'


class SelectedSourceBuildTest(unittest.TestCase):
    def test_each_selection_runs_only_its_source_and_install(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cmake = root / 'cmake'
            cmake.write_text('#!/usr/bin/env python3\n'
                             'import json,os,sys\n'
                             'with open(os.environ["CALLS"],"a") as f:\n'
                             ' f.write(json.dumps({"args":sys.argv[1:],"destdir":os.environ.get("DESTDIR")})+"\\n")\n')
            cmake.chmod(0o755)
            for selection, package in (('common', 'common'), ('px4', 'px4_multirotor_ros1_adapter'),
                                       ('ugv', 'ugv_ros1_adapter'), ('tools', 'ros1_tools_adapter')):
                with self.subTest(selection=selection):
                    calls = root / (selection + '.jsonl')
                    env = dict(os.environ, PATH=str(root) + ':' + os.environ['PATH'], CALLS=str(calls), BUILD_JOBS='3')
                    stage, work = root / 'stage', root / 'work'
                    subprocess.run([str(SCRIPT), str(work), str(stage), '/opt/ros/noetic', selection],
                                   env=env, check=True, capture_output=True, text=True)
                    entries = [json.loads(line) for line in calls.read_text().splitlines()]
                    self.assertEqual(len(entries), 3)
                    self.assertEqual(entries[0]['args'][:4], ['-S', str(ROOT / 'src' / package), '-B', str(work / package)])
                    self.assertEqual(entries[1]['args'], ['--build', str(work / package), '--parallel', '3'])
                    self.assertEqual(entries[2]['args'], ['--install', str(work / package)])
                    self.assertEqual(entries[2]['destdir'], str(stage))

    def test_unknown_selection_cannot_invoke_build(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run([str(SCRIPT), tmp, tmp, '/opt/ros/noetic', 'scene'], text=True, capture_output=True)
            self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main()
