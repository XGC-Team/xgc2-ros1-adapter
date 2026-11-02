"""Exercise real selected Debian packing with real installed ELF fixtures.

Requires XGC2_TEST_ADAPTER_IMAGE, XGC2_TEST_ADAPTER_BUILT (three debs from a
real selected source Build), XGC2_TEST_ADAPTER_UGV (the UGV CI deb), and
XGC2_TEST_BABEL_DEB (the distro ROS Babel Fish deb). All work is disposable.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
IMAGE = os.environ.get('XGC2_TEST_ADAPTER_IMAGE')
BUILT = os.environ.get('XGC2_TEST_ADAPTER_BUILT')
UGV = os.environ.get('XGC2_TEST_ADAPTER_UGV')
BABEL = os.environ.get('XGC2_TEST_BABEL_DEB')


@unittest.skipUnless(IMAGE and BUILT and UGV and BABEL and shutil.which('docker'), 'builder and Debian fixtures are required')
class SelectedPackagesTest(unittest.TestCase):
    def test_real_packages_include_tools_and_only_selected_type(self):
        with tempfile.TemporaryDirectory(prefix='adapter-packages-') as tmp:
            output = Path(tmp) / 'output'
            output.mkdir()
            script = '''set -euo pipefail
trap 'chown -R "$TEST_UID:$TEST_GID" /output' EXIT
dpkg -i /fixtures/babel.deb >/dev/null
mkdir -p /tmp/stage
for file in /built/*.deb /fixtures/ugv.deb; do dpkg-deb -x "$file" /tmp/stage; done
export LD_LIBRARY_PATH=/tmp/stage/opt/ros/noetic/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}
for selection in px4 ugv; do
 XGC2_ADAPTER_PACKAGE_SELECTION="$selection" /adapter/.xgc2/scripts/package_debs.sh --install-root /tmp/stage --output-dir "/output/$selection"
done
'''
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'none',
                '-e', 'TEST_UID=' + str(os.getuid()), '-e', 'TEST_GID=' + str(os.getgid()),
                '-v', str(ROOT) + ':/adapter:ro', '-v', str(Path(BUILT).resolve()) + ':/built:ro',
                '-v', str(Path(UGV).resolve()) + ':/fixtures/ugv.deb:ro',
                '-v', str(Path(BABEL).resolve()) + ':/fixtures/babel.deb:ro',
                '-v', str(output) + ':/output', '--entrypoint', '/bin/bash', IMAGE, '-c', script,
            ], text=True, capture_output=True, timeout=120)
            self.assertEqual(result.returncode, 0, result.stderr)
            for selection, expected in (('px4', 'ros-noetic-xgc2-px4-multirotor-adapter'),
                                        ('ugv', 'ros-noetic-xgc2-ugv-adapter')):
                packages = list((output / selection).glob('*.deb'))
                names = {subprocess.check_output(['dpkg-deb', '-f', str(p), 'Package'], text=True).strip(): p for p in packages}
                self.assertEqual(set(names), {expected, 'ros-noetic-xgc2-ros1-adapter-common', 'ros-noetic-xgc2-ros1-tools-adapter'})
                tools = names['ros-noetic-xgc2-ros1-tools-adapter']
                deps = subprocess.check_output(['dpkg-deb', '-f', str(tools), 'Depends'], text=True)
                version = subprocess.check_output(['dpkg-deb', '-f', str(tools), 'Version'], text=True).strip()
                self.assertIn('ros-noetic-xgc2-ros1-adapter-common (= ' + version + ')', deps)
                self.assertIn('ros-noetic-ros-babel-fish', deps)
                self.assertIn('ros-noetic-rosbag', deps)
                listing = subprocess.check_output(['dpkg-deb', '-c', str(tools)], text=True)
                self.assertIn('xgc_ros1_tools_adapter_rosbag_recorder', listing)
                self.assertIn('xgc_ros1_tools_adapter_service_helper', listing)


if __name__ == '__main__':
    unittest.main()
