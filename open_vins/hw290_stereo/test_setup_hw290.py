#!/usr/bin/env python3
"""Exercise setup control flow with fake system/network/build commands."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
STUB = r'''#!/usr/bin/python3
import json, os, pathlib, sys
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
root = pathlib.Path(os.environ['TEST_ROOT'])
with open(os.environ['TEST_CALLS'], 'a') as f:
    f.write(json.dumps([name] + args) + '\n')
if name == 'sudo':
    if args == ['-v']: sys.exit(0)
    if args[0] not in ('env', 'apt-get', 'dpkg', 'usermod', 'install', 'systemctl', 'docker', 'locale-gen', 'add-apt-repository'):
        sys.exit('Unexpected privileged command')
    os.environ['TEST_SUDO'] = '1'
    os.execvpe(args[0], args, os.environ)
elif name == 'curl':
    url = next(a for a in args if a.startswith('https://'))
    dest = pathlib.Path(args[args.index('-o') + 1])
    if url.endswith('/releases/latest'): dest.write_text('{"tag_name": "1.3.0"}')
    elif url.endswith('.deb'): dest.write_text('fake deb')
    else: sys.exit('Unexpected download: ' + url)
elif name == 'apt-cache':
    sys.exit(1 if os.environ.get('TEST_BOOTSTRAP') else 0)
elif name == 'dpkg-query':
    print('install ok installed' if any('${Status}' in a for a in args) else 'fake-package\t1.0')
elif name == 'dpkg':
    if args == ['--print-architecture']: print('amd64')
elif name == 'arduino-cli':
    if args == ['version']: print('arduino-cli Version: 1.5.1')
    elif args == ['core', 'list']: print('arduino:avr 1.8.8 1.8.8 Arduino AVR Boards')
    elif args[:2] not in (['core', 'install'], ['core', 'update-index']): sys.exit('Unexpected MCU operation')
elif name == 'python3':
    if args[:2] == ['-m', 'venv']:
        dest = pathlib.Path(args[2]) / 'bin'; dest.mkdir(parents=True)
        for exe in ('python', 'rosbags-convert'):
            p = dest / exe
            p.write_text('#!/bin/bash\n[[ "${3:-}" != freeze ]] || echo rosbags==0.11.0\nexit 0\n')
            p.chmod(0o755)
    else: os.execv(os.environ['TEST_PYTHON'], [os.environ['TEST_PYTHON']] + args)
elif name == 'colcon':
    if os.environ.get('TEST_BUILD_FAIL'): sys.exit(14)
    for package, names in {'ov_msckf':['run_subscribe_msckf'], 'ov_hw290':[
        'hw290_imu','stereo_splitter','calibration_screen','build_rectified_profile','check_mount_calibration']}.items():
        for binary in names:
            p = root / 'install_vio' / package / 'lib' / package / binary
            p.parent.mkdir(parents=True, exist_ok=True); p.write_text('#!/bin/bash\nexit 0\n'); p.chmod(0o755)
    (root / 'install_vio/setup.bash').write_text('# fake overlay\n')
elif name == 'git':
    if args[0] == 'init': (pathlib.Path(args[1]) / '.git').mkdir(parents=True)
    if 'fetch' in args:
        p = pathlib.Path(args[1]) / 'Dockerfile_ros1_20_04'
        p.write_text('FROM fake\nRUN catkin build -j$(nproc)\n')
    if 'rev-parse' in args:
        if '--verify' in args and not (pathlib.Path(args[1]) / 'Dockerfile_ros1_20_04').exists(): sys.exit(1)
        print('1f60227442d25e36365ef5f72cd80b9666d73467' if 'kalibr-' in args[1] else 'a' * 40)
elif name == 'docker':
    if args == ['info'] and os.environ.get('TEST_DOCKER_PERMISSION') and not os.environ.get('TEST_SUDO'):
        sys.exit(1)
    if args[:2] == ['image', 'inspect']: print('sha256:fake')
elif name not in ('apt-get', 'usermod', 'install', 'systemctl', 'locale-gen', 'add-apt-repository'):
    sys.exit('Unexpected stub: ' + name)
'''


class SetupTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='hw290-setup-test-')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.root = self.base / 'workspace with spaces'
        self.hw = self.root / 'hw290_stereo'
        self.hw.mkdir(parents=True)
        self.bin = self.base / 'bin'
        self.bin.mkdir()
        self.ros = self.base / 'ros_setup.bash'
        self.ros.write_text('# fake ROS environment\n')
        self.os_release = self.base / 'os-release'
        self.os_release.write_text('ID=ubuntu\nVERSION_ID=22.04\nPRETTY_NAME="Ubuntu test"\n')
        text = (HERE / 'setup_hw290_openvins.sh').read_text()
        # Substitute host paths in the fixture only; retain the install logic.
        text = text.replace('ROS_SETUP=/opt/ros/humble/setup.bash', 'ROS_SETUP="$TEST_ROS_SETUP"')
        text = text.replace('source /etc/os-release', 'source "$TEST_OS_RELEASE"')
        text = text.replace('$HOME/.local/bin', '$TEST_USER_BIN')
        (self.hw / 'setup_hw290_openvins.sh').write_text(text)
        shutil.copy(HERE / 'build_calibration_tools.sh', self.hw)
        (self.hw / 'build_camera_driver.sh').write_text('''#!/bin/bash
set -e
echo '["camera-build"]' >> "$TEST_CALLS"
mkdir -p "$TEST_ROOT/install_camera/usb_cam/lib/usb_cam"
printf '#!/bin/bash\nexit 0\n' > "$TEST_ROOT/install_camera/usb_cam/lib/usb_cam/usb_cam_node_exe"
chmod +x "$TEST_ROOT/install_camera/usb_cam/lib/usb_cam/usb_cam_node_exe"
''')
        (self.hw / 'calibration').mkdir()
        self.mount = self.hw / 'calibration/current_mount.yaml'
        self.mount.write_text('untouched calibration\n')
        for name in ('sudo','curl','apt-cache','apt-get','dpkg-query','dpkg','arduino-cli',
                     'python3','colcon','git','docker','usermod','install','systemctl',
                     'locale-gen','add-apt-repository'):
            p = self.bin / name
            p.write_text(STUB); p.chmod(0o755)
        self.calls = self.base / 'calls.jsonl'
        self.env = dict(os.environ, PATH=str(self.bin)+':'+os.environ['PATH'],
                        TEST_ROOT=str(self.root), TEST_CALLS=str(self.calls),
                        TEST_PYTHON=sys.executable, TEST_ROS_SETUP=str(self.ros),
                        TEST_OS_RELEASE=str(self.os_release), TEST_USER_BIN=str(self.bin),
                        CAL_TOOLS=str(self.base / 'cal tools'))

    def run_setup(self, *args):
        return subprocess.run(['bash', str(self.hw / 'setup_hw290_openvins.sh'), *args],
                              env=self.env, text=True, capture_output=True, timeout=30)

    def recorded(self):
        return [json.loads(s) for s in self.calls.read_text().splitlines()] if self.calls.exists() else []

    def test_preview_has_no_install_or_network_side_effects(self):
        r = self.run_setup('--dry-run','--with-calibration-tools')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertFalse(self.calls.exists())
        self.assertFalse((self.root / 'benchmark').exists())

    def test_invalid_jobs_and_conflicting_modes_fail_before_install(self):
        for args in (('--jobs','0'),('--jobs','2; touch bad'),('--jobs',),('--check','--dry-run')):
            self.assertNotEqual(self.run_setup(*args).returncode, 0)
        self.assertFalse(self.calls.exists())

    def test_wrong_os_is_rejected(self):
        self.os_release.write_text('ID=ubuntu\nVERSION_ID=24.04\n')
        self.assertNotEqual(self.run_setup('--dry-run').returncode, 0)
        self.assertFalse(self.calls.exists())

    @unittest.skipIf(os.geteuid() == 0, 'Installer deliberately rejects root accounts')
    def test_full_flow_bootstraps_ros_and_does_not_flash_or_recalibrate(self):
        self.env['TEST_BOOTSTRAP'] = '1'
        r = self.run_setup()
        self.assertEqual(r.returncode, 0, r.stdout+r.stderr)
        calls = self.recorded()
        self.assertTrue(any(c[0]=='curl' and any('ros2-apt-source' in x for x in c) for c in calls))
        self.assertTrue(any(c[0]=='colcon' for c in calls))
        self.assertIn(['camera-build'], calls)
        self.assertFalse(any(c[:2]==['arduino-cli','upload'] for c in calls))
        self.assertEqual(self.mount.read_text(), 'untouched calibration\n')
        self.assertIn('Setup complete.', r.stdout)

    @unittest.skipIf(os.geteuid() == 0, 'Installer deliberately rejects root accounts')
    def test_build_failure_stops_before_camera_or_success(self):
        self.env['TEST_BUILD_FAIL'] = '1'
        r = self.run_setup('--skip-system')
        self.assertEqual(r.returncode, 14, r.stdout+r.stderr)
        self.assertNotIn(['camera-build'], self.recorded())
        self.assertNotIn('Setup complete.', r.stdout)
        self.assertIn('OpenVINS and sensor build', r.stdout+r.stderr)

    @unittest.skipIf(os.geteuid() == 0, 'Installer deliberately rejects root accounts')
    def test_calibration_flow_uses_sudo_docker_when_account_has_no_access(self):
        self.env['TEST_DOCKER_PERMISSION'] = '1'
        r = self.run_setup('--with-calibration-tools')
        self.assertEqual(r.returncode, 0, r.stdout+r.stderr)
        calls = self.recorded()
        self.assertEqual(sum(c[:2]==['docker','build'] for c in calls), 2)
        self.assertEqual(sum(c[:2]==['docker','run'] for c in calls), 2)
        self.assertTrue(any(c[:3]==['sudo','docker','info'] for c in calls))

    def test_readiness_failure_is_read_only(self):
        r = self.run_setup('--check')
        self.assertNotEqual(r.returncode, 0)
        self.assertFalse(any(c[0] in ('sudo','curl','colcon','apt-get') for c in self.recorded()))
        self.assertFalse((self.root / 'benchmark').exists())


if __name__ == '__main__':
    unittest.main()
