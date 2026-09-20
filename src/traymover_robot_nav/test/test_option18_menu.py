"""Exercise option 10/18 menu bringup without launching hardware or ROS."""

import os
from pathlib import Path
import shlex
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[3]


def run_menu(tmp_path, option, *, ground='y', height='2.5', auto=True,
             cleanup_status=0, converter_status=0):
    launcher = (ROOT / 'scripts/traymover.sh').read_text()
    names = ('prompt_ndt_height_boost', 'prompt_wait_detour_settings',
             'action_start_nav', 'action_start_nav_wait_detour')
    functions = '\n'.join(
        name + '() {' + launcher.split(name + '() {', 1)[1].split('\n}', 1)[0] + '\n}'
        for name in names)
    pcd_dir = tmp_path / 'pcd maps'
    pcd_dir.mkdir(parents=True)
    (pcd_dir / "map's example.pcd").write_text('fixture')
    setup = tmp_path / 'setup.bash'
    setup.write_text(':\n')
    env = dict(os.environ, FASTLIO_PCD_DIR=str(pcd_dir), WORKSPACE_DIR=str(ROOT),
               SCRIPT_DIR=str(ROOT / 'scripts'),
               TRAYMOVER_NAV_LOG_ROOT=str(tmp_path / 'nav logs'),
               NAV_RUNTIME_DIR=str(tmp_path / 'maps'),
               ROS_LOG_DIR_DEFAULT=str(tmp_path / 'logs'),
               ROS_DISTRO_SETUP=str(setup), WS_SETUP=str(setup),
               NAV_SCRIPTS_DIR=str(ROOT / 'src/traymover_robot_nav/scripts'),
               NAV_PGM_Z_MIN='0.17', NAV_PGM_Z_MAX='2.20', NAV_PGM_MIN_POINTS='1',
               NAV_PGM_DILATE='2', NAV_PGM_MIN_REGION_CELLS='2',
               AUTO_ESTOP_ENABLED='false', CAPTURE=str(tmp_path / 'events'),
               CLEANUP_STATUS=str(cleanup_status), CONVERTER_STATUS=str(converter_status))
    stubs = r'''
python3() {
    if [[ "$1" == */cleanup_static_tf.py ]]; then
        printf 'cleanup\n' >> "$CAPTURE"
        return "$CLEANUP_STATUS"
    elif [[ "$1" == */pcd2pgm.py ]]; then
        printf 'convert'; printf ' %q' "$@"; printf '\n'
        if [ "$CONVERTER_STATUS" != 0 ]; then return "$CONVERTER_STATUS"; fi
        while [ "$#" -gt 0 ]; do
            if [ "$1" = --out ]; then printf 'image: fixture.pgm\n' > "${2}.yaml"; break; fi
            shift
        done
    else
        command python3 "$@"
    fi
}
spawn_in_terminal() { printf '%s\t%s\n' "$1" "$2" >> "$CAPTURE"; }
prompt_auto_estop() { AUTO_ESTOP_ENABLED=false; }
action_start_estop_keyboard() { printf 'keyboard\n' >> "$CAPTURE"; }
action_start_auto_estop() { printf 'standalone_auto_estop\n' >> "$CAPTURE"; }
sleep() { :; }
'''
    answers = ['1', ground, 'n']
    if option == 18:
        answers.extend(['y', 'y', '', '', '', '', ''] if auto else ['n'])
    answers.append(height)
    entrypoint = 'action_start_nav' if option == 10 else 'action_start_nav_wait_detour'
    result = subprocess.run(['bash', '-u', '-c', stubs + functions + '\n' + entrypoint],
                            input='\n'.join(answers) + '\n', env=env,
                            text=True, capture_output=True, timeout=10)
    capture = tmp_path / 'events'
    events = capture.read_text().splitlines() if capture.exists() else []
    runs = list((tmp_path / 'nav logs').glob(f'*_option{option}.*'))
    return result, events, runs


@pytest.mark.parametrize('option', [10, 18])
@pytest.mark.parametrize('ground,reference,height,enabled', [
    ('y', 'plane', '2.5', 'true'), ('n', 'floor', '', 'false'),
])
def test_navigation_baseline_map_localization_and_logs(
        tmp_path, option, ground, reference, height, enabled):
    result, events, runs = run_menu(tmp_path, option, ground=ground, height=height)
    assert result.returncode == 0, result.stdout + result.stderr
    assert len(runs) == 1
    info = dict(line.split('=', 1) for line in (runs[0] / 'run_info.txt').read_text().splitlines())
    assert info['mode'] == f'live_option{option}'
    assert info['z_reference'] == reference
    assert info['height_boost_enabled'] == enabled
    assert info['height_boost_min_z'] == (height or '2.0')
    assert info['height_boost_frame'] == 'base_link'
    assert info['height_boost_extra_copies'] == '1'
    assert info['auto_correction_enabled'] == 'true'
    assert info['use_sim_time'] == 'false'
    assert (runs[0] / 'map.yaml').read_text() == Path(info['map']).read_text()
    for artifact in ('software_commit.txt', 'software_status.txt', 'ros'):
        assert (runs[0] / artifact).exists()
    conversion = shlex.split(next(
        line for line in result.stdout.splitlines() if line.startswith('convert ')))
    assert conversion[conversion.index('--pcd') + 1] == info['pcd']
    for key, value in {'--z-reference': reference, '--z-min': '0.17', '--z-max': '2.20',
                       '--min-points': '1', '--dilate': '2', '--min-region-cells': '2'}.items():
        assert conversion[conversion.index(key) + 1] == value
    assert events[0] == 'cleanup'
    chassis = next(line for line in events if 'base_serial.launch.py' in line)
    assert 'odom_source_mode:=none' in chassis
    if option == 10:
        assert 'auto_estop_enabled:=false' in chassis
        assert 'standalone_auto_estop' in events
    assert 'keyboard' in events
    assert sum('traymover_lidar.launch.py enable_scan_bridge:=false' in line
               for line in events) == 1
    nav = shlex.split(next(line.split('\t', 1)[1] for line in events
                           if 'run_nav_logged.sh' in line))
    assert nav[:3] == ['bash', str(ROOT / 'scripts/run_nav_logged.sh'), str(runs[0])]
    if option == 10:
        assert nav[3:7] == ['ros2', 'launch', 'traymover_robot_nav', 'traymover_nav.launch.py']
        assert not any(arg.startswith('enable_detour:=') for arg in nav)
    for arg in (f'pcd_path:={info["pcd"]}', f'map:={info["map"]}', 'launch_rviz:=false',
                f'height_boost_enabled:={enabled}', f'height_boost_min_z:={height or "2.0"}',
                'auto_correction_enabled:=true'):
        assert arg in nav


@pytest.mark.parametrize('option', [10, 18])
@pytest.mark.parametrize('failure', ['cleanup_status', 'converter_status'])
def test_failed_preflight_never_starts_hardware(tmp_path, option, failure):
    result, events, runs = run_menu(tmp_path, option, **{failure: 1})
    assert result.returncode != 0
    assert events == ['cleanup']
    assert not runs


@pytest.mark.parametrize('auto', [True, False])
def test_option18_camera_and_auto_estop_are_owned_by_main_launch(tmp_path, auto):
    result, events, _ = run_menu(tmp_path, 18, auto=auto)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'standalone_auto_estop' not in events
    commands = '\n'.join(events)
    assert 'realsense2_camera' not in commands
    assert f'auto_estop_enabled:={str(auto).lower()}' in commands
    assert commands.count('traymover_nav_lidar_realsense_detour.launch.py') == 1
    assert f'enable_auto_estop:={str(auto).lower()}' in commands
    assert f'enable_detour:={str(auto).lower()}' in commands
    for arg in ('wait_before_detour_sec:=1.0', 'detour_trigger_distance_m:=1.00',
                'detour_guard_distance_m:=0.90', 'detour_max_linear_speed:=0.08',
                'detour_max_angular_speed:=0.25'):
        assert arg in commands
