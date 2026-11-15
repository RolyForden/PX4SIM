"""启动固定 H/V 独立 SITL；不修改 PX4 源码，不连接真实飞控。"""
import argparse
import json
import pathlib
import shutil
import subprocess

ROOT = pathlib.Path(__file__).resolve().parent
PROJECT = ROOT.parent
WORK = pathlib.Path('/work')
CONTAINER = 'px4-rtl-land-harmonic'
PX = '/work/PX4-Autopilot'
RUNTIME_HOST = PROJECT.parent / 'candidate_flight' / 'runtime'
RUNTIME_WORK = '/work/candidate_flight/runtime'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--airframe', required=True, type=int)
    parser.add_argument('--partition', required=True)
    args = parser.parse_args()

    airframe_src = ROOT / 'airframes' / f'{args.airframe}_{args.model}'
    if not airframe_src.exists():
        raise SystemExit(f'missing airframe file: {airframe_src}')
    runtime_airframes = RUNTIME_HOST / 'etc' / 'init.d-posix' / 'airframes'
    runtime_airframes.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(airframe_src, runtime_airframes / airframe_src.name)

    base = [
        'docker', 'exec', '-e', 'GZ_IP=127.0.0.1', '-e', f'GZ_PARTITION={args.partition}',
        '-e', 'LIBGL_ALWAYS_SOFTWARE=0', '-e', '__GLX_VENDOR_LIBRARY_NAME=nvidia',
        '-e', f'GZ_SIM_RESOURCE_PATH=/work/a_morphing/fixed_flight/models:{PX}/Tools/simulation/gz/models:{PX}/Tools/simulation/gz/worlds',
        '-e', f'GZ_SIM_SYSTEM_PLUGIN_PATH={PX}/build/px4_sitl_default/src/modules/simulation/gz_plugins',
        '-e', f'GZ_SIM_SERVER_CONFIG_PATH={PX}/src/modules/simulation/gz_bridge/server.config',
    ]
    top = subprocess.check_output(['docker', 'top', CONTAINER, '-eo', 'pid,args'], text=True)
    if PX + '/build/px4_sitl_default/bin/px4' in top:
        raise RuntimeError('A PX4 SITL process already exists; stop it first')
    server_cmd = base + [CONTAINER, 'gz', 'sim', '-s', '-r', '/work/candidate_flight/candidate_world.sdf']
    with (ROOT / 'gazebo_server.log').open('w') as f:
        server = subprocess.Popen(server_cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    px4_cmd = base + [
        '-e', f'PX4_SYS_AUTOSTART={args.airframe}', '-e', 'PX4_GZ_STANDALONE=1',
        '-e', f'PX4_GZ_MODELS=/work/a_morphing/fixed_flight/models',
        '-e', 'PX4_GZ_WORLD=candidate_world', '-e', f'PX4_SIM_MODEL={args.model}',
        CONTAINER, 'stdbuf', '-oL', '-eL', PX + '/build/px4_sitl_default/bin/px4',
        '-d', '-s', '/work/a_morphing/fixed_flight/startup.sh',
        '-w', RUNTIME_WORK, RUNTIME_WORK + '/etc',
    ]
    with (ROOT / 'sitl_console.log').open('w') as f:
        px4 = subprocess.Popen(px4_cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    record = {'model': args.model, 'airframe': args.airframe, 'partition': args.partition,
              'server': server_cmd, 'px4': px4_cmd, 'host_processes': [server.pid, px4.pid]}
    (ROOT / 'launch_record.json').write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'model': args.model, 'airframe': args.airframe, 'partition': args.partition,
                      'server_pid': server.pid, 'px4_pid': px4.pid}, ensure_ascii=False))


if __name__ == '__main__':
    main()
