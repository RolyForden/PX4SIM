"""在 Gazebo 中执行 X→H→X→V→X，并记录实际关节角、旋翼位置和轴向。

只控制运动学台架，不启动 PX4，不解锁，不飞行。
"""
import datetime
import json
import math
import os
import pathlib
import subprocess
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / 'config' / 'morph-config.json').read_text(encoding='utf-8'))
CONTAINER = os.environ.get('PX4SIM_CONTAINER', 'px4-rtl-land-harmonic')
PARTITION = os.environ.get('PX4SIM_GZ_PARTITION', 'a_morph_bench')
WORLD = os.environ.get('PX4SIM_MORPH_WORLD', 'a_morph_bench')
TOPIC = f'/world/{WORLD}/dynamic_pose/info'
NAMES = ['yaw_0', 'pitch_0', 'yaw_1', 'pitch_1', 'yaw_2', 'pitch_2', 'yaw_3', 'pitch_3']
TOL_DEG = 2.5


def gz_cmd(*args):
    return subprocess.run(
        ['docker', 'exec', '-e', f'GZ_PARTITION={PARTITION}', CONTAINER, 'gz', *args],
        check=True, capture_output=True, text=True, timeout=10)


def read_pose():
    out = gz_cmd('topic', '-e', '-t', TOPIC, '-n', '1', '--json-output').stdout
    return json.JSONDecoder().raw_decode(out)[0]


def quat_to_matrix(q):
    x = float(q.get('x', 0.0)); y = float(q.get('y', 0.0))
    z = float(q.get('z', 0.0)); w = float(q.get('w', 1.0))
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def mat_vec(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


def actual_state(data):
    poses = {p.get('name'): p for p in data.get('pose', [])}
    joints = {}
    rotors = []
    for i, arm in enumerate(CONFIG['arms']):
        arm_pose = poses[f'arm_{i}']
        rotor_pose = poses[f'rotor_{i}']
        R = quat_to_matrix(arm_pose.get('orientation', {}))
        hinge = arm['hinge_flu_m']
        rotor0 = CONFIG['rotors'][i]['position_flu_m']
        u0 = [rotor0[j] - hinge[j] for j in range(3)]
        norm = math.sqrt(sum(v * v for v in u0))
        u0 = [v / norm for v in u0]
        u = mat_vec(R, u0)
        yaw = math.atan2(u[1], u[0]) - math.atan2(u0[1], u0[0])
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        pitch = math.asin(max(-1.0, min(1.0, u[2]))) - math.asin(max(-1.0, min(1.0, u0[2])))
        joints[f'yaw_{i}'] = yaw
        joints[f'pitch_{i}'] = pitch
        up = mat_vec(R, [0.0, 0.0, 1.0])
        rp = rotor_pose.get('position', {})
        rp = [float(rp.get(k, 0.0)) for k in ('x', 'y', 'z')]
        rotors.append({
            'index': i,
            'position_flu_m': rp,
            'position_frd_m': [rp[0], -rp[1], -rp[2]],
            'axis_flu': up,
            'axis_frd': [up[0], -up[1], -up[2]],
        })
    return joints, rotors


def target(mode):
    values = {}
    for arm in CONFIG['arms']:
        values[f"yaw_{arm['index']}"] = arm[mode]['yaw_rad']
        values[f"pitch_{arm['index']}"] = arm[mode]['pitch_rad']
    return values


def send(mode):
    for name, value in target(mode).items():
        gz_cmd('topic', '-t', f'/a_uav/joint/{name}/cmd_pos',
               '-m', 'gz.msgs.Double', '-p', f'data: {value:.9f}')


def wait_for(mode, timeout=16.0):
    tgt = target(mode)
    end = time.monotonic() + timeout
    stable_since = None
    latest = None
    while time.monotonic() < end:
        data = read_pose()
        joints, rotors = actual_state(data)
        err = max(abs(joints[k] - tgt[k]) for k in NAMES)
        latest = (data, joints, rotors, err)
        if err < math.radians(TOL_DEG):
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= 1.0:
                return latest, True
        else:
            stable_since = None
        time.sleep(0.25)
    return latest, False


def snapshot(name, data, joints, rotors, err, curve):
    path = ROOT / 'results' / f'pose-{name}.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')
    return {
        'mode': name,
        'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'target_deg': {k: math.degrees(v) for k, v in target(name).items()},
        'actual_deg': {k: math.degrees(v) for k, v in joints.items()},
        'max_error_deg': math.degrees(err),
        'converged': bool(err < math.radians(TOL_DEG)),
        'rotors': rotors,
        'raw_pose': str(path.relative_to(ROOT)),
        'trajectory': curve,
    }


def main():
    results = []
    requested = []

    def run_mode(name):
        requested.append(name)
        send(name)
        (data, joints, rotors, err), ok = wait_for(name)
        results.append(snapshot(name, data, joints, rotors, err, requested))
        print(json.dumps({'mode': name, 'converged': ok, 'max_error_deg': math.degrees(err)}, ensure_ascii=False))

    # Reset first, then execute the requested sequence.
    run_mode('X')
    run_mode('H')
    run_mode('X')
    run_mode('V')
    run_mode('X')

    out = {
        'status': 'A kinematic bench actual feedback; NOT flight or final-aircraft validation',
        'tolerance_deg': TOL_DEG,
        'sequence': results,
        'config': 'config/morph-config.json',
        'source': 'Gazebo dynamic_pose/info readback through docker exec; temporary ROS/Gazebo adapter',
    }
    (ROOT / 'results' / 'sequence.json').write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'sequence': [r['mode'] for r in results], 'all_converged': all(r['converged'] for r in results)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
