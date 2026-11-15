"""动态变构链地面联调节点。

关节反馈 -> 机臂运动学 -> 当前 COM/旋翼位置/推力轴 -> B 矩阵 -> 有界悬停分配。
输出 JSON 供后续 PX4 适配器消费；本阶段不向飞控发送执行器输出。
"""
import itertools
import json
import math
import os
import pathlib
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ROOT = pathlib.Path(__file__).resolve().parents[2]
CONFIG = json.loads((ROOT / 'config' / 'morph-config.json').read_text(encoding='utf-8'))
LOG_PATH = pathlib.Path(os.environ.get('PX4SIM_DYNAMIC_LOG', str(ROOT / 'dynamic' / 'results' / 'ground-integration.jsonl')))
LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
NAMES = ['yaw_0', 'pitch_0', 'yaw_1', 'pitch_1', 'yaw_2', 'pitch_2', 'yaw_3', 'pitch_3']
MAX_THRUST = CONFIG['rotors'][0]['motor_constant_N_per_rad_s2'] * CONFIG['rotors'][0]['max_rad_s'] ** 2


def rot_axis(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def allocate(B, target, lower, upper, weights):
    B = np.asarray(B, dtype=float)
    target = np.asarray(target, dtype=float)
    n = B.shape[1]
    lower = np.broadcast_to(lower, (n,))
    upper = np.broadcast_to(upper, (n,))
    weights = np.asarray(weights, dtype=float)
    best = None
    for active in itertools.product([-1, 0, 1], repeat=n):
        active = np.array(active)
        fixed = active != 0
        free = ~fixed
        t = np.where(active < 0, lower, upper).copy()
        rhs = target - B[:, fixed] @ t[fixed]
        if free.any():
            t[free] = np.linalg.lstsq(weights[:, None] * B[:, free], weights * rhs, rcond=None)[0]
        if np.any(t < lower - 1e-9) or np.any(t > upper + 1e-9):
            continue
        residual = B @ t - target
        score = float(np.linalg.norm(weights * residual))
        if best is None or score < best[0] - 1e-12:
            best = (score, t.copy(), residual)
    if best is None:
        raise ValueError('Invalid thrust bounds')
    return best


class DynamicChain(Node):
    def __init__(self):
        super().__init__('a_morph_dynamic')
        self.q = None
        self.stamp_time = None
        self.last_recv = None
        self.prev_recv = None
        self.last_update = None
        self.prev_update = None
        self.updates = 0
        self.create_subscription(JointState, '/a_uav/joint_states', self.on_joint, 10)
        self.pub_matrix = self.create_publisher(String, '/a_uav/allocation/matrix', 10)
        self.pub_thrust = self.create_publisher(String, '/a_uav/allocation/motor_thrust', 10)
        self.pub_status = self.create_publisher(String, '/a_uav/allocation/status', 10)
        self.create_timer(0.05, self.tick)
        self.log = LOG_PATH.open('w', encoding='utf-8')
        self.get_logger().info(f'Dynamic morphing chain 20 Hz; log={LOG_PATH}')

    def on_joint(self, msg):
        if len(msg.position) < len(NAMES):
            return
        self.q = np.array(msg.position[:len(NAMES)], dtype=float)
        self.stamp_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = time.monotonic()
        self.prev_recv = self.last_recv
        self.last_recv = now

    def mode_of(self):
        candidates = {}
        for mode in ('X', 'H', 'V'):
            target = []
            for arm in CONFIG['arms']:
                target.extend([arm[mode]['yaw_rad'], arm[mode]['pitch_rad']])
            candidates[mode] = max(abs(a - b) for a, b in zip(self.q, target))
        best = min(candidates, key=candidates.get)
        return (best if candidates[best] < math.radians(3) else 'TRANSITION'), {k: math.degrees(v) for k, v in candidates.items()}

    def compute(self):
        coms = [CONFIG['body']['mass_kg'] * np.asarray(CONFIG['body']['com_flu_m'], dtype=float)]
        masses = [CONFIG['body']['mass_kg']]
        rotor_positions = []
        rotor_axes = []
        for i, arm in enumerate(CONFIG['arms']):
            yaw = self.q[2 * i]
            pitch = self.q[2 * i + 1]
            hinge = np.asarray(arm['hinge_flu_m'], dtype=float)
            R = rot_axis([0, 0, 1], yaw) @ rot_axis(arm['pitch_axis'], pitch)
            com = hinge + R @ (np.asarray(arm['com_flu_m'], dtype=float) - hinge)
            coms.append(arm['mass_kg'] * com)
            masses.append(arm['mass_kg'])
            rotor0 = np.asarray(CONFIG['rotors'][i]['position_flu_m'], dtype=float)
            rotor_positions.append(hinge + R @ (rotor0 - hinge))
            rotor_axes.append(R @ np.array([0.0, 0.0, 1.0]))
        for rp in rotor_positions:
            coms.append(CONFIG['rotor_mass_kg'] * rp)
            masses.append(CONFIG['rotor_mass_kg'])
        total_mass = sum(masses)
        com_flu = sum(coms, np.zeros(3)) / total_mass
        r = np.array([[(rp - com_flu)[0], -(rp - com_flu)[1], -(rp - com_flu)[2]] for rp in rotor_positions])
        d = np.array([[axis[0], -axis[1], -axis[2]] for axis in rotor_axes])
        d = d / np.linalg.norm(d, axis=1, keepdims=True)
        km = np.array([CONFIG['rotors'][i]['km_m'] for i in range(4)])
        B = np.vstack([(np.cross(r, d) - km[:, None] * d).T, d.T])
        target = np.array([0, 0, 0, 0, 0, -total_mass * 9.8])
        score, thrust, residual = allocate(B, target, 0, MAX_THRUST, np.array([5, 5, 5, 1, 1, 1.0]))
        return total_mass, com_flu, rotor_positions, rotor_axes, B, thrust, residual, score

    def tick(self):
        if self.q is None or self.last_recv is None:
            return
        now = time.monotonic()
        if now - self.last_recv > 1.5:
            self.pub_status.publish(String(data=json.dumps({'valid': False, 'reason': 'stale joint state'}, ensure_ascii=False)))
            return
        started = time.perf_counter()
        total_mass, com, rotor_pos, rotor_axis, B, thrust, residual, score = self.compute()
        compute_ms = (time.perf_counter() - started) * 1000
        mode, errors = self.mode_of()
        input_age_ms = (time.time() - self.stamp_time) * 1000 if self.stamp_time else None
        input_period_ms = (self.last_recv - self.prev_recv) * 1000 if self.prev_recv else None
        matrix_period_ms = (now - self.prev_update) * 1000 if self.prev_update else None
        self.prev_update = now
        self.updates += 1
        record = {
            'utc': time.time(),
            'mode': mode,
            'mode_error_deg': errors,
            'joints_deg': {NAMES[i]: math.degrees(self.q[i]) for i in range(len(NAMES))},
            'com_flu_m': com.tolist(),
            'rotors': [{'position_flu_m': p.tolist(), 'axis_flu': a.tolist()} for p, a in zip(rotor_pos, rotor_axis)],
            'B_tau_xyz_force_xyz_per_N': B.tolist(),
            'rank': int(np.linalg.matrix_rank(B)),
            'condition_number': float(np.linalg.cond(B)),
            'hover_thrust_N': thrust.tolist(),
            'hover_residual': residual.tolist(),
            'hover_weighted_residual_norm': score,
            'input_age_ms': input_age_ms,
            'input_period_ms': input_period_ms,
            'matrix_period_ms': matrix_period_ms,
            'compute_ms': compute_ms,
            'chain_latency_ms': (time.monotonic() - self.last_recv) * 1000,
            'update_index': self.updates,
        }
        self.log.write(json.dumps(record, ensure_ascii=False) + '\n')
        self.log.flush()
        self.pub_matrix.publish(String(data=json.dumps({k: record[k] for k in ('mode', 'joints_deg', 'com_flu_m', 'B_tau_xyz_force_xyz_per_N', 'rank', 'condition_number')}, ensure_ascii=False)))
        self.pub_thrust.publish(String(data=json.dumps({'mode': mode, 'thrust_N': record['hover_thrust_N'], 'residual_norm': score, 'max_thrust_N': MAX_THRUST}, ensure_ascii=False)))
        status = {
            'valid': True, 'mode': mode, 'input_age_ms': input_age_ms, 'input_period_ms': input_period_ms,
            'matrix_period_ms': matrix_period_ms, 'compute_ms': compute_ms,
            'chain_latency_ms': record['chain_latency_ms'], 'update_index': self.updates,
        }
        self.pub_status.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main():
    rclpy.init()
    node = DynamicChain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
