"""读取 Gazebo 实际连杆位姿，发布 A 台架关节状态和构形状态。

当前主机 ros_gz_bridge 是 Fortress 版本，不能直接桥接容器内 Harmonic，因此这里
用 docker exec 调用 gz transport 作为明确标注的临时适配。它不是最终 ROS-Gazebo 桥。
"""
import json
import math
import os
import pathlib
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / 'config' / 'morph-config.json').read_text(encoding='utf-8'))
CONTAINER = os.environ.get('PX4SIM_CONTAINER', 'px4-rtl-land-harmonic')
PARTITION = os.environ.get('PX4SIM_GZ_PARTITION', 'a_morph_bench')
WORLD = os.environ.get('PX4SIM_MORPH_WORLD', 'a_morph_bench')
TOPIC = f'/world/{WORLD}/dynamic_pose/info'
NAMES = ['yaw_0', 'pitch_0', 'yaw_1', 'pitch_1', 'yaw_2', 'pitch_2', 'yaw_3', 'pitch_3']


def quat_to_matrix(q):
    x = float(q.get('x', 0.0))
    y = float(q.get('y', 0.0))
    z = float(q.get('z', 0.0))
    w = float(q.get('w', 1.0))
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def mat_vec(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


class MorphState(Node):
    def __init__(self):
        super().__init__('a_morph_state')
        self.pub_joint = self.create_publisher(JointState, '/a_uav/joint_states', 10)
        self.pub_status = self.create_publisher(String, '/a_uav/morph/status', 10)
        self.last_positions = None
        self.last_time = None
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'Reading actual Gazebo poses from {TOPIC} via container {CONTAINER} '
            f'(partition {PARTITION}); temporary adapter, not ros_gz_bridge.')

    def read_pose(self):
        cmd = [
            'docker', 'exec', '-e', f'GZ_PARTITION={PARTITION}', CONTAINER,
            'gz', 'topic', '-e', '-t', TOPIC, '-n', '1', '--json-output',
        ]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=6, check=True).stdout
            return json.JSONDecoder().raw_decode(out)[0]
        except Exception as exc:
            self.get_logger().warning(f'Gazebo pose read failed: {exc}', throttle_duration_sec=2.0)
            return None

    def tick(self):
        data = self.read_pose()
        if not data:
            return
        poses = {p.get('name'): p for p in data.get('pose', [])}
        positions = []
        rotors = []
        joint_deg = {}
        for i, arm in enumerate(CONFIG['arms']):
            arm_pose = poses.get(f'arm_{i}')
            rotor_pose = poses.get(f'rotor_{i}')
            if not arm_pose or not rotor_pose:
                return
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
            positions.extend([yaw, pitch])
            joint_deg[f'yaw_{i}'] = math.degrees(yaw)
            joint_deg[f'pitch_{i}'] = math.degrees(pitch)
            up_fl = mat_vec(R, [0.0, 0.0, 1.0])
            rp = rotor_pose.get('position', {})
            rp = [float(rp.get(k, 0.0)) for k in ('x', 'y', 'z')]
            rotors.append({
                'index': i,
                'position_flu_m': rp,
                'position_frd_m': [rp[0], -rp[1], -rp[2]],
                'axis_flu': up_fl,
                'axis_frd': [up_fl[0], -up_fl[1], -up_fl[2]],
            })

        now = time.monotonic()
        velocities = [0.0] * len(positions)
        if self.last_positions is not None and self.last_time is not None:
            dt = max(now - self.last_time, 1e-6)
            velocities = [(a - b) / dt for a, b in zip(positions, self.last_positions)]
        self.last_positions = positions
        self.last_time = now

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = NAMES
        js.position = positions
        js.velocity = velocities
        self.pub_joint.publish(js)

        candidates = {}
        for mode in ('X', 'H', 'V'):
            target = []
            for arm in CONFIG['arms']:
                target.extend([arm[mode]['yaw_rad'], arm[mode]['pitch_rad']])
            candidates[mode] = max(abs(a - b) for a, b in zip(positions, target))
        best = min(candidates, key=candidates.get)
        mode = best if candidates[best] < math.radians(3.0) else 'TRANSITION'
        status = {
            'mode': mode,
            'actual': True,
            'approximate_model': True,
            'joints_rad': {NAMES[i]: positions[i] for i in range(len(NAMES))},
            'joints_deg': joint_deg,
            'rotors': rotors,
            'max_target_error_deg': {k: math.degrees(v) for k, v in candidates.items()},
            'source': 'Gazebo dynamic_pose readback through docker exec; temporary adapter',
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.pub_status.publish(msg)


def main():
    rclpy.init()
    node = MorphState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
