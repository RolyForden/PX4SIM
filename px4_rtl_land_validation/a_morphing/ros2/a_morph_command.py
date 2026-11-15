"""A 台架构形指令节点。

订阅 /a_uav/morph/command（String: X/H/V），通过 gz transport 设置等效关节目标。
所有目标都是可替换的近似假设；节点不启动 PX4，不触发飞行。
"""
import json
import math
import os
import pathlib
import subprocess
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / 'config' / 'morph-config.json').read_text(encoding='utf-8'))
CONTAINER = os.environ.get('PX4SIM_CONTAINER', 'px4-rtl-land-harmonic')
PARTITION = os.environ.get('PX4SIM_GZ_PARTITION', 'a_morph_bench')
NAMES = ['yaw_0', 'pitch_0', 'yaw_1', 'pitch_1', 'yaw_2', 'pitch_2', 'yaw_3', 'pitch_3']


class MorphCommand(Node):
    def __init__(self):
        super().__init__('a_morph_command')
        self.group = ReentrantCallbackGroup()
        self.create_subscription(String, '/a_uav/morph/command', self.on_command, 10, callback_group=self.group)
        self.create_subscription(JointState, '/a_uav/joint_states', self.on_joint_state, 10, callback_group=self.group)
        self.pub = self.create_publisher(String, '/a_uav/morph/command_status', 10)
        self.latest = None
        self.latest_time = None
        self.busy = False
        self.get_logger().info('Send X, H, or V on /a_uav/morph/command')

    def on_joint_state(self, msg):
        if len(msg.position) >= len(NAMES):
            self.latest = list(msg.position)
            self.latest_time = time.monotonic()

    def targets(self, mode):
        values = []
        for arm in CONFIG['arms']:
            values.extend([arm[mode]['yaw_rad'], arm[mode]['pitch_rad']])
        return values

    def send_joint(self, name, value):
        cmd = [
            'docker', 'exec', '-e', f'GZ_PARTITION={PARTITION}', CONTAINER,
            'gz', 'topic', '-t', f'/a_uav/joint/{name}/cmd_pos',
            '-m', 'gz.msgs.Double', '-p', f'data: {float(value):.9f}',
        ]
        subprocess.run(cmd, check=True, capture_output=True, text=True, timeout=8)

    def apply(self, mode):
        target = self.targets(mode)
        for name, value in zip(NAMES, target):
            self.send_joint(name, value)

    def wait_near(self, target, timeout=12.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.latest is not None and self.latest_time is not None:
                if time.monotonic() - self.latest_time < 1.5:
                    if max(abs(a - b) for a, b in zip(self.latest, target)) < math.radians(2.0):
                        return True
            time.sleep(0.2)
        return False

    def status(self, **kw):
        msg = String()
        msg.data = json.dumps(kw, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(msg.data)

    def on_command(self, msg):
        mode = msg.data.strip().upper()
        if mode not in ('X', 'H', 'V'):
            self.status(state='rejected', reason='command must be X, H, or V', command=msg.data)
            return
        if self.busy:
            self.status(state='busy', command=mode)
            return
        self.busy = True
        try:
            self.status(state='transition', command=mode, phase='reset_to_X')
            self.apply('X')
            self.wait_near(self.targets('X'), timeout=8.0)
            if mode != 'X':
                self.status(state='transition', command=mode, phase='go_to_target')
                self.apply(mode)
                ok = self.wait_near(self.targets(mode), timeout=14.0)
            else:
                ok = True
            self.status(state='done' if ok else 'timeout', command=mode,
                        verified=bool(ok), target_rad=dict(zip(NAMES, self.targets(mode))))
        except Exception as exc:
            self.status(state='error', command=mode, error=str(exc))
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = MorphCommand()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
