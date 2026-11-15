"""固定 H/V 独立 SITL 起飞/悬停/Land 测试；只连接本机回环，不连接真实飞控。"""
import argparse
import datetime
import json
import math
import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1]))
import validate  # noqa: E402
from pymavlink import mavutil  # noqa: E402

validate.ROOT = ROOT
validate.LOGS = ROOT / 'logs'
validate.RESULTS = ROOT / 'results'
validate.PARAMS = ROOT / 'parameters'
for d in (validate.LOGS, validate.PARAMS):
    d.mkdir(parents=True, exist_ok=True)
RUNTIME_LOG = ROOT.parents[1] / 'candidate_flight' / 'runtime' / 'log'


class FixedFlight(validate.Flight):
    expected_airframe = 0

    def pump(self, duration=.1, setpoint=None):
        super().pump(duration, setpoint)
        if getattr(self, 'monitor_flight', False) and self.state.get('armed'):
            s = self.state
            o = self.origin
            if (max(abs(s.get('roll', 0)), abs(s.get('pitch', 0))) > math.radians(35)
                    or math.hypot(s.get('x', o['x']) - o['x'], s.get('y', o['y']) - o['y']) > 8
                    or abs(s.get('z', o['z']) - o['z']) > 15):
                self.monitor_flight = False
                raise RuntimeError('Fixed flight exceeded attitude/position test envelope')

    def snapshot(self, tag):
        self.link.mav.param_request_list_send(1, 1)
        self.pump(12)
        assert self.params.get('SYS_AUTOSTART') == self.expected_airframe, 'Unexpected airframe ID'
        (validate.PARAMS / f'{self.name}_{tag}.json').write_text(json.dumps(self.params, indent=2, sort_keys=True))
        self.event('parameters', airframe=self.params['SYS_AUTOSTART'])

    def run_fixed(self):
        self.snapshot('before')
        self.wait('ready_ground', lambda: self.state.get('landed') == 1 and not self.state.get('armed'), timeout=60)
        origin = {k: self.state[k] for k in ['x', 'y', 'z', 'alt']}
        self.event('origin', origin=origin)
        self.origin = origin
        self.monitor_flight = True
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1])
        self.wait('armed', lambda: self.state.get('armed'), timeout=10)
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     [0, 0, 0, float('nan'), float('nan'), float('nan'), origin['alt'] + 5])

        def hover():
            s = self.state
            return (abs(s.get('z', 999) - (origin['z'] - 5)) < .5
                    and abs(s.get('vz', 999)) < .3
                    and math.hypot(s.get('x', 999) - origin['x'], s.get('y', 999) - origin['y']) < 1
                    and max(abs(s.get('roll', 9)), abs(s.get('pitch', 9))) < math.radians(10))

        self.wait('hover_confirmed', hover, timeout=90, stable=8)
        self.event('land_trigger')
        self.mode(4, 6)
        self.wait('ground_detected', lambda: self.state.get('landed') == 1, timeout=120)
        self.wait('auto_disarmed', lambda: not self.state.get('armed'), timeout=30)
        self.snapshot('after')
        self.event('test_complete')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--airframe', required=True, type=int)
    parser.add_argument('--name-prefix', default=None)
    args = parser.parse_args()
    config_path = ROOT / 'config' / f'{args.model}.json'
    if not config_path.exists():
        raise SystemExit(f'missing config: {config_path}')
    top = subprocess.check_output(['docker', 'top', 'px4-rtl-land-harmonic', '-eo', 'pid,args'], text=True)
    assert '/work/a_morphing/fixed_flight/startup.sh' in top, 'Fixed startup not confirmed'
    assert '/work/candidate_flight/runtime' in top, 'Runtime not confirmed'
    warm = mavutil.mavlink_connection('udpin:127.0.0.1:14650')
    assert warm.wait_heartbeat(timeout=45) is not None, 'Heartbeat timeout'
    warm.close()
    prefix = args.name_prefix or args.model
    name = f'{prefix}_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    (ROOT / 'logs' / f'{name}_model_config.json').write_text(config_path.read_text(encoding='utf-8'), encoding='utf-8')
    shutil.copyfile(ROOT / 'models' / args.model / 'model.sdf', ROOT / 'logs' / f'{name}_model.sdf')
    shutil.copyfile(ROOT.parents[1] / 'candidate_flight' / 'candidate_world.sdf', ROOT / 'logs' / f'{name}_world.sdf')
    FixedFlight.expected_airframe = args.airframe
    f = FixedFlight(name)
    try:
        f.run_fixed()
    except Exception as exc:
        f.monitor_flight = False
        f.event('test_failed', error=str(exc))
        if f.state.get('armed'):
            try:
                f.command(mavutil.mavlink.MAV_CMD_NAV_LAND,
                          [0, 0, 0, float('nan'), float('nan'), float('nan'), float('nan')])
                f.wait('recovery_disarmed', lambda: not f.state.get('armed'), timeout=150)
            except Exception as recovery:
                f.event('recovery_failed', error=str(recovery))
        raise
    finally:
        paths = list(RUNTIME_LOG.rglob('*.ulg'))
        if paths:
            source = max(paths, key=lambda p: p.stat().st_mtime)
            shutil.copyfile(source, validate.LOGS / f'{name}.ulg')
        (ROOT / 'last_run.json').write_text(json.dumps(
            {'name': name, 'model': args.model, 'airframe': args.airframe,
             'events': f.events,
             'status': 'passed' if f.events[-1]['event'] == 'test_complete' else 'failed'}, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
