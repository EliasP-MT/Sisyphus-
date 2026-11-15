import sys
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from postal_station.tasks import TaskRunner  # noqa: E402


class FakeStation:
    """Records the order of side actions and arm legs; no ROS."""
    def __init__(self, fail_leg=None):
        self.log, self.fail_leg, self.go_handle, self.go_result = [], fail_leg, None, None
        self.lock = __import__('threading').Lock()
        self.gripper = 1500
        self.cameras = {1: object(), 2: object()}
        self.motors = types.SimpleNamespace(motors=[types.SimpleNamespace(role='gripper', ever_received=True,
                                                                          position_ticks=1500)])

    def call(self, service, timeout=0., **f):
        self.log.append((service, f))
        if service == 'plan':
            return types.SimpleNamespace(success=True, executable=True, plan_id='plan-' + f['pose_name'],
                                         duration_s=1., message='', reasons=[])
        if service == 'gripper':
            self.motors.motors[0].position_ticks = f['ticks']
        return types.SimpleNamespace(success=True, message='ok')

    def execute(self, plan_id, gripper):
        self.log.append(('execute', plan_id))
        self.go_result = {'success': plan_id != self.fail_leg, 'message': 'arrived ' + plan_id}
        return {'success': True, 'message': 'Motion started'}


def fake_ocr(runner, name='Jan Smith'):
    def read_card(camera=None):
        runner.ocr = {'success': True, 'name': name, 'camera': camera or runner.ocr_camera}
        return f'name: {name}'
    runner.read_card = read_card


def wait(runner, phases, timeout=10.):
    end = time.time() + timeout
    while time.time() < end:
        if runner.snapshot()['phase'] in phases and not runner.busy():
            return runner.snapshot()
        time.sleep(.02)
    raise AssertionError(runner.snapshot())


class TaskTests(unittest.TestCase):
    def test_plans_everything_then_runs_in_order(self):
        s = FakeStation()
        r = TaskRunner(s)
        fake_ocr(r)
        r.prepare('pick_scan', 1.5)
        snap = wait(r, {'ready', 'failed'})
        self.assertEqual(snap['phase'], 'ready', snap['message'])
        plans = [f for n, f in s.log if n == 'plan']
        self.assertEqual([p['pose_name'] for p in plans], ['before_grab', 'grab', 'pull', 'up_after_pull', 'read_card'])
        self.assertEqual([p['start_pose_name'] for p in plans], ['scan', 'before_grab', 'grab', 'pull', 'up_after_pull'])
        self.assertFalse(any(n == 'execute' for n, _ in s.log))  # nothing moved while planning
        s.log.clear()
        s.go_result = None
        orig = s.execute

        def execute(plan_id, gripper):
            s.go_result = None
            return orig(plan_id, gripper)
        s.execute = execute
        self.assertTrue(r.start()['success'])
        snap = wait(r, {'done', 'failed', 'canceled'})
        self.assertEqual(snap['phase'], 'done', snap['message'])
        self.assertIn('card name: Jan Smith', snap['message'])
        self.assertEqual(snap['steps'][-1]['detail'], 'name: Jan Smith')
        names = [n for n, _ in s.log]
        order = [(n, f) for n, f in s.log if n in ('gripper', 'dispense', 'execute')]
        self.assertEqual([x[0] for x in order].count('execute'), 5)
        self.assertEqual(order[0], ('gripper', {'ticks': 2300}))
        self.assertIn(('dispense', {'duration_s': 4.5}), order)
        self.assertIn(('gripper', {'ticks': 1270}), order)
        self.assertEqual([x[1] for x in order if x[0] == 'execute'],
                         ['plan-before_grab', 'plan-grab', 'plan-pull', 'plan-up_after_pull', 'plan-read_card'])
        self.assertLess(order.index(('gripper', {'ticks': 1270})),
                        order.index(('execute', 'plan-pull')))

    def test_failure_stops_dispenser_and_nothing_follows(self):
        s = FakeStation(fail_leg='plan-grab')
        r = TaskRunner(s)
        fake_ocr(r)
        r.prepare('pick_scan', 1.)
        wait(r, {'ready'})
        s.log.clear()
        r.start()
        snap = wait(r, {'failed', 'done'})
        self.assertEqual(snap['phase'], 'failed')
        names = [n for n, _ in s.log]
        self.assertIn('dispense_stop', names)
        self.assertNotIn(('gripper', {'ticks': 1270}), s.log)
        self.assertNotIn(('execute', 'plan-pull'), s.log)

    def test_start_requires_planning(self):
        self.assertFalse(TaskRunner(FakeStation()).start()['success'])


if __name__ == '__main__':
    unittest.main()
