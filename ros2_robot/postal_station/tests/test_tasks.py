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

    # --- what the insert-letter task needs from the station
    scan_ticks = [2026, 1263, 1643, 2908, 1902, 2354, 2226]

    def wait_arm_still(self, *a, **k):
        return True

    def arm_ticks(self):
        return list(self.scan_ticks) if getattr(self, 'arm_at', None) is None else list(self.arm_at)

    def detect_mailboxes(self, camera):
        self.log.append(('detect', camera))
        self.detect_result = {'labels': [
            {'id': 1, 'name': 'Jan Smith', 'slot_base_m': [0.02, 0.33, 0.23], 'approach_dir': [0.08, 0.997, 0.0]},
            {'id': 2, 'name': 'Bob Ross', 'slot_base_m': [0.015, 0.34, 0.14], 'approach_dir': [0.08, 0.997, 0.0]},
            {'id': 3, 'name': 'Luc Marti', 'slot_base_m': None, 'approach_dir': None}]}
        return {'success': True, 'message': '3 labels'}

    def planner_call(self, path, body, timeout=0.):
        self.log.append((path, body))
        if path == '/pivot_ticks':
            goal = list(body['ticks'])
            goal[6] += 1024
            return {'goal_ticks': goal}
        return {'goal_ticks': [1500 + len(self.log)] * 7}

    def call(self, service, timeout=0., **f):
        if service == 'list':
            return types.SimpleNamespace(success=True, poses=[types.SimpleNamespace(name='scan', ticks=self.scan_ticks + [1270])])
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
        # Only `pull` skips the static-error correction (its sag is well within the tolerance);
        # `up_after_pull` sags by more than the tolerance and needs it.
        self.assertEqual({p['pose_name']: p['skip_compensation'] for p in plans},
                         {'before_grab': False, 'grab': False, 'pull': True, 'up_after_pull': False, 'read_card': False})
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

    def insert_runner(self, arm_at=None, **params):
        s = FakeStation()
        s.arm_at = arm_at
        r = TaskRunner(s)
        base = dict(recipient='bob ross', depth_cm=5, front_cm=10, tcp_offset_cm=10.6, pivot_deg=90)
        base.update(params)
        return s, r, r.prepare('insert_letter', 1.0, 1, base)

    def test_insert_letter_plans_every_leg_from_the_scan_pose_then_runs_the_sequence(self):
        s, r, started = self.insert_runner()
        self.assertTrue(started['success'])
        snap = wait(r, {'ready', 'failed'})
        self.assertEqual(snap['phase'], 'ready', snap['message'])
        self.assertEqual([st['label'][:12] for st in snap['steps']][:4],
                         ['Detect the m', 'Pivot the gr', 'Go in front ', 'Enter the bo'])
        plans = [f for n, f in s.log if n == 'plan']
        self.assertEqual([p['pose_name'] for p in plans], ['pivot', 'in front of the slot', 'inside the box', 'out of the box', 'scan'])
        self.assertEqual(plans[0]['start_ticks'], s.scan_ticks)                      # pivot starts at scan
        self.assertEqual(plans[0]['goal_ticks'][6], s.scan_ticks[6] + 1024)           # J7 +90 deg, nothing else
        self.assertEqual(plans[0]['goal_ticks'][:6], s.scan_ticks[:6])
        self.assertEqual(plans[1]['start_ticks'], plans[0]['goal_ticks'])             # legs are chained
        self.assertEqual(plans[2]['start_ticks'], plans[1]['goal_ticks'])
        self.assertEqual(plans[3]['start_ticks'], plans[2]['goal_ticks'])
        self.assertEqual(plans[4]['goal_ticks'], s.scan_ticks)                        # and the last one is scan
        self.assertEqual([p['skip_compensation'] for p in plans], [False, False, False, False, False])
        iks = [b for n, b in s.log if n == '/ik_xyz']
        self.assertEqual(len(iks), 2)
        front, inside = iks
        self.assertAlmostEqual(front['target_xyz_m'][1], 0.34 + 0.03 * 0.08 - 0.10 * 0.997, places=4)   # 10 cm in front of the slot (+ 3 cm left offset)
        self.assertAlmostEqual(inside['target_xyz_m'][1], 0.34 + 0.03 * 0.08 + 0.05 * 0.997, places=4)  # 5 cm inside it
        self.assertEqual(front['pivot_reference_ticks'], s.scan_ticks)
        self.assertFalse(any(n == 'execute' for n, _ in s.log))                       # nothing moved while planning
        s.log.clear()
        orig = s.execute

        def execute(plan_id, gripper):
            s.go_result = None
            return orig(plan_id, gripper)
        s.execute = execute
        self.assertTrue(r.start()['success'])
        snap = wait(r, {'done', 'failed', 'canceled'})
        self.assertEqual(snap['phase'], 'done', snap['message'])
        order = [(n, f.get('ticks') if n == 'gripper' else f) for n, f in s.log if n in ('gripper', 'execute')]
        self.assertEqual(order, [('execute', 'plan-pivot'), ('execute', 'plan-in front of the slot'),
                                 ('execute', 'plan-inside the box'), ('gripper', 2000),
                                 ('execute', 'plan-out of the box'), ('execute', 'plan-scan')])  # the gripper stays open

    def test_insert_letter_refuses_when_the_arm_is_not_at_scan_or_the_recipient_is_unknown(self):
        s, r, _ = self.insert_runner(arm_at=[2026, 1263, 1643, 2908, 1902, 2354, 3250])
        snap = wait(r, {'ready', 'failed'})
        self.assertEqual(snap['phase'], 'failed')
        self.assertIn('not at scan', snap['message'])
        self.assertFalse(any(n == 'plan' for n, _ in s.log))
        s, r, _ = self.insert_runner(recipient='Nobody Here')
        snap = wait(r, {'ready', 'failed'})
        self.assertIn('not found on camera 2', snap['message'])
        self.assertIn('Jan Smith', snap['message'])
        s, r, _ = self.insert_runner(recipient='Luc Marti')
        self.assertIn('no reliable position', wait(r, {'ready', 'failed'})['message'])

    def test_insert_letter_parameters_are_validated(self):
        for bad in (dict(recipient=''), dict(depth_cm=40), dict(front_cm=0), dict(pivot_deg=270), dict(depth_cm='x')):
            _, _, started = self.insert_runner(**bad)
            self.assertFalse(started['success'], bad)

    def test_pick_deliver_plans_the_pick_first_and_the_insertion_live_from_the_name_read(self):
        s = FakeStation()
        s.arm_at = None
        r = TaskRunner(s)
        fake_ocr(r, 'Bob Ross')
        r.prepare('pick_deliver', 1.0, 1, dict(depth_cm=5, front_cm=10, tcp_offset_cm=10.6, pivot_deg=90))
        snap = wait(r, {'ready', 'failed'})
        self.assertEqual(snap['phase'], 'ready', snap['message'])
        plans = [f['pose_name'] for n, f in s.log if n == 'plan']
        self.assertEqual(plans, ['before_grab', 'grab', 'pull', 'up_after_pull', 'read_card', 'scan'])  # insertion not planned yet
        self.assertFalse(any(n in ('/ik_xyz', 'detect') for n, _ in s.log))
        s.log.clear()
        orig = s.execute

        def execute(plan_id, gripper):
            s.go_result = None
            return orig(plan_id, gripper)
        s.execute = execute
        self.assertTrue(r.start()['success'])
        snap = wait(r, {'done', 'failed', 'canceled'})
        self.assertEqual(snap['phase'], 'done', snap['message'])
        legs = [f for n, f in s.log if n == 'execute']
        self.assertEqual(legs[-6:], ['plan-scan', 'plan-pivot', 'plan-in front of the slot',
                                     'plan-inside the box', 'plan-out of the box', 'plan-scan'])
        iks = [b for n, b in s.log if n == '/ik_xyz']
        self.assertAlmostEqual(iks[0]['target_xyz_m'][1], 0.34 + 0.03 * 0.08 - 0.10 * 0.997, places=3)  # Bob Ross' slot

    def test_pick_deliver_stops_with_the_letter_held_when_no_name_was_read(self):
        s = FakeStation()
        r = TaskRunner(s)
        fake_ocr(r, '')
        r.prepare('pick_deliver', 1.0, 1, {})
        wait(r, {'ready', 'failed'})
        orig = s.execute
        s.execute = lambda plan_id, gripper: (setattr(s, 'go_result', None), orig(plan_id, gripper))[1]
        r.start()
        snap = wait(r, {'done', 'failed', 'canceled'})
        self.assertEqual(snap['phase'], 'failed')
        self.assertIn('No card name', snap['message'])
        self.assertNotIn(('gripper', {'ticks': 2000}), s.log)

    def test_start_requires_planning(self):
        self.assertFalse(TaskRunner(FakeStation()).start()['success'])


if __name__ == '__main__':
    unittest.main()
