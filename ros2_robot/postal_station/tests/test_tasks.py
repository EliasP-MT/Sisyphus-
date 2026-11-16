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
        if path == '/tcp_error':
            return {'error_mm': [-0.5, -1.2, -0.8]}
        if path == '/pivot_ticks':
            goal = list(body['ticks'])
            goal[6] += 1024
            return {'goal_ticks': goal}
        return {'goal_ticks': [1500 + len(self.log)] * 7}

    def call(self, service, timeout=0., **f):
        if service == 'list':
            others = ['before_grab', 'grab', 'pull', 'up_after_pull', 'read_card']
            return types.SimpleNamespace(success=True, poses=[types.SimpleNamespace(name='scan', ticks=self.scan_ticks + [1270])] +
                                         [types.SimpleNamespace(name=n, ticks=[1500 + k] * 7 + [1270]) for k, n in enumerate(others)])
        self.log.append((service, f))
        if service == 'plan':
            ends = self.__dict__.setdefault('ends', {})
            ends[f['pose_name']] = [7000 + len(self.log)] * 7  # where the fake plan "ends" (the next leg may start there)
            return types.SimpleNamespace(success=True, executable=True, plan_id='plan-' + f['pose_name'],
                                         duration_s=1., message='', reasons=[], end_ticks=ends[f['pose_name']])
        if service == 'gripper':
            self.motors.motors[0].position_ticks = f['ticks']
        return types.SimpleNamespace(success=True, message='ok')

    def execute(self, plan_id, gripper):
        self.log.append(('execute', plan_id))
        self.go_result = {'success': plan_id != self.fail_leg, 'message': 'arrived ' + plan_id}
        return {'success': True, 'message': 'Motion started'}

    # --- base drive (the real one is closed loop on the wheel odometry)
    drive_error = None
    base_cmd = None

    def drive_check(self):
        if self.drive_error:
            raise RuntimeError(self.drive_error)

    def drive(self, meters, speed, should_stop):
        self.log.append(('drive', round(meters, 3)))
        return abs(meters)


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
        # the pull is a straight 8 cm line from grab; the leg after it starts where the line ended, not at the stored `pull`
        self.assertEqual([p.get('start_pose_name') for p in plans], ['scan', 'before_grab', 'grab', None, 'up_after_pull'])
        # 8 cm forward, targeted a little higher and further: the arm sags back onto the ideal line (PULL_DROOP_MM_PER_M)
        from postal_station.tasks import PULL_DROOP_MM_PER_M
        for got, ideal, droop in zip(plans[2]['line_delta_m'], (0., .08, 0.), PULL_DROOP_MM_PER_M):
            self.assertAlmostEqual(got, ideal + droop / 1000. * .08, places=9)
        self.assertGreater(plans[2]['line_delta_m'][2], 0.005)  # raised by more than 5 mm
        self.assertEqual([p.get('line_delta_m') for i, p in enumerate(plans) if i != 2], [None] * 4)
        self.assertEqual(plans[3]['start_ticks'], s.ends['pull'])
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
        self.assertAlmostEqual(front['target_xyz_m'][1], 0.34 + 0.01 * 0.08 - 0.10 * 0.997, places=4)   # 10 cm in front of the slot (+ 1 cm left offset)
        self.assertAlmostEqual(inside['target_xyz_m'][1], 0.34 + 0.01 * 0.08 + 0.05 * 0.997, places=4)  # 5 cm inside it
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
        self.assertAlmostEqual(iks[0]['target_xyz_m'][1], 0.34 + 0.01 * 0.08 - 0.10 * 0.997, places=3)  # Bob Ross' slot

    def _run_pick_deliver(self, s, params=None):
        r = TaskRunner(s)
        fake_ocr(r, 'Bob Ross')
        r.prepare('pick_deliver', 1.0, 1, params or {})
        wait(r, {'ready', 'failed'})
        s.log.clear()
        orig = s.execute
        s.execute = lambda plan_id, gripper: (setattr(s, 'go_result', None), orig(plan_id, gripper))[1]
        r.start()
        return wait(r, {'done', 'failed', 'canceled'})

    def test_pick_deliver_drives_forward_to_scan_and_back_after_the_drop(self):
        s = FakeStation()
        snap = self._run_pick_deliver(s)
        self.assertEqual(snap['phase'], 'done', snap['message'])
        seq = [(n, f) for n, f in s.log if n in ('execute', 'drive', 'gripper')]
        # +20 cm with the arm leg that brings the letter back to scan (before the mailbox detection),
        # -20 cm with the last leg, after the gripper opened and the arm left the box
        self.assertEqual([x for x in seq if x[0] == 'drive'], [('drive', .2), ('drive', -.2)])
        self.assertEqual(seq.index(('drive', .2)), seq.index(('execute', 'plan-scan')) + 1)
        self.assertLess(seq.index(('gripper', {'ticks': 2000})), seq.index(('drive', -.2)))
        self.assertLess(s.log.index(('drive', .2)), [n for n, _ in s.log].index('detect'))
        self.assertIn('base forward 20.0 cm', ' '.join(st['detail'] for st in snap['steps']))
        self.assertIn('base back 20.0 cm', ' '.join(st['detail'] for st in snap['steps']))

    def test_pick_deliver_drive_distances_are_parameters_and_zero_disables(self):
        s = FakeStation()
        snap = self._run_pick_deliver(s, dict(approach_cm=12, retreat_cm=0))
        self.assertEqual(snap['phase'], 'done', snap['message'])
        self.assertEqual([f for n, f in s.log if n == 'drive'], [.12])
        for bad in (dict(approach_cm=-1), dict(retreat_cm=60), dict(approach_cm='x')):
            self.assertFalse(TaskRunner(FakeStation()).prepare('pick_deliver', 1., 1, bad)['success'], bad)

    def test_pick_deliver_does_not_move_the_arm_when_the_base_cannot_drive(self):
        s = FakeStation()
        s.drive_error = 'Base drive: base torque is off (enable the base)'
        r = TaskRunner(s)
        fake_ocr(r, 'Bob Ross')
        r.prepare('pick_deliver', 1.0, 1, {})
        wait(r, {'ready', 'failed'})
        s.log.clear()
        started = r.start()  # refused before ANY motion: the letter would otherwise be picked and then stuck
        self.assertFalse(started['success'])
        self.assertIn('base torque is off', started['message'])
        self.assertFalse(any(n in ('execute', 'gripper', 'dispense') for n, _ in s.log))

    # ---------------------------------------------------------------- deliver_all (the mission FSM)
    def _mission(self, s, names, params=None):
        r = TaskRunner(s)
        queue = iter(names)

        def read_card(camera=None):
            name = next(queue)
            r.ocr = {'success': True, 'name': name, 'camera': 1}
            return f'name: {name}'
        r.read_card = read_card
        r.prepare('deliver_all', 1.0, 1, params or {})
        return r, wait(r, {'ready', 'failed'})

    def _start(self, r, s, timeout=60.):
        s.log.clear()
        orig = s.execute
        s.execute = lambda plan_id, gripper: (setattr(s, 'go_result', None), orig(plan_id, gripper))[1]
        self.assertTrue(r.start()['success'])
        return wait(r, {'done', 'failed', 'canceled'}, timeout)

    def test_mission_surveys_then_delivers_three_letters(self):
        s = FakeStation()
        r, snap = self._mission(s, ['Jan Smith', 'Bob Ross', 'Jan Smith'])
        self.assertEqual(snap['phase'], 'ready', snap['message'])
        # only letter 1 is planned up front (plans live 10 min); letters 2 and 3 are planned when they start
        self.assertEqual([f['pose_name'] for n, f in s.log if n == 'plan'],
                         ['before_grab', 'grab', 'pull', 'up_after_pull', 'read_card', 'scan'])
        states = [k for k, _ in __import__('itertools').groupby(st['state'] for st in snap['steps'])]
        cycle = ['PICK', 'READ', 'APPROACH', 'LOCATE', 'INSERT', 'RETREAT']
        self.assertEqual(states, ['SURVEY'] + cycle + ['PLAN'] + cycle + ['PLAN'] + cycle)
        snap = self._start(r, s)
        self.assertEqual(snap['phase'], 'done', snap['message'])
        self.assertEqual(snap['fsm'], {'state': 'DONE', 'letter': None})
        self.assertEqual([(l['letter'], l['name'], l['status']) for l in snap['letters']],
                         [(1, 'Jan Smith', 'delivered'), (2, 'Bob Ross', 'delivered'), (3, 'Jan Smith', 'delivered')])
        self.assertEqual([m['name'] for m in snap['mailboxes']], ['Jan Smith', 'Bob Ross', 'Luc Marti'])
        self.assertIsNone(snap['mailboxes'][2]['slot_base_m'])  # named, but no reliable slot position (seen edge-on)
        self.assertEqual(snap['mailboxes'][0]['slot_base_m'], [0.02, 0.33, 0.23])
        names = [n for n, _ in s.log]
        self.assertEqual(names[0], 'detect')  # the survey is the first thing that happens
        self.assertEqual(names.count('detect'), 1 + 3)  # survey + one live detection per letter
        self.assertEqual(names.count('dispense'), 3)    # three letters picked
        self.assertEqual([f for n, f in s.log if n == 'drive'], [.2, -.2] * 3)
        planned = [f['pose_name'] for n, f in s.log if n == 'plan']
        self.assertEqual(planned.count('before_grab'), 2)  # letters 2 and 3, planned live
        first_exec = names.index('execute')
        self.assertTrue(all(i > first_exec for i, n in enumerate(names) if n == 'plan' and s.log[i][1]['pose_name'] == 'before_grab'))
        live = [f for n, f in s.log if n == 'plan']  # letters 2 and 3: planned when they start
        self.assertEqual([len(p['line_delta_m']) for p in live if p['pose_name'] == 'pull'], [3, 3])
        self.assertTrue(all(p['line_delta_m'][2] > 0.005 for p in live if p['pose_name'] == 'pull'))
        self.assertTrue(all('start_ticks' in p and 'start_pose_name' not in p for p in live if p['pose_name'] == 'up_after_pull'))
        iks = [b for n, b in s.log if n == '/ik_xyz']
        self.assertEqual(len(iks), 6)  # front + inside, three times
        self.assertAlmostEqual(iks[0]['target_xyz_m'][1], 0.33 + 0.01 * 0.08 - 0.10 * 0.997, places=3)  # Jan Smith
        self.assertAlmostEqual(iks[2]['target_xyz_m'][1], 0.34 + 0.01 * 0.08 - 0.10 * 0.997, places=3)  # Bob Ross
        self.assertAlmostEqual(iks[4]['target_xyz_m'][1], 0.33 + 0.01 * 0.08 - 0.10 * 0.997, places=3)  # Jan Smith again

    def test_mission_unknown_name_is_looked_for_at_the_mailboxes_then_stops(self):
        s = FakeStation()
        r, snap = self._mission(s, ['Zed Nobody'])
        self.assertEqual(snap['phase'], 'ready', snap['message'])
        snap = self._start(r, s)
        self.assertEqual(snap['phase'], 'failed')
        self.assertIn('not found on camera 2', snap['message'])  # the live detection decided
        self.assertIn('not in the survey', snap['steps'][8]['detail'])  # READ warned but let it continue
        self.assertEqual(snap['fsm'], {'state': 'FAILED', 'letter': 1})
        self.assertEqual(snap['letters'][0]['status'], 'failed')
        self.assertNotIn(('gripper', {'ticks': 2000}), s.log)   # the letter stays in the gripper

    def test_mission_second_letter_failure_keeps_the_first_delivered(self):
        s = FakeStation()
        r, _ = self._mission(s, ['Jan Smith', 'Zed Nobody'])
        snap = self._start(r, s)
        self.assertEqual(snap['phase'], 'failed')
        self.assertEqual([l['status'] for l in snap['letters']], ['delivered', 'failed', 'pending'])
        self.assertEqual([f for n, f in s.log if n == 'drive'], [.2, -.2, .2])  # letter 2 reached the mailboxes, then failed

    def test_mission_needs_the_arm_at_scan_and_valid_parameters(self):
        s = FakeStation()
        s.arm_at = [0] * 7
        r, snap = self._mission(s, [])
        self.assertEqual(snap['phase'], 'failed')
        self.assertIn('not at scan', snap['message'])
        for bad in (dict(letters=0), dict(letters=7), dict(letters='x')):
            self.assertFalse(TaskRunner(FakeStation()).prepare('deliver_all', 1., 1, bad)['success'], bad)
        s = FakeStation()
        r, snap = self._mission(s, ['Jan Smith', 'Bob Ross'], dict(letters=2))
        self.assertEqual(self._start(r, s)['phase'], 'done')
        self.assertEqual([f for n, f in s.log if n == 'drive'], [.2, -.2] * 2)

    def test_mission_fails_when_no_mailbox_name_can_be_read(self):
        s = FakeStation()
        s.detect_mailboxes = lambda camera: (setattr(s, 'detect_result', {'labels': [{'id': 1, 'name': '', 'slot_base_m': None}]}),
                                             {'success': True, 'message': '1 label'})[1]
        r, _ = self._mission(s, ['Jan Smith'])
        snap = self._start(r, s)
        self.assertEqual(snap['phase'], 'failed')
        self.assertIn('No mailbox name could be read', snap['message'])
        self.assertFalse(any(n in ('execute', 'dispense') for n, _ in s.log))  # nothing moved

    def test_the_end_of_the_line_is_measured_and_reported(self):
        s = FakeStation()
        r = TaskRunner(s)
        fake_ocr(r)
        r.prepare('pick_scan', 1.)
        wait(r, {'ready'})
        snap = self._start(r, s)
        self.assertEqual(snap['phase'], 'done', snap['message'])
        pull = next(st for st in snap['steps'] if 'Pull the letter out' in st['label'])
        self.assertIn('end of line: fingertips -0.5 mm in x, -1.2 in y, -0.8 in z', pull['detail'])
        calls = [b for n, b in s.log if n == '/tcp_error']
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]['ideal_delta_m'], [0., .08, 0.])  # measured against the IDEAL line, not the compensated target

    def test_a_leg_waits_for_the_arm_to_be_still_and_reports_a_creep(self):
        s = FakeStation()
        s.wait_arm_still = lambda *a, **k: False
        s.arm_speeds = lambda: [0, 0, 31, 0, 0, 0, 0]
        r = TaskRunner(s)
        fake_ocr(r)
        r.prepare('pick_scan', 1.)
        wait(r, {'ready'})
        s.log.clear()
        r.start()
        snap = wait(r, {'failed', 'done'})
        self.assertEqual(snap['phase'], 'failed')
        self.assertIn('keeps moving', snap['message'])
        self.assertIn('[0, 0, 31, 0, 0, 0, 0]', snap['message'])
        self.assertFalse(any(n == 'execute' for n, _ in s.log))  # nothing was sent while it was creeping

    def test_pick_scan_never_drives(self):
        s = FakeStation()
        r = TaskRunner(s)
        fake_ocr(r)
        r.prepare('pick_scan', 1.)
        wait(r, {'ready'})
        orig = s.execute
        s.execute = lambda plan_id, gripper: (setattr(s, 'go_result', None), orig(plan_id, gripper))[1]
        r.start()
        wait(r, {'done', 'failed'})
        self.assertFalse(any(n == 'drive' for n, _ in s.log))

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
