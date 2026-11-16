"""Station-side task sequencer (pick and scan a letter, ...).

A task is a list of arm legs between stored poses with side actions (gripper,
dispenser). Everything is planned BEFORE the task starts: leg 0 from the measured
arm pose, every following leg from the previous stored pose (fixed legs, cached by
arm_motion). Execution then only chains already collision-checked plans; any
failure stops the dispenser, cancels the arm goal and leaves the arm held where it
is. Nothing resumes by itself."""
import base64
import json
import math
import threading
import time
import urllib.request

PLAN_LIFETIME_S = 600.
OCR_URL = 'http://127.0.0.1:8894'
OCR_SKIP_FRAMES = 5   # as ocr_base.read_recipient_name: discard five frames, read the sixth

# Gravity droop of the pull (the arm servos are soft: P=16, D=0): measured at the end of the 8 cm line the
# fingertips were 1.2 mm short in x (-1.2), 5.3 mm short in y and 6.6 mm LOW in z. The line is therefore planned
# towards a target raised / extended by this much PER METRE of travel (0.08 m -> +1.2, +5.3, +6.6 mm), the arm sags
# back onto the ideal line. First order: the end of every pull is measured and shown in the step (retune from it).
PULL_DROOP_MM_PER_M = [15., 66.25, 82.5]

# kind 'leg': go to pose (optionally with a gripper/dispenser side action started with it)
# kind 'gripper': move the gripper and wait for it to settle
TASKS = {
    'pick_scan': {
        'title': 'Pick and scan a letter',
        'start_pose': 'scan',   # the task starts from this stored pose: every leg is a fixed, cached plan
        'steps': [
            {'kind': 'leg', 'pose': 'before_grab', 'start_tol': .05, 'arrival_tol': .03, 'gripper_open': 2300, 'label': 'Go to before_grab, opening gripper to 2300'},
            {'kind': 'leg', 'pose': 'grab', 'dispenser_s': 4.5, 'arrival_tol': .07, 'label': 'Go to grab, dispenser −1000 for 4.5 s'},
            {'kind': 'gripper', 'ticks': 1270, 'label': 'Close gripper to 1270'},
            # Pull the letter out of the dispenser: a STRAIGHT 8 cm line forward (+y of the arm frame, towards the
            # mailboxes) from the grab configuration, planned as one smooth motion (arm_control Arm.line_waypoints).
            # The next leg starts from where the line ends, not from the stored `pull` pose.
            {'kind': 'leg', 'pose': 'pull', 'line_cm': [0., 8., 0.], 'arrival_tol': .07, 'no_compensation': True,
             'droop_mm_per_m': PULL_DROOP_MM_PER_M, 'label': 'Pull the letter out: straight line, 8 cm forward (gravity compensated)'},
            {'kind': 'leg', 'pose': 'up_after_pull', 'arrival_tol': .10, 'label': 'Go to up_after_pull'},
            {'kind': 'leg', 'pose': 'read_card', 'arrival_tol': .04, 'label': 'Go to read_card'},
            {'kind': 'ocr', 'label': 'Read the name on the card (OCR, one frame)'},
        ],
    },
    # Steps are built when planning: they depend on where the slot of the chosen recipient is.
    'insert_letter': {'title': 'Insert the letter into a mailbox slot', 'dynamic': True},
    # Pick, read the name, back to scan, then insert into the mailbox of the name that was read: the
    # slot legs can only be planned once the name is known, so they are planned live at scan.
    'pick_deliver': {'title': 'Pick, read the name and insert into its mailbox', 'dynamic': True, 'combined': True},
    # Mission: from scan, photograph the mailboxes once (names + positions), then for each letter
    # PICK -> READ -> APPROACH -> LOCATE -> INSERT -> RETREAT (the pick_deliver cycle). States are explicit
    # (steps carry `state` and `letter`; the snapshot has `fsm`, `mailboxes` and `letters`).
    'deliver_all': {'title': 'Deliver the letters (survey the mailboxes, then pick, read and insert each)',
                    'dynamic': True, 'combined': True, 'mission': True},
}

LETTERS = 3           # deliver_all: letters delivered per mission (param `letters`, 1-6)
OPEN_GRIPPER_TICKS = 2000
DRIVE_CM = 20.        # pick_deliver: base distance towards / away from the mailboxes (approach_cm, retreat_cm)
DRIVE_SPEED_M_S = .08
SCAN_TOLERANCE_TICKS = 40  # the task starts from the scan pose


class TaskAborted(Exception):
    pass


class TaskRunner:
    def __init__(self, station):
        self.s = station
        self.lock = threading.Lock()
        self.thread = None
        self.cancel = False
        self.reset()

    def reset(self, task=None):
        self.task, self.phase, self.message = task, 'idle', ''
        self.ocr_camera = getattr(self, 'ocr_camera', 1)
        self.ocr = getattr(self, 'ocr', None)
        self.steps, self.plans, self.prepared_at = [], {}, 0.
        self.defs = []  # the step definitions of the task being planned / run
        self.current = -1
        self.line_info = {}  # leg index -> where its straight line starts and the ideal displacement (for the end-of-line report)
        self.mailboxes, self.letters = {}, []  # deliver_all: surveyed mailboxes (by normalised name), per-letter report
        self.fsm = {'state': 'IDLE', 'letter': None}

    # ------------------------------------------------------------------ state
    def snapshot(self):
        with self.lock:
            return {'task': self.task, 'phase': self.phase, 'message': self.message, 'current': self.current,
                    'steps': [dict(st) for st in self.steps], 'prepared_age_s': time.monotonic() - self.prepared_at
                    if self.prepared_at else None,
                    'tasks': {k: v['title'] for k, v in TASKS.items()}, 'ocr': self.ocr,
                    'fsm': dict(self.fsm), 'letters': [dict(l) for l in self.letters],
                    'mailboxes': sorted((dict(m) for m in self.mailboxes.values()), key=lambda m: str(m.get('id')))}

    def _set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def _step(self, i, **kw):
        with self.lock:
            self.steps[i].update(kw)

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    # ------------------------------------------------------------------ API
    def prepare(self, name, speed, ocr_camera=1, params=None):
        if name not in TASKS:
            raise KeyError(f'unknown task {name}')
        if self.busy():
            return {'success': False, 'message': 'A task is already planning or running'}
        if self.s.go_handle is not None and self.s.go_result is None:
            return {'success': False, 'message': 'An arm motion is running'}
        self.cancel = False
        ocr = self.ocr
        self.reset(name)
        self.ocr, self.ocr_camera = ocr, int(ocr_camera) if int(ocr_camera) in self.s.cameras else 1
        if TASKS[name].get('dynamic'):
            try:
                self.defs = self._insert_definition(params or {}, combined=bool(TASKS[name].get('combined')),
                                                    mission=bool(TASKS[name].get('mission')))
            except ValueError as exc:
                self.phase, self.message = 'failed', str(exc)
                return {'success': False, 'message': str(exc)}
        else:
            self.defs = [dict(st) for st in TASKS[name]['steps']]
        self.steps = [{'label': st['label'], 'status': 'pending', 'detail': '', 'state': st.get('state', ''),
                       'letter': st.get('letter')} for st in self.defs]
        if TASKS[name].get('mission'):
            self.letters = [{'letter': k, 'name': '', 'mailbox': None, 'status': 'pending'}
                            for k in range(1, max(st.get('letter', 0) or 0 for st in self.defs) + 1)]
        self.phase = 'planning'
        self.speed = speed
        self.thread = threading.Thread(target=self._prepare_insert if TASKS[name].get('dynamic') and not TASKS[name].get('combined') else self._prepare,
                                       args=(name, speed), daemon=True)
        self.thread.start()
        return {'success': True, 'message': 'Planning every leg of the task'}

    def start(self):
        if self.busy():
            return {'success': False, 'message': 'A task is already planning or running'}
        if self.phase != 'ready':
            return {'success': False, 'message': 'Plan the task first'}
        if time.monotonic() - self.prepared_at > PLAN_LIFETIME_S - 30:
            self._set(phase='idle', message='Plans expired; plan the task again')
            return {'success': False, 'message': 'Plans expired; plan the task again'}
        if any(st.get('drive_cm') for st in self.defs):  # refuse before ANY motion if the base cannot drive
            try:
                self.s.drive_check()
            except RuntimeError as exc:
                return {'success': False, 'message': str(exc)}
        self.cancel = False
        self._set(phase='running', message='Task running')
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return {'success': True, 'message': 'Task started'}

    def stop(self):
        self.cancel = True
        if self.s.go_handle is not None:
            self.s.go_handle.cancel_goal_async()
        self._safe_stop()
        if not self.busy() and self.phase in ('ready', 'planning'):
            self._set(phase='idle', message='Task discarded')
        return {'success': True, 'message': 'Task stop requested; dispenser stopped, arm held'}

    # ------------------------------------------------------------------ planning
    def _plan_fixed(self, i, st, previous, previous_tol, speed):
        """Plan the fixed leg `st` and return (plan id, the 7 ticks it ends at).

        `previous` is where the leg starts: a stored pose name, or the 7 ticks a straight line ended at. A leg
        with `line_cm` is a straight Cartesian line of the fingertips from there instead of a stored pose."""
        if self.cancel:
            raise TaskAborted('planning canceled')
        self._step(i, status='planning')
        began = time.monotonic()
        fields = dict(timeout=110., pose_name=st['pose'], speed_factor=float(speed), valid_for_s=PLAN_LIFETIME_S,
                      arrival_tolerance_rad=st.get('arrival_tol', 0.), skip_compensation=bool(st.get('no_compensation')),
                      start_tolerance_rad=max(st.get('start_tol', 0.), previous_tol))
        if isinstance(previous, str):
            fields['start_pose_name'] = previous
        else:
            fields['start_ticks'] = [int(t) for t in previous]
        if st.get('line_cm'):
            ideal = [v / 100. for v in st['line_cm']]
            length = math.sqrt(sum(v * v for v in ideal))
            droop = st.get('droop_mm_per_m') or [0., 0., 0.]
            # straight line towards the target raised / extended by the expected droop (it sags back onto the ideal line)
            fields['line_delta_m'] = [v + d / 1000. * length for v, d in zip(ideal, droop)]
            with self.lock:
                self.line_info[i] = dict(start=previous, ideal_m=ideal)
        r = self.s.call('plan', **fields)
        if not r.success or not r.executable:
            why = r.message if not r.success else '; '.join(r.reasons)
            raise RuntimeError(f'{st["pose"]}: {why}')
        self._step(i, status='ok', duration_s=round(r.duration_s, 1),
                   detail=f'{r.duration_s:.1f} s, planned in {time.monotonic() - began:.1f} s'
                          + (f'; {r.message.split("plan at")[0].split("Collision-free ")[-1].strip(" ;")}' if st.get('line_cm') else ''))
        return r.plan_id, [int(t) for t in r.end_ticks]

    @staticmethod
    def _next_start(st, end_ticks):
        """Where the leg after `st` starts: the end of a straight line (not a stored pose), else the pose."""
        return end_ticks if st.get('line_cm') else st['pose']

    def _plan_legs(self, st):
        """deliver_all, letters 2..n: plan the fixed pick legs when the letter starts (plans live 10 min, a
        mission lasts longer). The legs are cached on disk by arm_motion, so this is quick."""
        previous, previous_tol, plans = 'scan', .05, {}
        for t in st['targets']:
            leg = self.defs[t]
            try:
                plans[t], end = self._plan_fixed(t, leg, previous, previous_tol, self.speed)
            except Exception as exc:
                self._step(t, status='failed', detail=str(exc))
                raise
            previous, previous_tol = self._next_start(leg, end), leg.get('arrival_tol', 0.)
        with self.lock:
            self.plans.update(plans)
        return f'{len(plans)} legs planned'

    def _prepare(self, name, speed):
        i = -1
        try:
            if TASKS[name].get('mission'):
                self._require_at_scan()  # the mission starts from the scan pose
            previous = TASKS[name].get('start_pose', '') or TASKS['pick_scan']['start_pose']
            previous_tol = .05  # the arm is expected near the start pose
            plans = {}
            for i, st in enumerate(self.defs):
                if st.get('later'):
                    self._step(i, detail=st.get('later_note') or ('planned live once the card name is read' if st.get('live') else ''))
                    continue
                if st['kind'] != 'leg':
                    self._step(i, status='ok', detail='no planning needed')
                    continue
                plans[i], end = self._plan_fixed(i, st, previous, previous_tol, speed)
                previous, previous_tol = self._next_start(st, end), st.get('arrival_tol', 0.)
            with self.lock:
                self.plans, self.prepared_at = plans, time.monotonic()
                self.phase, self.message = 'ready', 'All legs planned; ready to start'
        except Exception as exc:
            if i >= 0:
                self._step(i, status='failed', detail=str(exc))
            self._set(phase='failed', message=f'Planning failed: {exc}')

    # ------------------------------------------------------------------ insert a letter
    def _insert_definition(self, params, combined=False, mission=False):
        recipient = str(params.get('recipient', '')).strip()
        if not recipient and not combined:
            raise ValueError('Give the recipient (the name on the mailbox)')
        try:
            depth, front = float(params.get('depth_cm', 5)), float(params.get('front_cm', 10))
            tcp, pivot = float(params.get('tcp_offset_cm', 10.6)), float(params.get('pivot_deg', 90))
            left = float(params.get('left_cm', 1))
            # pick_deliver only: the base drives forward while the arm returns to scan with the letter, and
            # backward while the arm returns to scan after the drop (0 = the base stays where it is)
            approach = float(params.get('approach_cm', DRIVE_CM)) if combined else 0.
            retreat = float(params.get('retreat_cm', DRIVE_CM)) if combined else 0.
            letters = int(params.get('letters', LETTERS)) if mission else 1
        except (TypeError, ValueError):
            raise ValueError('depth, front distance, fingertip offset, pivot, drive distances and letters must be numbers') from None
        if not (0 <= depth <= 15 and 3 <= front <= 40 and 0 <= tcp <= 20 and abs(pivot) <= 180 and abs(left) <= 10):
            raise ValueError('depth 0-15 cm, front distance 3-40 cm, fingertip offset 0-20 cm, pivot within 180 degrees')
        if not (0 <= approach <= 50 and 0 <= retreat <= 50):
            raise ValueError('approach and retreat distances within 0-50 cm')
        if not 1 <= letters <= 6:
            raise ValueError('letters: 1 to 6')
        self.insert = dict(recipient=recipient or None, depth=depth / 100., front=front / 100., tcp=tcp / 100.,
                           pivot=pivot * 3.141592653589793 / 180., left=left / 100.)
        leg = lambda label, **kw: dict(kind='leg', label=label, **kw)  # noqa: E731

        def insertion():
            return [
                dict(kind='detect', label='Detect the mailboxes (camera 2) and locate the slot of ' + (recipient or 'the name read on the card')),
                leg(f'Pivot the gripper {pivot:+.0f}° (J7) at scan', arrival_tol=.03),
                # .03 rad: the fine 0.01 default was refused on the robot (J6 stays 0.0215 rad off and no collision-free
                # compensation bound exists next to the box, so the arrival timed out)
                leg(f'Go in front of the slot ({front:g} cm)', arrival_tol=.03),
                leg(f'Enter the box ({depth:g} cm inside the slot plane)', arrival_tol=.10),
                dict(kind='gripper', ticks=OPEN_GRIPPER_TICKS, label=f'Open the gripper to {OPEN_GRIPPER_TICKS}'),
                leg('Leave the box', arrival_tol=.10),
                leg('Back to scan (pivot undone, gripper stays open)', arrival_tol=.03),
            ]
        if not combined:
            return insertion()

        def letter_group(k):
            """One letter: PICK (grab, pull, up) -> READ (OCR) -> APPROACH (back to scan, base forward) ->
            LOCATE (detect the slot at scan) -> INSERT (pivot, in front, enter, open) -> RETREAT (leave, back
            to scan, base back)."""
            steps = insertion()
            for st in steps:
                st['later'] = True  # planned at run time, once the card name is known
            steps[0]['live'] = True
            picks = [dict(st) for st in TASKS['pick_scan']['steps']]
            to_scan = dict(kind='leg', pose='scan', arrival_tol=.04, label='Back to scan, letter held')
            if approach:
                to_scan['drive_cm'] = approach
                to_scan['label'] += f', base forward {approach:g} cm'
            if retreat:  # last leg: the arm has left the box and the gripper is open
                steps[-1]['drive_cm'] = -retreat
                steps[-1]['label'] += f', base back {retreat:g} cm'
            for st in picks:
                st['state'] = 'READ' if st['kind'] == 'ocr' else 'PICK'
            to_scan['state'] = 'APPROACH'
            for st, state in zip(steps, ('LOCATE', 'INSERT', 'INSERT', 'INSERT', 'INSERT', 'RETREAT', 'RETREAT')):
                st['state'] = state
            steps[-1]['last_of_letter'] = True
            for st in picks + [to_scan] + steps:
                st['letter'] = k
            return picks, to_scan, steps

        if not mission:
            picks, to_scan, steps = letter_group(1)
            return picks + [to_scan] + steps
        defs = [dict(kind='survey', state='SURVEY', letter=0,
                     label='Photograph the mailboxes from scan (camera 2): names and positions')]
        for k in range(1, letters + 1):
            picks, to_scan, steps = letter_group(k)
            picks[-1]['verify'] = True  # READ: the name must be one of the surveyed mailboxes
            for st in picks + [to_scan]:
                st['label'] = f'Letter {k}: ' + st['label']
            for st in steps:
                st['label'] = f'Letter {k}: ' + st['label']
            group = picks + [to_scan]
            if k > 1:  # plans live 10 min and a mission is longer: plan these legs when the letter starts
                base = len(defs)
                defs.append(dict(kind='plan_legs', state='PLAN', letter=k, label=f'Letter {k}: plan the pick legs (fixed, cached)',
                                 targets=[base + 1 + j for j, st in enumerate(group) if st['kind'] == 'leg']))
                for st in group:
                    st['later'] = True
                    st['later_note'] = f'planned when letter {k} starts'
            defs += group + steps
        return defs

    def _pose_ticks(self, name):
        poses = self.s.call('list')
        pose = next((p for p in poses.poses if p.name == name), None) if poses.success else None
        if pose is None:
            raise RuntimeError(f'Pose "{name}" not found on the robot')
        return [int(t) for t in list(pose.ticks)[:7]]

    @staticmethod
    def _norm(text):
        return ''.join(c for c in str(text).casefold() if c.isalnum())

    def _require_at_scan(self):
        """The arm must be at the stored scan pose (within SCAN_TOLERANCE_TICKS); returns its ticks."""
        scan = self._pose_ticks('scan')
        off = max(abs(a - b) for a, b in zip(self.s.arm_ticks(), scan))
        if off > SCAN_TOLERANCE_TICKS:
            raise RuntimeError(f'The arm is not at scan (largest joint error {off} ticks > {SCAN_TOLERANCE_TICKS}): go to scan first')
        return scan

    def _survey(self):
        """SURVEY: one photograph of the mailboxes from scan (camera 2). Records every label whose name was
        read: id, name and the label / slot positions in the arm frame (base_link), plus the viewing direction."""
        self._require_at_scan()
        registry, labels_seen = {}, 0
        for attempt in range(2):  # a single photograph often misses a label: merge two
            result = self.s.detect_mailboxes(2)
            if not result['success']:
                if attempt == 0:
                    raise RuntimeError(result['message'])
                continue
            labels = self.s.detect_result['labels']
            labels_seen = max(labels_seen, len(labels))
            for l in labels:
                if l.get('name'):
                    key = self._norm(l['name'])
                    if key not in registry or (l.get('slot_base_m') and not registry[key]['slot_base_m']):
                        registry[key] = dict(id=l.get('id'), name=l['name'], slot_base_m=l.get('slot_base_m'),
                                             label_base_m=l.get('label_base_m'), approach_dir=l.get('approach_dir'))
        labels = [None] * labels_seen
        if not registry:
            raise RuntimeError('No mailbox name could be read: nothing to deliver to (check the lighting and the position)')
        with self.lock:
            self.mailboxes = registry
        fmt = lambda m: (f'{m["name"]} slot at {[round(v * 100, 1) for v in m["slot_base_m"]]} cm' if m['slot_base_m']  # noqa: E731
                         else f'{m["name"]} (no reliable slot position yet)')
        unread = len(labels) - len(registry)
        return '; '.join(fmt(m) for m in registry.values()) + (f'; {unread} label(s) without a readable name' if unread else '')

    def _verify_recipient(self, detail, k):
        """READ: the name read on the card must be a surveyed mailbox, checked BEFORE the base drives."""
        name = (self.ocr or {}).get('name')
        if not name:
            raise RuntimeError(f'No card name was read ({detail}): nothing to deliver (the letter stays in the gripper)')
        box = self.mailboxes.get(self._norm(name))
        if box is None:  # the survey can miss a label: the live detection at the mailboxes decides
            known = ', '.join(m['name'] for m in self.mailboxes.values())
            if k:
                self._letter(k, name=name)
            return f' -> not in the survey ({known}); will look for it at the mailboxes'
        if k:
            self._letter(k, name=name, mailbox=box['id'])
        return f' -> mailbox {box["id"]}'

    def _letter(self, k, **kw):
        with self.lock:
            if 1 <= k <= len(self.letters):
                self.letters[k - 1].update(kw)

    def _prepare_insert(self, name, speed):
        self._at = -1
        try:
            self._plan_insert(speed, 0, self.insert['recipient'])
            with self.lock:
                self.prepared_at = time.monotonic()
                self.phase, self.message = 'ready', 'All legs planned; ready to start'
        except Exception as exc:
            if self._at >= 0:
                self._step(self._at, status='failed', detail=str(exc))
            self._set(phase='failed', message=f'Planning failed: {exc}')

    def _plan_insert(self, speed, o, recipient):
        """Detect the mailboxes, find the slot of `recipient` and plan the insertion legs. o = index of the
        detect step in self.defs (the legs follow it). Raises on any problem."""
        if True:
            p, s = self.insert, self.s
            scan = self._require_at_scan()
            self._at = o
            self._step(o, status='planning')
            result = s.detect_mailboxes(2)
            if not result['success']:
                raise RuntimeError(result['message'])
            labels = s.detect_result['labels']
            label = next((l for l in labels if l.get('name') and self._norm(l['name']) == self._norm(recipient)), None)
            if label is None:
                seen = ', '.join(l['name'] for l in labels if l.get('name')) or 'no name read'
                raise RuntimeError(f'"{p["recipient"]}" not found on camera 2 (names read: {seen})')
            if not label.get('slot_base_m'):
                raise RuntimeError(f'The slot of {label["name"]} has no reliable position (label seen edge-on?)')
            slot, d = label['slot_base_m'], label['approach_dir']
            self._step(o, status='ok', detail=f'{label["name"]}: slot at {[round(v * 100, 1) for v in slot]} cm in the arm frame (before the {p["left"] * 100:g} cm left offset)')
            # shift the target to the left of the slot centre, seen from the robot facing the box
            slot = [slot[0] - d[1] * p['left'], slot[1] + d[0] * p['left'], slot[2]]
            offset = [0., p['tcp'], 0.]

            def ik(seed, standoff):
                target = [a - standoff * b for a, b in zip(slot, d)]
                return s.planner_call('/ik_xyz', {'start_ticks': seed, 'target_xyz_m': target, 'tcp_offset_m': offset,
                                                  'approach_dir': d, 'pivot_rad': p['pivot'],
                                                  'pivot_reference_ticks': scan})['goal_ticks']

            pivoted = s.planner_call('/pivot_ticks', {'ticks': scan, 'delta_rad': p['pivot']})['goal_ticks']
            front = ik(pivoted, p['front'])
            inside = ik(front, -p['depth'])
            routes = {o + 1: (scan, pivoted, 'pivot'), o + 2: (pivoted, front, 'in front of the slot'), o + 3: (front, inside, 'inside the box'),
                      o + 5: (inside, front, 'out of the box'), o + 6: (front, scan, 'scan')}
            plans = {}
            for i, (start, goal, label_) in routes.items():
                if self.cancel:
                    raise TaskAborted('planning canceled')
                self._at = i
                self._step(i, status='planning')
                began = time.monotonic()
                d_ = self.defs[i]
                r = s.call('plan', timeout=120., pose_name=label_, start_ticks=start, goal_ticks=goal,
                           speed_factor=float(speed), valid_for_s=PLAN_LIFETIME_S,
                           arrival_tolerance_rad=d_.get('arrival_tol', 0.), start_tolerance_rad=.05,
                           skip_compensation=bool(d_.get('no_compensation')))
                if not r.success or not r.executable:
                    raise RuntimeError(f'{label_}: ' + (r.message if not r.success else '; '.join(r.reasons)))
                plans[i] = r.plan_id
                self._step(i, status='ok', duration_s=round(r.duration_s, 1),
                           detail=f'{r.duration_s:.1f} s, planned in {time.monotonic() - began:.1f} s')
            for k, st in enumerate(self.defs):
                if o < k <= o + 6 and st['kind'] == 'gripper':  # this letter's insertion only
                    self._step(k, status='ok', detail='no planning needed')
            with self.lock:
                self.plans.update(plans)

    # ------------------------------------------------------------------ execution
    def _check(self):
        if self.cancel:
            raise TaskAborted('task stopped by the operator')

    def _gripper_ticks(self):
        with self.s.lock:
            motors = self.s.motors
        g = next((m for m in motors.motors if m.role == 'gripper'), None) if motors else None
        return g.position_ticks if g and g.ever_received else None

    def _wait_gripper(self, target, tol, timeout, require):
        """Wait for the gripper to reach target, or (when not require) to stop moving."""
        end, last, stable = time.monotonic() + timeout, None, time.monotonic()
        while time.monotonic() < end:
            self._check()
            pos = self._gripper_ticks()
            if pos is not None:
                if abs(pos - target) <= tol:
                    return pos
                if last is None or abs(pos - last) > 2:
                    last, stable = pos, time.monotonic()
                elif not require and time.monotonic() - stable >= .8:
                    return pos
            time.sleep(.05)
        if require:
            raise RuntimeError(f'gripper did not reach {target} ticks within {timeout:.0f} s (now {self._gripper_ticks()})')
        return self._gripper_ticks()

    # ------------------------------------------------------------------ OCR
    def grab_frame(self, camera):
        """One JPEG frame: the sixth new frame received now (the first five let exposure settle)."""
        cam = self.s.cameras[camera]
        with cam.cond:
            base = cam.count
            if not cam.cond.wait_for(lambda: cam.count >= base + 1 + OCR_SKIP_FRAMES, timeout=8.):
                raise RuntimeError(f'camera {camera} delivered no fresh frames (stream stalled?)')
            return cam.jpeg

    def read_card(self, camera=None):
        """Read the recipient name from one fresh camera frame (base_ocr read_recipient_name)."""
        camera = camera or self.ocr_camera
        try:
            status = json.load(urllib.request.urlopen(OCR_URL + '/status', timeout=3))
            if not status['ready']:
                raise RuntimeError(status['reason'] or 'OCR model still loading, try again in a moment')
            jpeg = self.grab_frame(camera)
            self.last_frame = jpeg
            req = urllib.request.Request(OCR_URL + '/read', json.dumps({'jpeg_b64': base64.b64encode(jpeg).decode()}).encode(),
                                         {'Content-Type': 'application/json'})
            r = json.load(urllib.request.urlopen(req, timeout=120))
            if not r['success']:
                raise RuntimeError(r['message'])
            self.ocr = {'success': True, 'name': r['name'], 'seconds': r['seconds'], 'camera': camera,
                        'time': time.time(), 'frame_id': int(time.time() * 1000)}
            return f'name: {r["name"]}' if r['name'] else 'no known name read (empty, uncertain or ambiguous)'
        except Exception as exc:
            self.ocr = {'success': False, 'name': '', 'message': str(exc), 'camera': camera, 'time': time.time(),
                        'frame_id': int(time.time() * 1000) if getattr(self, 'last_frame', None) else 0}
            return f'OCR failed: {exc}'

    def read_now(self, camera):
        if self.busy():
            return {'success': False, 'message': 'A task is running'}
        self.thread = threading.Thread(target=self._read_now, args=(int(camera),), daemon=True)
        self.thread.start()
        return {'success': True, 'message': 'Reading one frame'}

    def _read_now(self, camera):
        self.read_card(camera if camera in self.s.cameras else 1)

    def _call_ok(self, service, **fields):
        r = self.s.call(service, **fields)
        if not r.success:
            raise RuntimeError(f'{service}: {r.message}')

    def _safe_stop(self):
        self.s.base_cmd = None  # the base never keeps a velocity command after a stop or a failure
        try:
            self.s.call('dispense_stop', timeout=3.)
        except Exception:
            pass

    def _line_report(self, i):
        """After a straight line: where the fingertips really are versus the ideal end of the line (measured at
        rest from the encoders). Diagnostic only, it never breaks the task."""
        try:
            info = self.line_info[i]
            start = self._pose_ticks(info['start']) if isinstance(info['start'], str) else list(info['start'])
            e = self.s.planner_call('/tcp_error', {'start_ticks': start, 'ideal_delta_m': info['ideal_m'],
                                                   'measured_ticks': self.s.arm_ticks()})['error_mm']
            return f'; end of line: fingertips {e[0]:+.1f} mm in x, {e[1]:+.1f} in y, {e[2]:+.1f} in z vs the ideal line end'
        except Exception as exc:  # noqa: BLE001
            return f'; end-of-line measurement unavailable ({exc})'

    def _start_drive(self, meters):
        """Drive the base on a thread while the arm leg runs. Returns a handle for _finish_drive."""
        box = {'stop': False, 'error': None, 'travelled': None}

        def run():
            try:
                box['travelled'] = self.s.drive(meters, DRIVE_SPEED_M_S, lambda: box['stop'] or self.cancel)
            except Exception as exc:  # noqa: BLE001 - reported by _finish_drive
                box['error'] = exc
        box['thread'] = threading.Thread(target=run, daemon=True)
        box['thread'].start()
        return box

    @staticmethod
    def _finish_drive(box, abort):
        if abort:
            box['stop'] = True
        box['thread'].join(30.)
        if box['error'] is not None:
            raise RuntimeError(str(box['error']))
        return box['travelled']

    def _run_leg(self, i, st):
        # The robot refuses to start a trajectory while any arm motor still moves (more than 10 ticks/s):
        # wait for 0.6 s of calm (up to 8 s), e.g. after a pull against the letter, and say so if it never comes.
        if not self.s.wait_arm_still(timeout=8., hold_s=.6):
            speeds = getattr(self.s, 'arm_speeds', lambda: [])()
            raise RuntimeError(f'The arm keeps moving after the previous leg (joint speeds {speeds} ticks/s): '
                               'check that the letter is free and the gripper holds it')
        meters = st.get('drive_cm', 0.) / 100.
        if meters:
            self.s.drive_check()  # before anything moves: base on, ACTIVE, odometry alive
        start = time.monotonic()
        if st.get('dispenser_s'):
            self._call_ok('dispense', duration_s=float(st['dispenser_s']))
            dispenser_end = start + st['dispenser_s']
        else:
            dispenser_end = None
        drive = None
        try:
            r = self.s.execute(self.plans[i], False)
            if not r['success']:
                raise RuntimeError(r['message'])
            if meters:  # the base moves while the arm goes to its pose
                drive = self._start_drive(meters)
            while self.s.go_result is None:
                if self.cancel:
                    if self.s.go_handle is not None:
                        self.s.go_handle.cancel_goal_async()
                if drive is not None and drive['error'] is not None:  # the base stopped on an error: hold the arm too
                    if self.s.go_handle is not None:
                        self.s.go_handle.cancel_goal_async()
                time.sleep(.05)
            if drive is not None and drive['error'] is not None:
                raise RuntimeError(str(drive['error']))
            result = self.s.go_result
            if not result['success']:
                raise RuntimeError(result['message'])
        except BaseException:
            if drive is not None:  # whatever stopped the leg, the base stops too
                drive['stop'] = True
                drive['thread'].join(30.)
            raise
        message = result['message']
        if st.get('line_cm'):
            message += self._line_report(i)
        if drive is not None:
            travelled = self._finish_drive(drive, False)
            message += f'; base {"forward" if meters > 0 else "back"} {travelled * 100:.1f} cm (wheel odometry)'
        if dispenser_end is not None:
            while time.monotonic() < dispenser_end + .3:
                self._check()
                time.sleep(.05)
        return message

    def _run(self):
        steps = self.defs
        try:
            for i, st in enumerate(steps):
                self._check()
                self._set(current=i, fsm={'state': st.get('state', ''), 'letter': st.get('letter')})
                if st.get('letter') and st.get('state') in ('PICK', 'READ', 'APPROACH', 'LOCATE', 'INSERT', 'RETREAT'):
                    self._letter(st['letter'], status=st['state'].lower())
                self._step(i, status='running')
                if st['kind'] == 'survey':
                    detail = self._survey()
                elif st['kind'] == 'plan_legs':
                    detail = self._plan_legs(st)
                elif st['kind'] == 'leg':
                    if st.get('gripper_open'):
                        self._call_ok('gripper', ticks=int(st['gripper_open']))
                    detail = self._run_leg(i, st)
                    if st.get('gripper_open'):  # never go on to grab with a closed gripper
                        pos = self._wait_gripper(st['gripper_open'], 15, 10., True)
                        detail += f'; gripper open at {pos}'
                elif st['kind'] == 'ocr':
                    detail = self.read_card()
                    if st.get('verify'):
                        detail += self._verify_recipient(detail, st.get('letter'))
                elif st['kind'] == 'detect' and st.get('live'):
                    name = (self.ocr or {}).get('name')
                    if not name:
                        raise RuntimeError('No card name was read: nothing to deliver (the letter stays in the gripper)')
                    self.insert['recipient'] = name
                    self._plan_insert(self.speed, i, name)
                    detail = self.steps[i]['detail']
                elif st['kind'] == 'detect':
                    detail = 'done while planning'
                else:
                    self._call_ok('gripper', ticks=int(st['ticks']))
                    pos = self._wait_gripper(st['ticks'], 15, 6., False)
                    detail = f'gripper at {pos} ticks' + ('' if abs(pos - st['ticks']) <= 15 else ' (stopped before target: holding the letter?)')
                self._step(i, status='failed' if (st['kind'] == 'ocr' and not (self.ocr or {}).get('success')) else 'ok',
                           detail=detail)
                if st.get('last_of_letter'):
                    self._letter(st['letter'], status='delivered')
            ocr = self.ocr or {}
            if TASKS[self.task].get('mission'):
                done = ', '.join(f'#{l["letter"]} {l["name"]} -> mailbox {l["mailbox"]}' for l in self.letters)
                self._set(phase='done', current=len(steps), fsm={'state': 'DONE', 'letter': None},
                          message=f'All {len(self.letters)} letters delivered ({done}); the arm is back at scan')
                return
            if self.task in ('insert_letter', 'pick_deliver'):
                self._set(phase='done', current=len(steps),
                          message=f'Letter released (gripper open at {OPEN_GRIPPER_TICKS}); the arm is back at scan')
                return
            self._set(phase='done', current=len(steps),
                      message='Task finished at read_card' + (f'; card name: {ocr["name"]}' if ocr.get('name')
                                                              else '; no known name read' if ocr.get('success') else '; OCR failed'))
        except Exception as exc:
            self._safe_stop()
            if 0 <= self.current < len(self.steps):
                self._step(self.current, status='failed', detail=str(exc))
            cancelled = isinstance(exc, TaskAborted) or self.cancel
            if self.fsm.get('letter'):
                self._letter(self.fsm['letter'], status='canceled' if cancelled else 'failed')
            self._set(phase='canceled' if cancelled else 'failed',
                      fsm={'state': 'CANCELED' if cancelled else 'FAILED', 'letter': self.fsm.get('letter')},
                      message=f'Task stopped: {exc}. Arm held where it is; nothing resumes by itself.')
