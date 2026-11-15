"""Station-side task sequencer (pick and scan a letter, ...).

A task is a list of arm legs between stored poses with side actions (gripper,
dispenser). Everything is planned BEFORE the task starts: leg 0 from the measured
arm pose, every following leg from the previous stored pose (fixed legs, cached by
arm_motion). Execution then only chains already collision-checked plans; any
failure stops the dispenser, cancels the arm goal and leaves the arm held where it
is. Nothing resumes by itself."""
import threading
import time

PLAN_LIFETIME_S = 600.

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
            {'kind': 'leg', 'pose': 'pull', 'arrival_tol': .05, 'label': 'Go to pull'},
            {'kind': 'leg', 'pose': 'up_after_pull', 'arrival_tol': .05, 'label': 'Go to up_after_pull'},
            {'kind': 'leg', 'pose': 'read_card', 'arrival_tol': .04, 'label': 'Go to read_card (stop)'},
        ],
    },
}


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
        self.steps, self.plans, self.prepared_at = [], {}, 0.
        self.current = -1

    # ------------------------------------------------------------------ state
    def snapshot(self):
        with self.lock:
            return {'task': self.task, 'phase': self.phase, 'message': self.message, 'current': self.current,
                    'steps': [dict(st) for st in self.steps], 'prepared_age_s': time.monotonic() - self.prepared_at
                    if self.prepared_at else None,
                    'tasks': {k: v['title'] for k, v in TASKS.items()}}

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
    def prepare(self, name, speed):
        if name not in TASKS:
            raise KeyError(f'unknown task {name}')
        if self.busy():
            return {'success': False, 'message': 'A task is already planning or running'}
        if self.s.go_handle is not None and self.s.go_result is None:
            return {'success': False, 'message': 'An arm motion is running'}
        self.cancel = False
        self.reset(name)
        self.steps = [{'label': st['label'], 'status': 'pending', 'detail': ''} for st in TASKS[name]['steps']]
        self.phase = 'planning'
        self.thread = threading.Thread(target=self._prepare, args=(name, speed), daemon=True)
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
    def _prepare(self, name, speed):
        i = -1
        try:
            previous = TASKS[name].get('start_pose', '')
            previous_tol = .05  # the arm is expected near the start pose
            plans = {}
            for i, st in enumerate(TASKS[name]['steps']):
                if st['kind'] != 'leg':
                    self._step(i, status='ok', detail='no planning needed')
                    continue
                if self.cancel:
                    raise TaskAborted('planning canceled')
                self._step(i, status='planning')
                began = time.monotonic()
                r = self.s.call('plan', timeout=110., pose_name=st['pose'], speed_factor=float(speed),
                                start_pose_name=previous, valid_for_s=PLAN_LIFETIME_S,
                                arrival_tolerance_rad=st.get('arrival_tol', 0.), start_tolerance_rad=max(st.get('start_tol', 0.), previous_tol))
                if not r.success or not r.executable:
                    why = r.message if not r.success else '; '.join(r.reasons)
                    raise RuntimeError(f'{st["pose"]}: {why}')
                plans[i] = r.plan_id
                self._step(i, status='ok', duration_s=round(r.duration_s, 1),
                           detail=f'{r.duration_s:.1f} s, planned in {time.monotonic() - began:.1f} s')
                previous, previous_tol = st['pose'], st.get('arrival_tol', 0.)
            with self.lock:
                self.plans, self.prepared_at = plans, time.monotonic()
                self.phase, self.message = 'ready', 'All legs planned; ready to start'
        except Exception as exc:
            if i >= 0:
                self._step(i, status='failed', detail=str(exc))
            self._set(phase='failed', message=f'Planning failed: {exc}')

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

    def _call_ok(self, service, **fields):
        r = self.s.call(service, **fields)
        if not r.success:
            raise RuntimeError(f'{service}: {r.message}')

    def _safe_stop(self):
        try:
            self.s.call('dispense_stop', timeout=3.)
        except Exception:
            pass

    def _run_leg(self, i, st):
        start = time.monotonic()
        if st.get('dispenser_s'):
            self._call_ok('dispense', duration_s=float(st['dispenser_s']))
            dispenser_end = start + st['dispenser_s']
        else:
            dispenser_end = None
        r = self.s.execute(self.plans[i], False)
        if not r['success']:
            raise RuntimeError(r['message'])
        while self.s.go_result is None:
            if self.cancel:
                if self.s.go_handle is not None:
                    self.s.go_handle.cancel_goal_async()
            time.sleep(.05)
        result = self.s.go_result
        if not result['success']:
            raise RuntimeError(result['message'])
        if dispenser_end is not None:
            while time.monotonic() < dispenser_end + .3:
                self._check()
                time.sleep(.05)
        return result['message']

    def _run(self):
        steps = TASKS[self.task]['steps']
        try:
            for i, st in enumerate(steps):
                self._check()
                self._set(current=i)
                self._step(i, status='running')
                if st['kind'] == 'leg':
                    if st.get('gripper_open'):
                        self._call_ok('gripper', ticks=int(st['gripper_open']))
                    detail = self._run_leg(i, st)
                    if st.get('gripper_open'):  # never go on to grab with a closed gripper
                        pos = self._wait_gripper(st['gripper_open'], 15, 10., True)
                        detail += f'; gripper open at {pos}'
                else:
                    self._call_ok('gripper', ticks=int(st['ticks']))
                    pos = self._wait_gripper(st['ticks'], 15, 6., False)
                    detail = f'gripper at {pos} ticks' + ('' if abs(pos - st['ticks']) <= 15 else ' (stopped before target: holding the letter?)')
                self._step(i, status='ok', detail=detail)
            self._set(phase='done', message='Task finished at read_card', current=len(steps))
        except Exception as exc:
            self._safe_stop()
            if 0 <= self.current < len(self.steps):
                self._step(self.current, status='failed', detail=str(exc))
            self._set(phase='canceled' if (isinstance(exc, TaskAborted) or self.cancel) else 'failed',
                      message=f'Task stopped: {exc}. Arm held where it is; nothing resumes by itself.')
