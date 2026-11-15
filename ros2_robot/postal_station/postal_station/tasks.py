"""Station-side task sequencer (pick and scan a letter, ...).

A task is a list of arm legs between stored poses with side actions (gripper,
dispenser). Everything is planned BEFORE the task starts: leg 0 from the measured
arm pose, every following leg from the previous stored pose (fixed legs, cached by
arm_motion). Execution then only chains already collision-checked plans; any
failure stops the dispenser, cancels the arm goal and leaves the arm held where it
is. Nothing resumes by itself."""
import base64
import json
import threading
import time
import urllib.request

PLAN_LIFETIME_S = 600.
OCR_URL = 'http://127.0.0.1:8894'
OCR_SKIP_FRAMES = 5   # as ocr_base.read_recipient_name: discard five frames, read the sixth

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
            {'kind': 'leg', 'pose': 'read_card', 'arrival_tol': .04, 'label': 'Go to read_card'},
            {'kind': 'ocr', 'label': 'Read the name on the card (OCR, one frame)'},
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
        self.ocr_camera = getattr(self, 'ocr_camera', 1)
        self.ocr = getattr(self, 'ocr', None)
        self.steps, self.plans, self.prepared_at = [], {}, 0.
        self.current = -1

    # ------------------------------------------------------------------ state
    def snapshot(self):
        with self.lock:
            return {'task': self.task, 'phase': self.phase, 'message': self.message, 'current': self.current,
                    'steps': [dict(st) for st in self.steps], 'prepared_age_s': time.monotonic() - self.prepared_at
                    if self.prepared_at else None,
                    'tasks': {k: v['title'] for k, v in TASKS.items()}, 'ocr': self.ocr}

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
    def prepare(self, name, speed, ocr_camera=1):
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
                elif st['kind'] == 'ocr':
                    detail = self.read_card()
                else:
                    self._call_ok('gripper', ticks=int(st['ticks']))
                    pos = self._wait_gripper(st['ticks'], 15, 6., False)
                    detail = f'gripper at {pos} ticks' + ('' if abs(pos - st['ticks']) <= 15 else ' (stopped before target: holding the letter?)')
                self._step(i, status='failed' if (st['kind'] == 'ocr' and not (self.ocr or {}).get('success')) else 'ok',
                           detail=detail)
            ocr = self.ocr or {}
            self._set(phase='done', current=len(steps),
                      message='Task finished at read_card' + (f'; card name: {ocr["name"]}' if ocr.get('name')
                                                              else '; no known name read' if ocr.get('success') else '; OCR failed'))
        except Exception as exc:
            self._safe_stop()
            if 0 <= self.current < len(self.steps):
                self._step(self.current, status='failed', detail=str(exc))
            self._set(phase='canceled' if (isinstance(exc, TaskAborted) or self.cancel) else 'failed',
                      message=f'Task stopped: {exc}. Arm held where it is; nothing resumes by itself.')
