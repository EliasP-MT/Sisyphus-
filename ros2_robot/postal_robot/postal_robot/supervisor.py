"""Station lease and fault supervision (pure logic, injected clock).

Contract:
* No actuator command is accepted until a heartbeat arrives AND the operator
  acknowledges it (WAITING_FOR_STATION -> AWAITING_ACK -> ACTIVE).
* Lease expiry in ACTIVE -> LINK_LOST: the caller must stop the base and the
  dispenser and hold the arm. Heartbeats returning -> AWAITING_ACK, never ACTIVE
  automatically. A new station session also requires acknowledgement.
* Faults are latched -> FAULT. Acknowledge succeeds only with a fresh lease
  from the current session and when no blocking condition remains.
"""
import time

WAITING_FOR_STATION, ACTIVE, LINK_LOST, AWAITING_ACK, FAULT = range(5)
NAMES = ['WAITING_FOR_STATION', 'ACTIVE', 'LINK_LOST', 'AWAITING_ACK', 'FAULT']


class Supervisor:
    def __init__(self, lease_timeout=.8, clock=time.monotonic):
        if not .1 <= lease_timeout <= 5:
            raise ValueError('Lease timeout must be within 0.1..5 s')
        self.lease_timeout, self.clock = lease_timeout, clock
        self.state = WAITING_FOR_STATION
        self.session, self.sequence, self.last_heartbeat = '', -1, None
        self.faults = []
        self.last_event = 'started; waiting for station heartbeat'
        self.acknowledged_at = None

    @property
    def name(self):
        return NAMES[self.state]

    def lease_age(self):
        return float('inf') if self.last_heartbeat is None else self.clock() - self.last_heartbeat

    def lease_fresh(self):
        return self.lease_age() <= self.lease_timeout

    def heartbeat(self, session, sequence):
        """Return a list of events ('link_lost' when a new session replaces an active one)."""
        if not session:
            return []
        events = []
        if session != self.session:
            if self.state == ACTIVE:
                self.state = AWAITING_ACK
                events.append('link_lost')
                self.last_event = 'new station session; acknowledgement required'
            self.session, self.sequence = session, -1
        if sequence <= self.sequence:
            return events  # replayed or reordered heartbeat: never extends the lease
        self.sequence, self.last_heartbeat = sequence, self.clock()
        if self.state in (WAITING_FOR_STATION, LINK_LOST):
            self.state = AWAITING_ACK
            self.last_event = 'station heartbeat received; acknowledgement required'
        return events

    def tick(self):
        if self.state == ACTIVE and not self.lease_fresh():
            self.state = LINK_LOST
            self.last_event = f'station lease expired after {self.lease_timeout:.2f} s'
            return ['link_lost']
        if self.state == AWAITING_ACK and not self.lease_fresh():
            self.state = LINK_LOST
            self.last_event = 'station lease expired before acknowledgement'
        return []

    def fault(self, message):
        """Latch a fault. Returns True when this changes the state to FAULT."""
        if message not in self.faults and len(self.faults) < 20:
            self.faults.append(message)
        self.last_event = f'fault: {message}'
        if self.state != FAULT:
            self.state = FAULT
            return True
        return False

    def acknowledge(self, session, blocking=()):
        if not self.lease_fresh():
            return False, 'No fresh station lease'
        if session != self.session:
            return False, 'Acknowledgement from a different station session'
        if blocking:
            return False, 'Fault still present: ' + '; '.join(blocking)
        if self.state == ACTIVE:
            return True, 'Already active'
        self.faults.clear()
        self.state = ACTIVE
        self.acknowledged_at = self.clock()
        self.last_event = 'operator acknowledged; commands accepted'
        return True, 'Acknowledged; commands issued from now on are accepted'

    def commands_allowed(self):
        return self.state == ACTIVE and self.lease_fresh()
