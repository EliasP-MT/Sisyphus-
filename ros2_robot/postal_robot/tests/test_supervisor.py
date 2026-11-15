import unittest

from postal_robot.supervisor import (ACTIVE, AWAITING_ACK, FAULT, LINK_LOST, WAITING_FOR_STATION,
                                     Supervisor)


class Clock:
    t = 0.

    def __call__(self):
        return self.t


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.s = Supervisor(.8, self.clock)

    def test_requires_heartbeat_and_ack(self):
        self.assertEqual(self.s.state, WAITING_FOR_STATION)
        self.assertFalse(self.s.commands_allowed())
        self.s.heartbeat('a', 1)
        self.assertEqual(self.s.state, AWAITING_ACK)
        self.assertFalse(self.s.commands_allowed())
        ok, _ = self.s.acknowledge('a')
        self.assertTrue(ok and self.s.commands_allowed())

    def test_link_loss_and_no_automatic_resume(self):
        self.s.heartbeat('a', 1)
        self.s.acknowledge('a')
        self.clock.t = .5
        self.assertEqual(self.s.tick(), [])
        self.clock.t = 1.
        self.assertEqual(self.s.tick(), ['link_lost'])
        self.assertEqual(self.s.state, LINK_LOST)
        self.s.heartbeat('a', 2)
        self.assertEqual(self.s.state, AWAITING_ACK)
        self.assertFalse(self.s.commands_allowed())
        self.assertTrue(self.s.acknowledge('a')[0])

    def test_replayed_heartbeat_does_not_extend_lease(self):
        self.s.heartbeat('a', 5)
        self.s.acknowledge('a')
        self.clock.t = .7
        self.s.heartbeat('a', 5)
        self.s.heartbeat('a', 3)
        self.clock.t = .9
        self.assertEqual(self.s.tick(), ['link_lost'])

    def test_new_session_requires_ack(self):
        self.s.heartbeat('a', 1)
        self.s.acknowledge('a')
        self.assertEqual(self.s.heartbeat('b', 1), ['link_lost'])
        self.assertEqual(self.s.state, AWAITING_ACK)
        self.assertFalse(self.s.acknowledge('a')[0])
        self.assertTrue(self.s.acknowledge('b')[0])

    def test_fault_latches_until_condition_clears(self):
        self.s.heartbeat('a', 1)
        self.s.acknowledge('a')
        self.assertTrue(self.s.fault('wheel 9 stale'))
        self.assertFalse(self.s.fault('wheel 9 stale'))
        self.assertEqual(self.s.state, FAULT)
        self.assertFalse(self.s.commands_allowed())
        self.assertFalse(self.s.acknowledge('a', ['wheel 9 stale'])[0])
        self.assertTrue(self.s.acknowledge('a', [])[0])
        self.assertEqual((self.s.state, self.s.faults), (ACTIVE, []))

    def test_ack_needs_fresh_lease(self):
        self.s.heartbeat('a', 1)
        self.clock.t = 2.
        self.s.tick()
        self.assertFalse(self.s.acknowledge('a')[0])


if __name__ == '__main__':
    unittest.main()
