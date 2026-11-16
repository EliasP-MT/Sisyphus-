import unittest
from types import SimpleNamespace

from mailbox_slam.live_health import geometry_warnings, readiness_blockers


def state(**updates):
    values = dict(
        state=1,
        state_name="ACTIVE",
        base_enabled=True,
        arm_torque=True,
        arm_busy=False,
        dispenser_running=False,
        faults=[],
    )
    return SimpleNamespace(**(values | updates))


class HealthTests(unittest.TestCase):
    def blockers(self, s, **updates):
        args = dict(runtime_verified=True, supervisor_age_s=0.1, camera_age_s=0.1)
        return readiness_blockers(s, **(args | updates))

    def test_ready_and_lease_expiry(self):
        self.assertEqual(self.blockers(state()), [])
        result = self.blockers(state(state=2, state_name="LINK_LOST"))
        self.assertIn("LINK_LOST", result[0])
        self.assertIn("Station", result[0])

    def test_stale_streams_and_partial_startup_are_visible(self):
        result = self.blockers(None, runtime_verified=False, camera_age_s=2.0)
        self.assertEqual(len(result), 3)
        self.assertIn("supervisor state stale", self.blockers(state(), supervisor_age_s=0.5))

    def test_arm_and_base_conditions_never_bypassed(self):
        result = self.blockers(state(arm_busy=True, base_enabled=False, faults=["servo error"]))
        self.assertEqual(len(result), 3)

    def test_unvalidated_geometry_is_reported_not_silently_certified(self):
        self.assertEqual(len(geometry_warnings({"validated": False}, {}, {})), 3)
        self.assertEqual(
            geometry_warnings(
                {"validated": True}, {"hardware_verified": True}, {"validated": True}
            ),
            [],
        )
