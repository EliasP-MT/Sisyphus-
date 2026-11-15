import unittest
from station import fresh_feedback


class FeedbackTests(unittest.IsolatedAsyncioTestCase):
    async def test_waits_for_new_feedback_without_disabling_age_check(self):
        class Stream:
            calls = 0
            def state(self):
                self.calls += 1
                if self.calls < 3:
                    raise RuntimeError('Moteur 1 : mesure périmée')
                return ['fresh']
        stream = Stream()
        self.assertEqual(await fresh_feedback(stream), ['fresh'])
        self.assertEqual(stream.calls, 3)

    async def test_non_freshness_error_not_retried(self):
        class Stream:
            def state(self):
                raise RuntimeError('servo error')
        with self.assertRaisesRegex(RuntimeError, 'servo error'):
            await fresh_feedback(Stream())

    async def test_stale_forever_refuses_motion(self):
        class Stream:
            def state(self):
                raise RuntimeError('État Station absent ou périmé')
        with self.assertRaisesRegex(RuntimeError, 'no motion started'):
            await fresh_feedback(Stream(), timeout=0)
