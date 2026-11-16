"""GPU-accelerated path search, plan validation and compensation bounds (cuRobo backend).

Safety model: every decision combines the dense GPU check (curobo_backend, all samples at edge_step)
with the exact PyBullet check (collision.py) on the start, the goal and samples at most `exact_step`
rad apart. The GPU never relaxes the exact check; it replaces the bulk of the 5-8 ms exact queries."""
import itertools

import numpy as np


class GpuPlanning:
    def __init__(self, gpu, world, exact_step=0.05, via_samples=1024, seed=0):
        self.gpu, self.world = gpu, world
        self.exact_step = float(exact_step)
        self.via_samples = int(via_samples)
        self.rng = np.random.default_rng(seed)
        self.exact_checks = 0

    # ---- exact check on a sparse subset ---------------------------------------------------------------
    def _exact(self, q):
        self.exact_checks += 1
        return self.world.valid(q)

    def exact_sparse(self, samples):
        """Exact check of the first, last, and samples at most exact_step apart (max joint delta)."""
        samples = np.asarray(samples, float)
        last = samples[0]
        if not self._exact(last):
            return False
        for q in samples[1:-1]:
            if np.max(np.abs(q - last)) >= self.exact_step:
                if not self._exact(q):
                    return False
                last = q
        return self._exact(samples[-1])

    def path_ok(self, path):
        dense = self.gpu.densify(path)
        return bool(np.all(self.gpu.valid_batch(dense))) and self.exact_sparse(dense)

    # ---- path search ------------------------------------------------------------------------------------
    def joint_path(self, start, goal):
        """[start, goal] if the straight joint motion is free, else [start, via, goal] with the shortest
        free via among random samples (one GPU batch), else None (caller falls back to OMPL)."""
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        self.world.require_pose(start, 'Départ')
        self.world.require_pose(goal, 'Arrivée')
        if self.path_ok([start, goal]):
            return [start, goal]
        lo, hi = self.gpu.bounds[:, 0], self.gpu.bounds[:, 1]
        mid = (start + goal) / 2
        vias = np.concatenate([
            self.rng.uniform(lo, hi, (self.via_samples // 2, 7)),
            np.clip(mid + self.rng.normal(0, 0.6, (self.via_samples // 2, 7)), lo, hi)])
        vias = vias[self.gpu.valid_batch(vias)]
        if not len(vias):
            return None
        cost = np.abs(vias - start).max(1) + np.abs(goal - vias).max(1)
        vias = vias[np.argsort(cost)]
        coarse = 4 * self.gpu.edge_step
        # Screen candidates coarsely in batches, then verify the best ones densely + exactly.
        for k in range(0, len(vias), 128):
            batch = vias[k:k + 128]
            ok = np.array([True] * len(batch))
            pts = [self.gpu.densify([start, v, goal], coarse) for v in batch]
            sizes = [len(p) for p in pts]
            flags = self.gpu.valid_batch(np.concatenate(pts))
            idx = np.cumsum([0] + sizes)
            ok = np.array([flags[a:b].all() for a, b in zip(idx, idx[1:])])
            for v in batch[ok][:8]:
                if self.path_ok([start, v, goal]):
                    return [start, v, goal]
        return None

    # ---- validation ---------------------------------------------------------------------------------------
    def validate_rows(self, rows):
        """Timed samples (already joint-continuous): dense GPU check with edge_step interpolation, plus the
        exact check on a sparse subset. Raises ValueError like collision.validate_plan."""
        q = np.array([r['q_rad'] for r in rows], float)
        dense = self.gpu.densify(q)
        if not np.all(self.gpu.valid_batch(dense)):
            raise ValueError('Collision sur la trajectoire temporisée (GPU)')
        if not self.exact_sparse(q):
            raise ValueError('Collision sur la trajectoire temporisée (contrôle exact)')

    # ---- compensation box ------------------------------------------------------------------------------
    def compensation_box(self, final, in_range, limit=.06):
        """Same algorithm as worker._compensation_box: largest per-joint bound with a free motion from the
        final pose, then every box vertex (and the straight motion to it) valid, halving until it is.
        GPU for every motion; exact check on the final vertices."""
        final = np.asarray(final, float)

        def motions_ok(targets):
            targets = [t for t in targets]
            if not targets:
                return np.zeros(0, bool)
            rng_ok = np.array([in_range(t) for t in targets])
            pts = [self.gpu.densify([final, t]) for t in targets]
            idx = np.cumsum([0] + [len(p) for p in pts])
            flags = self.gpu.valid_batch(np.concatenate(pts))
            return rng_ok & np.array([flags[a:b].all() for a, b in zip(idx, idx[1:])])

        candidates = (limit, .04, .025, .012)
        targets, keys = [], []
        for j in range(7):
            for b in candidates:
                for d in (-b, b):
                    t = final.copy()
                    t[j] += d
                    targets.append(t)
                    keys.append((j, b))
        ok = motions_ok(targets)
        good = {}
        for (j, b), v in zip(keys, ok):
            good[(j, b)] = good.get((j, b), True) and bool(v)
        bounds = [next((b for b in candidates if good[(j, b)]), 0.) for j in range(7)]
        for _ in range(4):
            active = [j for j in range(7) if bounds[j] > 0]
            if not active:
                break
            verts = []
            for signs in itertools.product((-1., 1.), repeat=len(active)):
                t = final.copy()
                for j, s in zip(active, signs):
                    t[j] += s * bounds[j]
                verts.append(t)
            if motions_ok(verts).all() and all(self._exact(v) for v in verts):
                break
            bounds = [b / 2 if b / 2 >= .006 else 0. for b in bounds]
        else:
            bounds = [0.] * 7
        return bounds
