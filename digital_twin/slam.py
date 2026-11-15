"""Reference SE(2) range SLAM: SciPy scan matching + sparse pose graph + occupancy.

Inputs are wheel increments and range scans only. No MuJoCo pose/world geometry.
"""
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree
from skimage.draw import disk, line

from .common import KiwiKinematics, compose, relative, rotation, wrap


@dataclass
class Keyframe:
    pose: np.ndarray
    odom: np.ndarray
    points: np.ndarray
    normals: np.ndarray
    scan: np.ndarray


class OccupancyMap:
    def __init__(self, settings):
        self.resolution = settings['resolution_m']
        self.origin = np.array(settings['origin_m'])
        self.shape = tuple(np.ceil(np.array(settings['size_m'])[::-1]/self.resolution).astype(int))
        self.logodds = np.zeros(self.shape)

    def pixel(self, xy):
        point = np.floor((np.asarray(xy)-self.origin)/self.resolution).astype(int)
        return int(point[1]), int(point[0])

    def world(self, rc):
        return self.origin+(np.asarray(rc)[::-1]+.5)*self.resolution

    def inside(self, rc): return 0 <= rc[0] < self.shape[0] and 0 <= rc[1] < self.shape[1]

    def add_scan(self, pose, scan, angles, laser):
        origin = np.asarray(pose[:2])+rotation(pose[2]) @ np.asarray(laser['xyz_m'][:2])
        start = self.pixel(origin)
        if not self.inside(start): return
        directions = np.c_[np.cos(angles+pose[2]), np.sin(angles+pose[2])]
        for distance, direction in zip(scan, directions):
            end = self.pixel(origin+distance*direction)
            rr, cc = line(*start, *end)
            valid = (rr >= 0) & (rr < self.shape[0]) & (cc >= 0) & (cc < self.shape[1])
            self.logodds[rr[valid], cc[valid]] -= .35
            if self.inside(end) and distance < laser['max_range_m']-.03:
                self.logodds[end] += 1.25
        rr, cc = disk(start, .08/self.resolution, shape=self.shape)
        self.logodds[rr, cc] = -4
        np.clip(self.logodds, -5, 5, out=self.logodds)

    def cost(self, radius):
        occupied = self.logodds > .7
        clearance = distance_transform_edt(~occupied)*self.resolution
        cost = np.where(self.logodds < -.4, 1.0, 2.5)
        cost += 0.1/np.maximum(clearance, .04)
        cost[clearance < radius] = np.inf
        return cost

    def ros_grid(self):
        return np.where(np.abs(self.logodds) < .4, -1, np.where(self.logodds > .7, 100, 0)).astype(np.int8)


class RangeSlam:
    def __init__(self, c):
        self.c, self.kinematics = c, KiwiKinematics(c)
        self.pose = np.array(c['start_pose'], dtype=float)
        self.odometry_pose = self.pose.copy()
        self.map = OccupancyMap(c['map'])
        self.angles = np.linspace(-np.pi, np.pi, c['lidar']['rays'], endpoint=False)
        self.keyframes, self.edges = [], []
        self.tree, self.target, self.target_normals = None, None, None
        self.last_scan_time = -np.inf
        self.quality = dict(rms_m=None, inliers=0, accepted=False)
        self.loop_closures = 0

    def scan_points(self, ranges):
        points = np.c_[np.cos(self.angles), np.sin(self.angles)]*ranges[:, None]
        points += self.c['lidar']['xyz_m'][:2]
        tangent = np.roll(points, -1, axis=0)-np.roll(points, 1, axis=0)
        lengths = np.linalg.norm(tangent, axis=1)
        normals = np.c_[-tangent[:, 1], tangent[:, 0]]/np.maximum(lengths[:, None], 1e-6)
        valid = (ranges > .04) & (ranges < self.c['lidar']['max_range_m']-.03) & (lengths < .4) & (lengths > 1e-5)
        return points[valid], normals[valid]

    def _target_map(self):
        frames = self.keyframes[-40:]
        self.target = np.vstack([f.points @ rotation(f.pose[2]).T+f.pose[:2] for f in frames])
        self.target_normals = np.vstack([f.normals @ rotation(f.pose[2]).T for f in frames])
        self.tree = cKDTree(self.target)

    def _match(self, points, prediction, target=None, normals=None):
        target = self.target if target is None else target
        normals = self.target_normals if normals is None else normals
        tree = self.tree if target is self.target else cKDTree(target)
        estimate = prediction.copy()
        count, rms = 0, np.inf
        for _ in range(5):
            transformed = points @ rotation(estimate[2]).T+estimate[:2]
            distances, indices = tree.query(transformed)
            good = distances < .18
            if np.count_nonzero(good) < 20: break
            p, q, n = points[good], target[indices[good]], normals[indices[good]]
            def residual(x):
                delta = (p @ rotation(x[2]).T+x[:2])-q
                prior = np.array([*(x[:2]-prediction[:2]), wrap(x[2]-prediction[2])])*np.array([.025, .025, .02])
                return np.r_[np.sum(delta*n, axis=1), prior]
            result = least_squares(residual, estimate, loss='soft_l1', f_scale=.01, max_nfev=12)
            update = np.linalg.norm(result.x-estimate)
            estimate = result.x
            count = int(np.count_nonzero(good))
            rms = float(np.sqrt(np.mean(residual(estimate)[:-3]**2)))
            if update < 1e-5: break
        correction = relative(prediction, estimate)
        accepted = bool(count >= 20 and rms < .045 and np.linalg.norm(correction[:2]) < .2 and abs(correction[2]) < .18)
        return (estimate if accepted else prediction), dict(rms_m=rms if np.isfinite(rms) else None, inliers=count, accepted=accepted)

    def update(self, observation):
        increment = self.kinematics.body(observation['wheel_delta'])
        # Midpoint integration preserves all three holonomic degrees of freedom.
        increment[:2] = rotation(increment[2]/2) @ increment[:2]
        self.odometry_pose = compose(self.odometry_pose, increment)
        self.pose = compose(self.pose, increment)
        if not observation['scan_fresh']: return self.pose.copy()
        ranges = np.asarray(observation['scan'])
        points, normals = self.scan_points(ranges)
        if len(points) < 20: return self.pose.copy()
        self.last_scan_time = float(observation['time'][0])
        if self.tree is not None:
            self.pose, self.quality = self._match(points, self.pose)
        else:
            self.quality = dict(rms_m=0, inliers=len(points), accepted=True)
        self.map.add_scan(self.pose, ranges, self.angles, self.c['lidar'])
        if (not self.keyframes or np.linalg.norm(relative(self.keyframes[-1].pose, self.pose)[:2]) > .13
                or abs(wrap(self.pose[2]-self.keyframes[-1].pose[2])) > .15):
            self._keyframe(points, normals, ranges)
        return self.pose.copy()

    def _keyframe(self, points, normals, ranges):
        index = len(self.keyframes)
        if index:
            previous = self.keyframes[-1]
            self.edges.append((index-1, index, relative(previous.odom, self.odometry_pose), 1.0))
            self.edges.append((index-1, index, relative(previous.pose, self.pose), 8.0))
        current = Keyframe(self.pose.copy(), self.odometry_pose.copy(), points.copy(), normals.copy(), ranges.copy())
        self.keyframes.append(current)
        if index >= 12:
            candidates = [(np.linalg.norm(f.pose[:2]-self.pose[:2]), i)
                          for i, f in enumerate(self.keyframes[:-8]) if abs(wrap(f.pose[2]-self.pose[2])) < .6]
            if candidates:
                distance, other = min(candidates)
                if distance < .28:
                    old = self.keyframes[other]
                    target = old.points @ rotation(old.pose[2]).T+old.pose[:2]
                    normals_world = old.normals @ rotation(old.pose[2]).T
                    pose, quality = self._match(points, self.pose, target, normals_world)
                    if quality['accepted'] and quality['rms_m'] < .025 and quality['inliers'] > len(points)*.55:
                        self.edges.append((other, index, relative(old.pose, pose), 20.0))
                        self._optimise_graph()
                        self.loop_closures += 1
        self._target_map()

    def _optimise_graph(self):
        poses = np.array([f.pose for f in self.keyframes])
        fixed = poses[0].copy()
        def unpack(x): return np.vstack([fixed, x.reshape(-1, 3)])
        def residual(x):
            p = unpack(x)
            result = []
            for i, j, z, weight in self.edges:
                error = relative(p[i], p[j])-z
                error[2] = wrap(error[2])
                result.extend(error*np.sqrt(weight)*np.array([1, 1, .4]))
            return result
        sparsity = lil_matrix((len(self.edges)*3, (len(poses)-1)*3), dtype=int)
        for row, (i, j, _, _) in enumerate(self.edges):
            for index in (i, j):
                if index: sparsity[row*3:row*3+3, (index-1)*3:index*3] = 1
        result = least_squares(residual, poses[1:].ravel(), jac_sparsity=sparsity.tocsr(), loss='soft_l1', f_scale=.03, max_nfev=30)
        corrected = unpack(result.x)
        for frame, pose in zip(self.keyframes, corrected): frame.pose = pose.copy()
        self.pose = corrected[-1].copy()
        self.map.logodds.fill(0)
        for frame in self.keyframes: self.map.add_scan(frame.pose, frame.scan, self.angles, self.c['lidar'])

    def healthy(self, now):
        return now-self.last_scan_time < .6 and self.quality['accepted']
