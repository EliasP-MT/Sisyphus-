"""Holonomic navigation on the estimated occupancy grid, using scikit-image MCP."""
import numpy as np
from skimage.draw import line
from skimage.graph import route_through_array

from .common import rotation, wrap


class HolonomicNavigator:
    def __init__(self, c):
        self.c = c
        self.path = []
        self.goal = None
        self.last_plan = -np.inf
        self.blocked_since = None
        self.status = 'idle'

    def command(self, goal, slam, observation):
        now = float(observation['time'][0])
        goal = np.asarray(goal, dtype=float)
        pose = slam.pose
        delta = goal[:2]-pose[:2]
        settings = self.c['navigation']
        if not slam.healthy(now):
            self.status = 'sensor_stale'
            return np.zeros(3), False
        if np.linalg.norm(delta) < settings['goal_tolerance_m']:
            yaw_error = wrap(goal[2]-pose[2])
            self.status = 'arrived' if abs(yaw_error) < settings['yaw_tolerance_rad'] else 'aligning'
            return np.array([0, 0, np.clip(1.5*yaw_error, -settings['max_yaw_rad_s'], settings['max_yaw_rad_s'])]), self.status == 'arrived'
        if self.goal is None or np.linalg.norm(goal-self.goal) > 1e-6 or now-self.last_plan > .8:
            cost = slam.map.cost(settings['footprint_radius_m']+self.c['map']['resolution_m'])
            self.costmap = cost
            start, end = slam.map.pixel(pose[:2]), slam.map.pixel(goal[:2])
            self.last_plan, self.goal = now, goal.copy()
            self.path = []
            if slam.map.inside(start) and slam.map.inside(end) and np.isfinite(cost[start]) and np.isfinite(cost[end]):
                try:
                    pixels, _ = route_through_array(cost, start, end, fully_connected=True, geometric=True)
                    self.path = np.array([slam.map.world(rc) for rc in pixels])
                except ValueError:
                    pass
            if not len(self.path):
                if self.blocked_since is None: self.blocked_since = now
                self.status = 'blocked'
                return np.zeros(3), False
            self.blocked_since = None
        if not len(self.path):
            self.status = 'blocked'
            return np.zeros(3), False
        closest = int(np.argmin(np.linalg.norm(self.path-pose[:2], axis=1)))
        target = self.path[-1]
        for point in self.path[closest:]:
            if np.linalg.norm(point-pose[:2]) >= .16:
                target = point
                break
        if np.linalg.norm(delta) < .25:
            start, end = slam.map.pixel(pose[:2]), slam.map.pixel(goal[:2])
            rr, cc = line(*start, *end)
            if np.all(np.isfinite(self.costmap[rr, cc])): target = goal[:2]
        velocity = target-pose[:2]
        speed = min(settings['max_speed_m_s'], 1.2*np.linalg.norm(delta))
        velocity *= speed/max(np.linalg.norm(velocity), 1e-9)
        body = rotation(pose[2]).T @ velocity
        direction = np.arctan2(body[1], body[0])
        angles = np.linspace(-np.pi, np.pi, len(observation['scan']), endpoint=False)
        forward = np.abs(wrap(angles-direction)) < .5
        clearance = float(np.min(observation['scan'][forward]))
        if clearance < settings['footprint_radius_m']+.055:
            self.status = 'local_stop'
            return np.zeros(3), False
        if clearance < settings['footprint_radius_m']+.15: body *= .5
        omega = np.clip(1.2*wrap(goal[2]-pose[2]), -settings['max_yaw_rad_s'], settings['max_yaw_rad_s'])
        self.status = 'moving'
        return np.r_[body, omega], False
