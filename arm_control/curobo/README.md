# cuRobo GPU backend for the planner worker

The station planner worker (`ros2_robot/postal_station/planner_worker/worker.py`) uses cuRobo (NVIDIA, Apache-2.0)
as a GPU collision accelerator. The robot (RDK) is unchanged: it still receives a timed joint trajectory.

## What changes
- Direct path, one-via detour search, validation of the timed and tick-quantized trajectory, and the
  compensation box are checked densely on the GPU (about 7 µs per configuration, versus 6 ms for PyBullet).
- The exact PyBullet checker (`collision.py`) still decides. It runs on the start, the goal, samples at most
  0.05 rad apart, and the compensation box vertices. If the GPU rejects a plan, the full exact validation
  runs. If no direct or one-via path exists, OMPL runs as before.
- Ruckig timing, calibrated limits and the plan format are unchanged.
- Measured on recorded and random poses: planning takes 1.2–2.4 s (CPU: 17–28 s). Motion durations and
  compensation bounds are identical.

## Model
- `build_model.py` builds `elrobot.yml` (spheres fitted on the URDF meshes; the gripper is frozen closed).
  cuRobo needs axis-aligned joints, so the generated URDF re-expresses each joint frame (verified exact to 1e-12 m).
- `calibrate_self.py` writes `self_exempt.json`: the self-collision link pairs whose spheres overlap in more
  than 1 % of exactly-free poses. Those pairs are left to the exact check.
- `curobo_backend.py` checks against the scene with a sphere buffer of 6 mm on top of `margin_m`. The base group and the J1 stage are left to the exact check.
- Agreement on 3000 poses: 2 GPU-free/exact-collision cases, which the exact check covers, and 7 % false alarms.

## Install (station PC with an NVIDIA GPU)
```bash
git clone https://github.com/NVlabs/curobo ~/hack/curobo-src/curobo
python3 -m venv ~/hack/curobo-src/.venv && ~/hack/curobo-src/.venv/bin/pip install uv
cd ~/hack/curobo-src/curobo && ../.venv/bin/uv pip install --python ../.venv/bin/python ".[cu13-torch]"
../.venv/bin/uv pip install --python ../.venv/bin/python ikpy==3.4.2 ruckig==0.12.2 pybullet==3.2.7 ompl==2.0.1 cadquery-ocp==7.7.2 pytest
~/hack/curobo-src/.venv/bin/python arm_control/curobo/build_model.py        # if the URDF changed
~/hack/curobo-src/.venv/bin/python arm_control/curobo/calibrate_self.py     # after every model rebuild
```
`deploy/station.sh` uses this venv when it exists. `POSTAL_PLANNER_BACKEND=cpu` forces the old pipeline.
`elrobot.yml` contains absolute paths for this PC; rebuild it on another machine.
