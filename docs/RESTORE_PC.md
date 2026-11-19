# Recreate the PC cuRobo setup

This recipe restores the disposable `~/hack/curobo-src` directory from this
repository and upstream packages. It does **not** reconstruct robot-specific
calibrations, scenes, captured images or station state from Git. Keep the existing
`humanoid-hackathon` checkout and its ignored local files when cleaning up cuRobo.

## Recorded environment (2026-11-19)

- Ubuntu 24.04.3, Linux x86_64, Python 3.12.3.
- NVIDIA GeForce RTX 3050 Ti Laptop GPU; driver 595.99.02.
- PyTorch 2.14.1+cu130. A compatible NVIDIA driver and GPU are required.
- cuRobo commit `71ce1726925adfdcc7d0d39dabec5f2d6bb71acd` from
  <https://github.com/NVlabs/curobo> (not the moving `main` branch).
- All installed Python package versions are recorded in
  `arm_control/curobo/requirements-linux-py312.lock`.

Internet access and approximately 12 GB of free space are recommended during install
(including temporary downloads and the PyBullet build).
This snapshot depends on upstream Git and package indexes remaining available;
it is not an offline backup. Other Python versions/platforms are not covered.

## Restore after cloning

Install Git, a C++ compiler and Python 3.12 with development headers and venv
support first (Ubuntu 24.04 packages: `git build-essential python3.12-dev
python3.12-venv`). PyBullet is built from source for this Python version.
Install a compatible NVIDIA driver and confirm
`nvidia-smi` works. Then:

```bash
mkdir -p ~/hack
git clone git@github.com:EliasP-MT/Sisyphus-.git ~/hack/humanoid-hackathon
cd ~/hack/humanoid-hackathon
bash arm_control/curobo/restore.sh
```

The script downloads the recorded cuRobo commit, installs the pinned packages,
checks dependencies and performs imports plus a CUDA tensor operation. It does
not start the robot. It refuses to overwrite an existing destination. To install
elsewhere, use `CUROBO_ROOT=/absolute/new/path bash arm_control/curobo/restore.sh`
and set `PLANNER_PY=/absolute/new/path/.venv/bin/python` when starting the station.
On failure, retain the output for diagnosis; partial files remain at the chosen
destination and a retry requires a different empty destination or removal of
that partial installation.

`ros2_robot/deploy/station.sh` automatically selects
`~/hack/curobo-src/.venv/bin/python` when it exists. The CPU planner environment
can be recreated with `bash arm_control/setup.sh`; this fetches NormaCore commit
`6b494f77fac366a878ae71222458b196f8400dee` for the arm meshes.

## Models and local data

Reinstalling cuRobo on this PC at the same paths does not require regenerating
the robot model. The existing files in `arm_control/curobo/` remain in Git.
On another machine or after moving the checkout, the model contains absolute
paths and must be rebuilt. From the **humanoid repository root**, after restoring
the SDK with `bash arm_control/setup.sh`:

```bash
~/hack/curobo-src/.venv/bin/python arm_control/curobo/build_model.py
~/hack/curobo-src/.venv/bin/python arm_control/curobo/calibrate_self.py \
  --calibration /absolute/path/to/calibration.local.json \
  --scene /absolute/path/to/rover-scene.local.json
```

Calibration also requires `arm_control/poses.local.json` and scene mesh assets.
The calibration, poses, `arm_control/assets/rover-fixed.local/`, other ignored
assets and `station/station_data/` are machine/robot data, not dependencies.
Keep them locally or transfer them separately using a private backup. A fresh
clone alone is **not** sufficient to reproduce the complete robot installation.
See `ros2_robot/README.md` for ROS 2 and station setup.

## Cleanup scope

Validation on 2026-11-19: downloaded the pinned commit and packages into a new
temporary installation, built PyBullet and cuRobo, and passed `uv pip check`
(101 packages). Verified CUDA tensor execution and initialized the station's
`Planner` with the retained calibration and rover scene. GPU collision queries
passed both with existing caches and after compiling Warp kernels into an empty
cache. No station server or robot commands were started. This validates cuRobo
reinstallation on this PC, not a fresh ROS installation or hardware operation.

Only remove `~/hack/curobo-src` after the restoration has passed and these
instructions are pushed to GitHub. Check that no process uses that environment
and that the cuRobo source has no local changes. This frees about 8.1 GiB on the
recorded PC. Keep `~/hack/humanoid-hackathon`, including local data and the CPU
environment. GPU planning will require running the restore script again.
