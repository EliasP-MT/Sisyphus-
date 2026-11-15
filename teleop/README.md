# Leader → follower teleop

Two ST3215 arms on one NormaCore station. Moving the leader by hand moves the follower.
The 50 Hz mirroring loop runs inside the station (its built-in `motors-mirroring`
driver, the same one the web UI dropdown uses). `teleop.py` sets it up safely.
Python stdlib only. The station must run with `./station --web --tcp`.

## First time

1. Plug in both arms and attach each to WSL (admin PowerShell):
   `usbipd list` → `usbipd bind --busid X-Y` (once) → `usbipd attach --wsl --busid X-Y`
2. `python3 teleop.py status`: both buses should be listed, with motors calibrated.
3. `python3 teleop.py identify`: move **only the leader** for 8 s. Roles get saved to `arms.json`.

## Every time

```bash
python3 teleop.py start            # align follower slowly, start mirroring, show tracking error
                                   # Ctrl+C = stop
python3 teleop.py start --detach   # leave mirroring running in the station
python3 teleop.py stop
python3 teleop.py status
```

Overrides: `--leader SERIAL --follower SERIAL`, `--host other-machine`, `--no-soft-start`,
`--soft-speed 400`.

## Safety

- **When mirroring stops, the station turns follower torque OFF and the arm goes limp.**
  Support the arm or put it in a rest pose before you stop.
- `start` turns off leader torque so you can move it by hand. It then moves the follower
  slowly (speed 400) to the leader's pose before mirroring takes over. Without this
  step, the station's mirroring would snap the follower across at up to speed 3300.
- Mirroring maps each joint by percentage of its calibrated range, so both arms must be calibrated.
- The station's current protection holds a joint if it pushes against something (threshold 100,
  see `current-threshold` in `station.yaml`).

`vendor/` holds the station Python client and generated protobufs from
norma-core `v0.1.0-beta.10`, matching the station binary.
