# Station on RDK X5

## Read-only 3D arm view

`arm-view.py` and `arm-view.html` provide an ElRobot URDF view restricted to
arm IDs 1–8. The backend only sends position-register READ packets; there
are no movement or calibration controls. Its pose is approximate until
joint order, offsets and ranges have been calibrated. Model assets and
JavaScript dependencies require internet access in the laptop browser.

The deployed transient user service is `arm-view.service`, listening on
RDK loopback port 8890. The current laptop SSH tunnel exposes it at
http://127.0.0.1:18890. Stop it before starting another serial controller:

```bash
systemctl --user stop arm-view.service
```

It is not enabled at boot. To recreate it on the RDK:

```bash
systemctl --user stop rdk-station.service
systemd-run --user --unit=arm-view --property=WorkingDirectory=/home/sunrise/station \
  python3 /home/sunrise/station/arm-view.py \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E114718-if00
```

From the laptop, leave this tunnel running:

```bash
ssh -N -L 127.0.0.1:18890:127.0.0.1:8890 sunrise@192.168.127.10
```

User lingering was enabled so user services remain running after SSH logout.
This does not enable the transient viewer at boot. Current wheel IDs are
11 (left), 12 (back), 9 (right); extra servo ID is 10.

Station runs on the RDK and communicates with its USB servo adapter. The laptop
only displays the web UI and can run the existing Python client through SSH.
Station is separate from the ROS rover controller; both must not open the same
serial bus simultaneously.

The repository's `station/station` binary is Linux x86-64, intended for the laptop.
The RDK is aarch64 and needs the **ARM64 build of the same v0.1.0-beta.10 release**.
The existing `teleop/teleop.py` and `teleop/vendor` are reused without modification.

## Installation (already performed on the RDK)

From the laptop repository root:

```bash
bash station/setup-rdk.sh
```

Set `RDK_HOST=sunrise@192.168.127.10` before the command to use direct Ethernet.
The installer verifies the ARM64 release archive SHA256, extracts only Station,
and copies the existing Python client. It preserves the laptop x86-64 installation
and does not copy laptop robot keys, recorded data, or leader/follower assignments.
The RDK runtime lives in `~/station`; the client lives in `~/teleop`.

## Start from the laptop

```bash
bash station/remote-rdk.sh
```

Enter the RDK SSH password. Leave this terminal open for browser access. The launcher starts the managed
`rdk-station.service` user service on the RDK, then keeps an SSH tunnel open.
The service is installed but not enabled for automatic boot startup. Station runs with
its TCP/web services bound to loopback; SSH forwards them to alternate laptop ports
so the existing laptop Station on 8888/8889 can keep running:

- RDK web UI: http://127.0.0.1:18889
- RDK TCP client endpoint: `127.0.0.1:18888`

Read the live motor list from another laptop terminal:

```bash
python3 teleop/teleop.py --host 127.0.0.1:18888 status
```

This `status` command reads motor state; it does not enable torque or move joints.
Do not confuse this RDK station with http://localhost:8889, which belongs to the
laptop's existing Station process.

Alternatively, SSH to the RDK and run:

```bash
systemctl --user start rdk-station.service
```

Then in another RDK terminal:

```bash
python3 ~/teleop/teleop.py status
```

## RDK configuration and storage

`rdk.yaml` enables ST3215 and system monitoring. USB video, VESC, and inference
are disabled for this arm setup. ROS can continue owning the camera devices.
The root filesystem has been expanded to 57 GB, with roughly 48 GB free.
The launcher uses 256 MB per-queue retention, a 128 MB memory-buffer budget, and
32 MB WAL rotation. These are per-component limits, not a hard cap on total
disk use or process RAM. Monitor `df -h /` and
`du -sh ~/station/station_data` during longer recordings. This configuration is
for arm control, not a full camera demonstration-data collection setup.

Always launch in `~/station`, as the launcher does: Station stores its private
key, motor calibration metadata, and history in the working directory's
`station_data`. Preserve that directory; do not commit or copy private keys to Git.
The laptop's previous calibration/role metadata remains on the laptop. A new bus
or new Station data directory may require calibration before motion/mirroring.

## Service management

On the RDK:

```bash
systemctl --user status rdk-station.service
journalctl --user -u rdk-station.service -n 30
systemctl --user stop rdk-station.service
```

Stop Station before starting a ROS rover controller on the same adapter. The
launcher uses error-level logging to avoid repetitive discovery warnings. It
does not automatically restart after a failure or enable mirroring.

## Arm and rover sharing a bus

The verified arm has **eight motors (IDs 1–8)** and uses Station's ElRobot model.
The extra servo is ID 10. Wheel IDs are **11 left, 12 back, 9 right**.
The left/back wheels were remapped from 7/8 to eliminate collisions with the arm.
Update rover controller mappings before driving; old code targeting 7/8 would
address arm joints instead. No controller mapping update is included here.

Adapter **5B8E114718** returned valid responses from all twelve motors together.
The original adapter **5B91029393** repeatedly returned corrupted responses;
do not confuse them merely because both enumerate as ttyACM0.
See [troubleshooting notes](../RDK-TROUBLESHOOTING.md) for the Ethernet workaround
and diagnostic history. Verify actual responses before enabling motion.

Do not start bus-wide auto-calibration or leader/follower mirroring while wheels
share the arm's adapter. Use a separate arm adapter for independent arm/base control,
or explicitly restrict commands to the verified arm joint IDs. Support the arm
before disabling torque; it can drop under gravity.

The existing `teleop.py start` is **leader-to-follower mirroring**, not a generic
single-arm "start" command. It enables torque and moves a follower to match the
leader. It requires two identified buses and suitable calibration. Do not run it
until those roles and joint mappings have been verified on the RDK setup.
For initial setup, use the web UI to inspect the arm and calibrate the appropriate
arm joints with only the arm connected to its adapter. Station was last verified
to report eight arm motors, torque off and mirroring off. No automatic arm movement is part
of installation or the `status` check.

Official instructions:
https://github.com/norma-core/norma-core/blob/main/software/station/bin/station/README.md
