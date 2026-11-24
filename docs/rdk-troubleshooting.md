# RDK troubleshooting notes

## Full rover CAD from Martin USB

`/media/evan/MARTIN_USB/full_robot_box_complete.step` (163 MB), SHA256
`7248490e2f068d324c7f5b1c99cc8f8e00b4879e71c399fcebae9aa60097bc0d`.
OpenCascade transferred one root: 9 solids, 69,949 shells, 70,136 faces.
Full assembly bounds in metres: [-0.202021, -0.164107, -0.092932]
to [0.142479, 0.382601, 0.256227]. The solid-only converter omitted most
of this assembly and returned the same nine box solids as the older STEP.
Do not approve that incomplete manifest for rover collision checking.
`cad.py` now refuses surface-shell assemblies until a surface-aware importer
is implemented. Generated partial geometry is ignored, not deployment-ready.

Surface-aware inspection import is now implemented in `import_surface_cad.py`.
It exported all 70,136 faces (23 verified zero-area faces skipped), 109,954
triangles, preserving assembly coordinates in metres. Full assembly is visible
in the calibration UI under "Inspect full CAD assembly". Its separate
`surface-assembly-preview-v1` format intentionally cannot be mistaken for an
articulated/validated collision manifest. Moving arm part bindings and the
CAD-to-URDF registration still need to be established before deployment.

## Current ID assignment — supersedes older notes below

Arm alone was verified to contain IDs 1–8 (not just 1–6), explaining
collisions with wheel IDs 7 and 8. User requested remapping the wheels.
With only wheels connected, targeted EEPROM writes successfully changed
left wheel 7 -> 11 and back wheel 8 -> 12. Right wheel remains 9.
All three verified model 777, velocity mode 1, torque 0; EEPROM relocked.
Extra servo remains ID 10. Arm IDs remain 1–8.
IMPORTANT: existing drive software may still target old wheel IDs 7/8/9;
update motor mappings before enabling any controller with the arm attached.

## Latest verified working state

Using adapter 5B8E114718 on the RDK restored valid communications.
With the arm disconnected, the extra servo was successfully changed from
ID 1 to ID 10 by targeted writes, verified model 777, ID register 10,
EEPROM lock 1, torque off and unchanged position 4092. Wheel IDs 7/8/9
still returned model 777; old ID 1 no longer responded. Arm can now be
reconnected with power off, followed by read-only verification of IDs 1–10.

## Direct Ethernet after power cycles

The laptop sometimes selects `Connexion filaire 1` instead of `rdk-direct`,
losing its IPv4 addresses. Even with `rdk-direct` selected, a complete
deactivation/reactivation restored connectivity when simple activation did not.

```bash
nmcli connection down rdk-direct
nmcli connection up rdk-direct
ip route get 192.168.127.10
ping -I enp3s0 -c 3 192.168.127.10
```

Laptop: 192.168.127.1/24 and 192.168.1.1/24 on enp3s0.
RDK: 192.168.127.10 (Ethernet), 172.20.10.3 (Wi-Fi).
`rdk-direct` autoconnect is enabled with priority 100.
Check this before concluding that the RDK failed to boot.

## Servo recovery in progress

Adapter: QinHeng 1a86:55d3, serial 5B91029393, normally ttyACM0.
Wheel IDs: 7, 8, 9. Extra STS3215 servo was changed from ID 1 to 10;
valid model/position reads were verified immediately after that change.
Later reads returned invalid bytes on both RDK and laptop, including
8080ff00 for ID 10. Station/ROS were stopped; no competing port owner found.
USB reset, CDC ACM rebind, serial parameter changes and a temporary
ModemManager stop did not restore communication.

User subsequently isolated the extra servo (arm and wheels disconnected).
A broadcast EEPROM unlock / ID=1 / relock sequence was sent at 1 Mbaud.
Neither ID 1 nor 10 returned a valid response afterward; ID change unconfirmed.
Never broadcast ID changes with the wheel motors connected.
Do not assume the fault is hardware or software without further evidence.

Recovery ID=1 broadcasts were subsequently tried at 1000000, 500000,
250000, 115200, 57600, 38400, 19200 and 9600 baud, still without a valid
model response. Baud-rate register was not changed. No motion was commanded.
