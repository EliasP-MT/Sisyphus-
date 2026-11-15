# humanoid-hackathon

## RDK arm control

See [Station on RDK X5](station/RDK.md) for installation, browser access, and the
existing teleoperation client. Start from the laptop with:

```bash
bash station/remote-rdk.sh
```

The RDK Station UI is at http://127.0.0.1:18889; the laptop Station UI remains on
8889. Run only one controller per physical servo adapter.

The arm uses IDs **1–8**, the extra servo **10**, and wheels **11 left / 12 back /
9 right**. Older rover code must be updated before driving with this mapping.
See [hardware and Ethernet troubleshooting](RDK-TROUBLESHOOTING.md).
