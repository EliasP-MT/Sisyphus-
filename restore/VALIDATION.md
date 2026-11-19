# Portable snapshot validation — 2026-11-19

- Exported the staged repository into a separate directory with a different path.
- Restored all 54 snapshot files and verified SHA-256 hashes before relocation.
- Ran restoration again: zero files changed (idempotence).
- Introduced a conflicting calibration in a separate fixture: restoration refused
  it before writing anything and preserved the local file unchanged.
- Verified that the relocated cuRobo YAML differs only in its root path, its
  exemption checksum matches, and every URDF and active scene mesh exists.
- Fetched the pinned NormaCore SDK revision from GitHub into the test checkout.
- Initialized the CPU Planner using the test checkout's saved calibration/scene
  and executed a collision query successfully.
- Downloaded both OCR models from their pinned Hugging Face revisions; all ten
  file checksums matched. The relocated OCR module loaded those models and read
  a synthetic image containing `TEST` successfully.
- Built all three ROS packages in the pinned Humble Docker image (no station
  server, robot connection or motor commands).
- Shell syntax and Git whitespace checks pass. Existing OCR environment passes
  `pip check`; CPU/OCR package snapshots match the installed environments.

CPU/OCR runtime checks reused the existing Python environments as interpreters;
they did not reinstall those packages or simulate a fresh operating system.
The cuRobo environment was separately rebuilt from scratch and tested earlier
the same day, including fresh CUDA kernel compilation (see docs/RESTORE_PC.md).
No physical second PC or hardware motion was tested. Robot-side state was not
queried, so the snapshot represents the PC's saved calibration/poses.
