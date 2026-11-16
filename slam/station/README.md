# Station integration boundary

Station and motor control are supplied by main's
[ROS robot/station stack](../../ros2_robot/README.md) and
[arm tools](../../arm_control/README.md). SLAM must not open a camera or motor bus
already owned by those services. No Station executable, SDK or encryption state
is included in this subproject or its deployment bundle.

`mailbox_slam/station.py` is an optional read-only decoder bridge for already
supplied ST3215 frames. A live adapter still needs aligned capture/encoder
stamps and measured motor/bus mappings. The standalone camera and manual base
scripts are diagnostic alternatives, not concurrent hardware owners.
