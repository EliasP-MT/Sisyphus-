cmake_minimum_required(VERSION 3.8)
project(postal_robot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Heartbeat.msg msg/MotorState.msg msg/MotorStateArray.msg
  msg/SupervisorState.msg msg/PoseInfo.msg
  srv/Acknowledge.srv srv/SetTorque.srv srv/RunDispenser.srv srv/CapturePose.srv
  srv/ListPoses.srv srv/DeletePose.srv srv/SetGripper.srv srv/PlanToPose.srv
  action/ExecuteTrajectory.action action/GoToPose.action
  DEPENDENCIES builtin_interfaces std_msgs trajectory_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
