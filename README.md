# Sisyphus

> **Built in 48 hours at the 2026 Leukerbad Robotics Hackathon**
>
> - **1st place in the Autonomous Urban Services track**
> - **1st place overall in the Grand Final**

Sisyphus is a low-cost, modular mobile robot for repetitive indoor delivery. Its first use case, **Leukerbad Postman**, autonomously sorted and delivered a stack of letters.

**Hackathon demo** · 43 seconds, accelerated footage

https://github.com/user-attachments/assets/c52f17e7-a493-4970-ab95-ac7294413782

[Watch on Google Drive](https://drive.google.com/file/d/1YhX-ApkGoVzBIu0LkoBMFkHpJ2p-o1RC/view?usp=sharing)

*Project status — Hackathon prototype demonstrated in 2026.*

## From stack to mailbox

The [delivery FSM](ros2_robot/postal_station/postal_station/tasks.py) surveys the mailbox labels before picking up the first letter. It reads each recipient’s name and checks it against the survey. The base approaches by a configured distance using wheel odometry, then the arm camera locates the matching slot. The arm inserts the letter and retreats before the next cycle.

~~~mermaid
flowchart LR
    Survey["Survey mailbox labels"] --> Stack["Feed and pick a letter"]
    Stack --> OCR["Read recipient name"]
    OCR --> Match["Match name to mailbox"]
    Match --> Drive["Approach using wheel odometry"]
    Drive --> Locate["Locate the matching slot"]
    Locate --> Deliver["Insert letter and retreat"]
    Deliver -->|More letters| Stack
~~~

The robot combines a Kiwi drive base, a robotic arm, cameras, and a servo-driven envelope dispenser. The modular design also explored interchangeable tools and payloads for tasks like delivering medicine or trays.

![Sisyphus mobile manipulator, showing the arm, dispenser, and drive base](docs/images/robot.jpg)

## Repository architecture

The delivery FSM runs on the control PC. It calls the ROS 2 robot runtime on the RDK X5 for base motion, dispensing, and arm execution. Two local station workers handle vision and arm planning.

| Folder | Role |
| --- | --- |
| [`ros2_robot/`](ros2_robot/README.md) | ROS 2 Humble runtime for the RDK X5 and PC-based control station with RViz, a web interface, and arm planning. |
| [`base_ocr/`](base_ocr/README.md) | Base-camera capture and name matching using PP-OCRv5 Mobile. |
| [`mailbox_vision/`](mailbox_vision/) | Label detection, recipient matching, and slot pose estimation from arm-camera images. |
| [`arm_control/`](arm_control/README.md) | Inverse kinematics, trajectory timing, and collision checks using robot CAD. |
| [`restore/`](restore/README.md) | Archived calibration, taught poses, model downloads, and pinned dependencies from the control PC. |
| [`docs/`](docs/) | [ROS 2 runtime architecture](docs/ros2-runtime.md), [RDK troubleshooting](docs/rdk-troubleshooting.md), and project photos. |

## Team

Evan Ayoub, Balthazar Bolze, Jérôme Courdacy, Vincent Lellu, Martin Mayer, and Elias Pajot built Sisyphus at the Leukerbad Robotics Hackathon.

![The Sisyphus team at the Leukerbad Robotics Hackathon](docs/images/team.jpg)

Thank you to the Edelweiss Innovation & Intelligence Institute (EIII), Pedro Jinjun Dong, and EuroTech Federation for organizing the event.
