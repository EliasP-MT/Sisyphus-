# Pont ROS 2 de simulation

ROS 2 Humble de base est présent sur cette machine. Le pont a été testé sur
localhost dans le domaine 42 : publication de scans/odométrie et réception d’un
`Twist.linear.y` non nul. Nav2 et SLAM Toolbox ne sont pas installés ici : leurs
profils sont fournis, mais leur exécution complète n’a pas été validée.

Depuis la racine du worktree :

```bash
bash digital_twin/run_ros2.sh
```

Ce script utilise `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=42`, un répertoire de
logs local et les topics de commande `/postal_sim/*`. Il n’importe pas le client
Station. Pour choisir un autre domaine de simulation, utiliser
`POSTAL_TWIN_ROS_DOMAIN`. Ne pas le partager avec un déploiement matériel.

| Topic | Type | Rôle |
|---|---|---|
| `/clock` | rosgraph_msgs/Clock | Temps physique simulé |
| `/postal_sim/scan` | sensor_msgs/LaserScan | LiDAR 2D virtuel, QoS sensor data |
| `/postal_sim/odom` | nav_msgs/Odometry | Odométrie de roues, avec vitesse latérale |
| `/postal_sim/imu` | sensor_msgs/Imu | Gyroscope ; orientation/accélération non publiées |
| `/postal_sim/joint_states` | sensor_msgs/JointState | Articulations, roues et rouleaux |
| `/postal_sim/tf` | tf2_msgs/TFMessage | odom → base_link → laser |
| `/postal_sim/cmd_vel` | geometry_msgs/Twist | Consigne SI vx/vy/lacet |
| `/postal_sim/arm_target` | sensor_msgs/JointState | Sept positions bras et pince optionnelle, radians |
| `/postal_sim/task_command` | std_msgs/String | JSON `grasp` avec `letter`, ou `release` |
| `/postal_sim/task_feedback` | std_msgs/String | Résultat logique de prise/dépôt |

Les frames sont préfixées : `postal_sim/odom`, `postal_sim/base_link`,
`postal_sim/laser`. Le SLAM externe publie `postal_sim/map → postal_sim/odom`.
Aucune TF de vérité terrain n’est fournie au SLAM. Une commande de base
absente depuis 0.5 s de simulation est remplacée par zéro.

Exemple dans un autre terminal du même domaine localhost :

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 topic pub -r 10 /postal_sim/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.08}, angular: {z: 0.0}}'
```

Pour l’intégration standard, installer sur votre environnement ROS les paquets
`ros-humble-navigation2`, `ros-humble-nav2-bringup` et `ros-humble-slam-toolbox`,
puis lancer avec le pont déjà actif :

```bash
ros2 launch digital_twin/ros/twin_navigation.launch.py
```

Le profil DWB autorise **vx et vy positifs/négatifs**, avec accélérations
latérales non nulles. Toutes les couches utilisent l’heure simulée, les frames
préfixées et les topics du pont. Les coordonnées initiales de carte de SLAM
Toolbox dépendent de son initialisation : convertir les objectifs sémantiques
du scénario dans sa frame de carte avant d’envoyer des objectifs Nav2.

Références officielles : [DWB holonomique](https://docs.nav2.org/configuration/packages/dwb-params/kinematic.html),
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/humble),
[MuJoCo Python](https://mujoco.readthedocs.io/en/3.3.7/python.html).
