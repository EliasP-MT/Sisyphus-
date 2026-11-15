# Migration ROS 2 du robot postal

Ce document décrit l'architecture livrée, son exploitation (RDK et PC), le
contrat de sécurité et l'état réel de validation. Les anciens outils basés sur
Station (Station, `arm-view`, `teleop`, `calibration_ui`, `hand_control`,
exécuteur et commissionnement d'`arm_control`) ont été supprimés après
validation, à la demande de l'utilisateur ; ils restent dans l'historique Git
(dernier commit les contenant : `934b2b0`).

## 1. Architecture

```
PC (Ubuntu 24.04)                                RDK X5 (Ubuntu 22.04, aarch64)
┌──────────────────────────────────────┐  Wi-Fi  ┌────────────────────────────────────────┐
│ conteneur ROS 2 Humble (--net=host)  │  DDS    │ postal-robot.service (systemd user)    │
│  /station/web        UI 127.0.0.1    │◄───────►│  /robot/core   SEUL propriétaire du bus│
│  /station/arm_motion plan + exécution│ domaine │    boucle 20 Hz : lecture 12 servos,   │
│  RSP aperçu (préfixe preview/), RViz │   43    │    superviseur, base, distributeur,    │
│                                      │         │    trajectoires bras, pince, poses     │
│ venv arm_control (hors conteneur)    │         │  /robot/robot_state_publisher (TF)     │
│  planner_worker 127.0.0.1:8893       │         │  /robot/camera_{1,2}/camera (JPEG)     │
│  = planner.Arm + collision.World     │         │                                        │
└──────────────────────────────────────┘         └────────────────────────────────────────┘
```

* **Un seul propriétaire du bus série** (adaptateur `5B8E114718`) : le processus
  `core`. Toutes les entrées-sorties série passent par un unique fil de
  contrôle ; les services ROS lui soumettent leurs écritures. L'unité systemd
  déclare `Conflicts=rdk-station.service arm-view.service`, et le port est ouvert
  en mode exclusif.
* **La planification reste sur le PC**, car pybullet, OMPL et ruckig ne sont
  disponibles que dans le venv validé (pas de wheel aarch64 pour ruckig,
  pybullet à compiler, réseau du partage de connexion ≈ 30 kB/s). Le code
  `arm_control/planner.py` et `collision.py` est réutilisé **sans modification**.
  Le robot exécute et surveille seul les trajectoires reçues.
* **Matériel et simulation sont séparés** : le matériel est sur
  `ROS_DOMAIN_ID=43`, espace `/robot`. Le jumeau MuJoCo (`digital_twin`) publie
  sous `/postal_sim` et doit tourner dans un autre domaine ; ses positions ne
  sont jamais du retour matériel. Le double de test `device:=sim://` (servos
  simulés au niveau octet, tests uniquement) est signalé dans chaque état
  publié.
* **Navigation et FSM futures** : la base consomme `geometry_msgs/Twist` sur
  `/robot/base/cmd_vel` et publie l'odométrie roues sur `/robot/base/odom`.
  Le TF `odom` n'est pas diffusé par défaut (`publish_odom_tf: false`) : le futur
  merge navigation décidera de son propriétaire. Une FSM pourra appeler
  `/station/arm/plan_to_pose` et `/station/arm/go_to_pose`, ainsi que les
  services du robot. Aucun scénario autonome n'est implémenté.

## 2. Interfaces (`postal_robot_interfaces`)

| Nom | Type | Rôle |
| --- | --- | --- |
| `/station/heartbeat` | `msg/Heartbeat` (best effort) | bail de la station, 10 Hz |
| `/robot/supervisor/state` | `msg/SupervisorState` | état, défauts, avertissements, couples |
| `/robot/motors` | `msg/MotorStateArray` | 12 servos bruts : ticks, vitesse, charge, courant, V, °C, statut 0x41, fraîcheur |
| `/robot/joint_states` | `sensor_msgs/JointState` | 7 articulations calibrées, mesures fraîches du cycle uniquement |
| `/robot/base/cmd_vel` | `geometry_msgs/Twist` (best effort) | vx, vy (m/s), wz (rad/s) ; homme mort de 0,3 s |
| `/robot/base/odom` | `nav_msgs/Odometry` | odométrie roues (frames `odom` → `base_footprint`) |
| `/robot/camera_{1,2}/image/compressed` | `sensor_msgs/CompressedImage` | JPEG 640×480, 5 Hz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | moteurs, superviseur, caméras (1 Hz) |
| `/robot/supervisor/acknowledge` | `srv/Acknowledge` | reprise explicite |
| `/robot/set_torque` | `srv/SetTorque` | `arm` / `base` / `dispenser`, explicite |
| `/robot/dispenser/run`, `/stop` | `srv/RunDispenser` | durée obligatoire, ≤ 10 s, −1000 par défaut |
| `/robot/gripper/set` | `srv/SetGripper` | ticks bruts dans 1268–2828 |
| `/robot/poses/list`, `capture`, `delete` | `srv/ListPoses`… | bibliothèque de poses du robot |
| `/robot/arm/execute_trajectory` | `action/ExecuteTrajectory` | exécution sur le robot |
| `/station/arm/plan_to_pose` | `srv/PlanToPose` | plan à usage unique (120 s), raisons de refus |
| `/station/arm/go_to_pose` | `action/GoToPose` | exécution d'un plan, pince en option |

## 3. Contrat de sécurité

### Bail de station et reprise

* Le bail n'existe que si un **onglet de l'interface web est ouvert** et envoie
  ses pings (navigateur → nœud web toutes les 200 ms, nœud web → robot à
  10 Hz). Fermer l'onglet, arrêter la station ou couper le Wi-Fi suspend le bail.
* Le robot juge la fraîcheur à **l'heure d'arrivée** (les horloges PC et robot
  ne sont jamais comparées) et ignore les heartbeats rejoués (numéro de séquence).
* États : `WAITING_FOR_STATION` → `AWAITING_ACK` → `ACTIVE` ;
  `ACTIVE` → `LINK_LOST` après 0,8 s sans bail ; `FAULT` pour les défauts latchés.
* **Arrêt sur perte de bail, de session ou défaut** :
  * roues : vitesse 0, couple conservé (freinage), arrêt confirmé par la vitesse
    mesurée sous 1,5 s, sinon défaut ;
  * distributeur : vitesse 0 ;
  * bras : la trajectoire est interrompue et plus aucune consigne n'est envoyée.
    Le servo termine la dernière consigne, qui est un point du chemin validé,
    au plus une période devant lui et dans la limite de l'erreur de suivi. Le
    couple est conservé. Une pince en mouvement est arrêtée sur place.
* **Reprise** : le retour du bail conduit à `AWAITING_ACK`, jamais
  automatiquement à `ACTIVE`. L'acquittement exige un bail frais de la même
  session et l'absence de défaut actif sur un groupe sous couple. Seules les
  commandes `cmd_vel` arrivées **après** l'acquittement sont exécutées ; une
  trajectoire interrompue n'est jamais reprise (nouveau plan obligatoire).
* Un redémarrage de la station crée une nouvelle session, qui doit être acquittée.
* Une simple perte de commandes n'arrête pas physiquement un servo. Ce sont les
  écritures de vitesse nulle du robot qui arrêtent la base, que la station
  soit joignable ou non.

### Écritures moteurs

* Écritures limitées aux registres RAM couple (0x28), accélération (0x29),
  consigne de position (0x2A) et vitesse (0x2E), par ID explicite. Aucun
  broadcast, aucune écriture EEPROM (ID, mode, limites, offsets, PID).
* **Une écriture de consigne active le couple d'un ST3215** (constaté sur le
  matériel). En conséquence, aucun zéro n'est écrit vers un servo dont le
  retour frais indique couple coupé. Lors de l'activation du bras, la consigne
  = position mesurée est écrite avant la vitesse.
* Les modes réels sont lus au démarrage (0x21 : bras/pince 0, roues et
  distributeur 1). Un groupe dont le mode diffère ne peut pas être activé.

### Mesures

* Chaque paquet est validé (en-tête, ID, longueur, somme de contrôle). Le
  statut du paquet et le statut servo (0x41) sont remontés comme défaut, jamais
  masqués. Aucune valeur absente n'est remplacée par zéro ou par l'ancienne
  valeur : `fresh=false` au-delà de 0,3 s.
* Un dépassement isolé du délai d'écriture série est une erreur de transaction
  pour ce moteur, pas une perte de bus.

### Bras

* Exécution d'un plan uniquement s'il est marqué `collision_checked` avec
  l'empreinte de scène. Tous les points sont convertis et vérifiés avant toute
  écriture (refus, pas d'écrêtage). Le départ mesuré doit correspondre au plan
  à 0,02 rad près et le bras être immobile. Chaque cycle exige des mesures
  fraîches ; une erreur de suivi > 0,1 rad ou une boucle en retard > 0,25 s
  arrête l'exécution. Une seule trajectoire à la fois.
* Côté station : la pose de départ mesurée ne doit pas avoir bougé de plus de
  6 ticks depuis le calcul ; chaque plan est à usage unique et expire après
  120 s. Pendant l'exécution, la pose mesurée est revérifiée en collision à
  4 Hz et l'objectif robot est annulé en cas de violation.
* **Vitesse par plan** : `PlanToPose.speed_factor` (0,1 à 2,0 ; champ
  « Speed » de l'interface) met à l'échelle la vitesse, l'accélération et le
  jerk calibrés du profil Ruckig. Le chemin, et donc le contrôle de collision,
  ne change pas. Le robot refuse toute trajectoire plus rapide que
  `max_speed_factor` × la vitesse calibrée (2,0 par défaut) et règle le
  registre de vitesse de chaque servo sur la vitesse de crête du plan, avec
  25 % de marge, entre la vitesse calibrée et ce plafond. Exemple
  `scan → vertical` : 43,5 s à 0,5×, 22,1 s à 1×, 11,5 s à 2×.
* **Suivi à vitesse élevée** : la limite d'erreur de suivi (0,1 rad à vitesse
  calibrée) est proportionnelle à la vitesse du plan (1,5× → 0,15 rad,
  2× → 0,2 rad), et l'affaissement statique mesuré au départ, par articulation
  (au plus la tolérance de départ de 0,02 rad), est retiré de l'erreur : la
  limite ne contrôle que le retard dû à la vitesse. L'accélération des servos
  augmente avec le carré du rapport de vitesse. Contrepartie : à 2× le bras
  peut s'écarter jusqu'à 0,2 rad du chemin vérifié ; la station contrôle en
  plus la collision de la pose mesurée à 4 Hz.
* **Compensation bornée de l'erreur statique** (choix de l'opérateur) : sous
  gravité, J3/J4 restent 17 à 26 ticks sous la consigne (≈ 10 mm au bout de
  l'outil). Une fois le bras immobile, la consigne finale est décalée de l'erreur
  mesurée, au plus de 0,06 rad sur J2–J4, en 5 itérations au plus. La boîte
  ±0,06 rad autour de la pose cible est validée en collision et en calibration
  par le planificateur (grille 3×3×3 et mouvements depuis la cible) ; sinon,
  pas de compensation. L'arrivée exige une erreur ≤ 0,01 rad par articulation.

## 4. Données locales (hors Git)

| Donnée | PC | RDK |
| --- | --- | --- |
| Calibration vérifiée | `arm_control/calibration-candidate-20261115-105050-197423.local.json` | `~/postal_config/calibration.local.json` |
| Poses (format `encoder-poses-v1`) | `arm_control/poses.local.json` | `~/postal_config/poses.local.json` (importé une fois, puis source de vérité) |
| Scène de collision | `arm_control/assets/rover-fixed.local/rover-scene.local.json` | — |
| URDF + maillages | `arm_control/.sdk/hardware/elrobot/simulation/` | `~/postal_config/elrobot.urdf` |

Le déploiement ne remplace jamais la calibration ni les poses du robot
(`--replace-calibration` sauvegarde l'ancienne calibration). Chaque
remplacement ou suppression de pose crée `poses-backup-<horodatage>.local.json`.
L'écriture est atomique (fichier temporaire, fsync, renommage).

## 5. Exploitation

### RDK (une fois, puis à chaque mise à jour) — depuis le PC

```bash
RDK_HOST=sunrise@172.20.10.3 bash ros2_robot/deploy/deploy_robot.sh --restart
# démarrage automatique au boot (systemd utilisateur, lingering déjà actif) :
RDK_HOST=sunrise@172.20.10.3 bash ros2_robot/deploy/deploy_robot.sh --enable-boot
```

Dépendances RDK (déjà installées) :
`sudo apt-get install ros-humble-control-msgs python3-colcon-common-extensions ros-humble-xacro`,
plus `python3-serial` et `python3-opencv`, présents. Le premier build des
interfaces prend environ 9 minutes (stockage lent).

Sur le RDK :

```bash
systemctl --user status postal-robot.service
journalctl --user -u postal-robot -f
systemctl --user stop postal-robot.service     # avant d'utiliser Station / arm-view
```

### PC

Prérequis : Docker, une image ROS 2 Humble desktop avec colcon (par défaut
`ghcr.io/epflxplore/base:humble-desktop`, sinon `STATION_IMAGE=osrf/ros:humble-desktop`),
et le venv `arm_control/.venv` (`arm_control/setup.sh`).

```bash
ROBOT_WIFI_IP=172.20.10.3 bash ros2_robot/deploy/station.sh            # + RViz
ROBOT_WIFI_IP=172.20.10.3 bash ros2_robot/deploy/station.sh rviz:=false
```

Puis ouvrir http://127.0.0.1:8895/ et cliquer sur « Acknowledge & resume ».
Le script restreint Fast DDS à l'interface qui route vers l'IP Wi-Fi du robot.
Côté robot, `robot_dds_profile.sh` (ExecStartPre du service) limite le DDS à
l'adresse Wi-Fi courante, à la boucle locale et à la mémoire partagée. Le lien
opérationnel reste ainsi le Wi-Fi, même avec le câble Ethernet de service
branché (SSH par Ethernet restant possible). L'API n'écoute que sur
127.0.0.1, exige le jeton de la page et un en-tête Host local.

Utilisation type : activer le couple du bras (maintien sur place), choisir une
pose, « Plan », vérifier le fantôme dans RViz et les raisons éventuelles, puis
« Execute ». Pour capturer une pose : bras immobile, puis « Capture current »
(trois nouvelles mesures stables à 3 ticks près, 8 moteurs frais, sans
erreur). Pour la base : activer le couple, puis maintenir W/A/S/D/Q/E. Pour
le distributeur : activer le couple, saisir une durée (obligatoire), puis « Run ».

### Dépannage

* Pas d'état robot : vérifier le Wi-Fi, `ROS_DOMAIN_ID=43`, le service. Si
  l'Ethernet ne répond plus, appliquer d'abord `RDK-TROUBLESHOOTING.md`
  (`nmcli connection down/up rdk-direct`).
* Planification refusée « outside the operating margin » : la position mesurée
  est hors des marges de calibration (par exemple au repos J2 = 1010 < 1012).
  Couper le couple, repositionner à la main, réessayer. Aucun écrêtage.
* « Arrivée en collision » : la pose enregistrée viole la marge de la scène.
  C'est le cas de `horizontal`, en collision avec `etage2_box_elias` à
  −0,7 mm : la recapturer.

## 6. Tests

```bash
# unitaires (sans ROS) : 47 tests
cd ros2_robot && PYTHONPATH=postal_robot:postal_robot/tests python3 -m unittest discover -s postal_robot/tests
# intégration ROS sur bus simulé (domaine isolé) : 10 tests
ROS_DOMAIN_ID=77 bash ros2_robot/deploy/station_docker.sh python3 /postal/src/postal_robot/tests_ros/test_core_sim.py
# bibliothèques arm_control (planificateur, collisions, CAO) : 21 tests
cd arm_control && .venv/bin/python -m unittest test_arm test_collision test_simplify_collision test_surface_cad test_rover_registration
```

## 7. Validation matérielle (2026-11-15, opérateur présent)

| Essai | Résultat mesuré |
| --- | --- |
| Lecture des 12 servos, 60 s | 1198 cycles, **0 réponse manquée, 0 mesure périmée**, période 50 ms (p99 50 ms), cœur ≈ 29 % d'un CPU |
| Station via Wi-Fi | état à 20 Hz, 2 caméras à 5 Hz, interface affichée (caméras, moteurs, poses, état, collision) confirmée par l'opérateur |
| Couple bras « maintien sur place » | positions inchangées au tick près |
| Pince | 1294 → 1447 ticks (consigne 1450) |
| Capture / suppression de pose | sauvegarde atomique ; suppression avec `poses-backup-*.local.json` |
| Base au sol | rotation anti-horaire et avance (à l'opposé de la roue 12) confirmées ; vitesses roues conformes à la cinématique ; arrêt confirmé |
| Distributeur −1000 pendant 1 s | ≈ ¼ de tour, sens de distribution confirmé, arrêt automatique |
| Poses `scan` → `vertical` → `scan` | trajectoires de 10 à 22 s sans défaut ; compensation de 0,025 rad ; arrivées ≤ 0,0092 rad (sans compensation : 17 à 26 ticks d'affaissement, ≈ 10 mm) |
| Arrêt de la station | `LINK_LOST` après 0,80 s, bras maintenu sous couple |
| Coupure du Wi-Fi robot (base en rotation + distributeur) | roues à 0 en 0,8 s (homme mort), `LINK_LOST` à 0,8 s, distributeur arrêté en 0,4 s, bras inchangé à ±2 ticks ; au retour, `AWAITING_ACK` et 12 s de commandes de rotation **ignorées** jusqu'à l'acquittement |
| Redémarrage du RDK | service actif au boot, bus et caméras ouverts, `AWAITING_ACK`, Station inactive |

Défauts découverts pendant ces essais, puis corrigés (commit `c442aa9`) :
activation implicite du couple des roues par l'écriture d'une vitesse nulle ;
affaissement cumulatif du bras à chaque maintien ; perte de bus déclarée sur un
simple dépassement d'écriture série ; bail transporté par le câble Ethernet
lors du premier essai de coupure (corrigé par le profil DDS du robot) ;
famine du moniteur de collision de la station.

## 8. Limites et risques restants

* **La planification dépend du PC.** Sans station, le robot ne peut pas
  rejoindre une pose : il faudra porter pybullet/OMPL/ruckig sur aarch64 ou
  embarquer des plans pré-validés pour la future FSM.
* Poses : `horizontal` est en collision dans la scène (−0,7 mm sous la marge)
  et doit être recapturée. La position de repos est hors des marges de
  calibration.
* Réseau : la redécouverte DDS prend ≈ 20 s après le retour du Wi-Fi.
  L'adresse Wi-Fi est figée au démarrage du service : en cas de changement
  DHCP, redémarrer `postal-robot`. Le pare-feu système (option plus robuste
  que le profil DDS) n'a pas été installé faute d'autorisation. Sans Wi-Fi, le
  robot peut encore envoyer son état au PC par la route par défaut `eth0`, ce
  qui est sans effet sur la sécurité.
* Horloge : le RDK n'a pas de RTC. Au boot, l'heure saute de « juin 2025 » à
  l'heure NTP, et les horodatages inter-machines sont faux jusqu'à la
  synchronisation. La logique de sécurité n'en dépend pas, mais la navigation
  en dépendra (prévoir chrony et une attente de synchronisation).
* L'odométrie roues n'est pas validée quantitativement (4096 pas/tour
  supposés ; seuls les sens sont vérifiés). L'accélération des roues n'est
  pas réglée.
* La compensation a été essayée sur deux poses, J2–J4 uniquement. La scène
  modélise la pince à une ouverture fixe (0 rad) : un mouvement de pince
  n'est pas vérifié en collision.
* Pince en ticks bruts (plage Station 1268–2828), sans calibration angulaire.
  Caméras sans intrinsèques ni extrinsèques ; pas de TF caméra ni
  `base_footprint → base_link` (montage du bras non mesuré).
* L'interface web n'a pas de comptes utilisateurs : sa protection repose sur
  l'écoute en loopback et le jeton de page. Ne pas la publier sur le réseau.
* Le double de test `sim://` modélise des servos idéaux : il ne remplace pas
  les essais matériels.
* Anciens outils supprimés : il n'existe plus d'outil de calibration des zéros
  et des signes du bras. Pour recalibrer, restaurer `arm_control/calibration_ui.py`
  depuis l'historique (il dépend de Station) ou écrire un outil ROS.
