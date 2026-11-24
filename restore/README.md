# Recréer le poste de contrôle sur un autre PC

Le dépôt contient maintenant les calibrations, poses, montage caméra, scènes,
maillages et CAO enregistrés sur le PC le 19 novembre 2026. L'archive
`restore/robot-data.tar.gz` et son manifeste SHA-256 sauvegardent 54 fichiers.
`restore/data.py` vérifie chaque fichier, adapte les chemins au nouveau clone
et refuse d'écraser une modification locale inconnue.

Ces données concernent **le même robot physique**. Les recopier sur un autre
ordinateur ne recalibre pas un autre robot. Cette procédure n'envoie aucune
commande au robot et ne lance pas automatiquement la station.

## Prérequis

Configuration enregistrée : Ubuntu 24.04.3 x86_64, Python 3.12.3, Docker
29.8.1. Pour cuRobo : NVIDIA RTX 3050 Ti Laptop, pilote 595.99.02,
PyTorch 2.14.1+cu130. Sur un autre PC, un GPU NVIDIA et un pilote compatible
CUDA sont nécessaires pour reproduire le moteur GPU ; le mode CPU reste disponible.
Windows, macOS et ARM ne sont pas couverts par les versions Python figées.

Prévoir une connexion Internet et environ 20 Go libres pour les environnements,
les compilations temporaires et l'image ROS. Installer Docker Engine et permettre
à l'utilisateur d'exécuter `docker info`. Avec Ubuntu 24.04 :

```bash
sudo apt-get update
sudo apt-get install git curl build-essential python3.12-dev python3.12-venv \
  libgl1 libglib2.0-0 x11-xserver-utils rsync openssh-client
```

RViz utilise l'affichage X11/XWayland. Pour le GPU, installer le pilote NVIDIA
adapté au matériel et vérifier `nvidia-smi`. Les clés SSH et mots de passe ne
sont pas inclus dans ce dépôt public.

## Restaurer depuis GitHub

```bash
mkdir -p ~/hack
git clone https://github.com/EliasP-MT/Sisyphus-.git ~/hack/humanoid-hackathon
cd ~/hack/humanoid-hackathon
bash restore/setup_pc.sh
# Option GPU, nécessaire pour retrouver l'accélération cuRobo :
bash arm_control/curobo/restore.sh
```

Le premier script restaure les données, le SDK, les environnements CPU et OCR,
les poids OCR exacts et l'image Docker. Il refuse de remplacer un environnement
Python existant : utiliser un clone neuf. En cas d'échec réseau, les éléments
déjà créés restent sur disque ; consulter la sortie avant toute nouvelle tentative.
Les étapes peuvent aussi être exécutées individuellement en lisant le script.

Pour restaurer uniquement les données, sans dépendances ni réseau :
`python3 restore/data.py`. Le script est idempotent. Il vérifie toute l'archive
et tous les conflits avant d'écrire. Il ne remplace que des fichiers absents,
identiques ou correspondant exactement à la version originale sauvegardée.

### Versions enregistrées

- SDK NormaCore : `6b494f77fac366a878ae71222458b196f8400dee`.
- Python CPU : `restore/requirements-arm.lock`.
- Python OCR : `restore/requirements-ocr.lock`.
- Poids PaddleOCR : dépôts PaddlePaddle/PP-OCRv5_mobile_det et
  PaddlePaddle/PP-OCRv5_mobile_rec, révisions et SHA-256 dans
  `restore/ocr-models.json`. Le téléchargement est vérifié et les fichiers vont
  dans `base_ocr/models.local/`. Le moteur OCR utilise ces fichiers en priorité.
  Les modèles et leurs notices restent disponibles auprès de PaddlePaddle.
- cuRobo : `71ce1726925adfdcc7d0d39dabec5f2d6bb71acd` de NVlabs/curobo ;
  dépendances dans `arm_control/curobo/requirements-linux-py312.lock`.
- Image ROS 2 Humble :
  `ghcr.io/epflxplore/base@sha256:3ee3faca2959b2eeb904babdabff40928d4e2d7d90afa00055e7a78a950b88b4`.

L'ancienne image locale du 30 septembre n'est plus téléchargeable sous son
identifiant enregistré. La restauration utilise donc la version Linux amd64
du 6 novembre de la même image ROS Humble, figée par son manifeste disponible.
Ce composant est une version de remplacement, pas une copie binaire de l'ancien
conteneur. L'image d'origine est conservée sur ce PC.

Le dépôt contient les données spécifiques au robot, mais ne constitue pas un
miroir hors ligne de Python, Docker, NVIDIA, du SDK ou des modèles OCR. Ces
composants sont téléchargés aux versions indiquées ; leur disponibilité dépend
de leurs hébergeurs.

## Portabilité des chemins et du modèle de collision

Le script remplace l'ancien chemin du clone dans les scènes JSON, le modèle
cuRobo YAML et son URDF. Les sphères et les paires d'exemption restent identiques.
Il vérifie d'abord le hash d'origine du modèle, puis adapte celui du fichier
`self_exempt.json` au YAML dont seuls les chemins ont changé.

**Il n'est donc pas nécessaire de régénérer ni de recalibrer le modèle simplement
parce que le nom d'utilisateur ou le dossier change.** Si la géométrie du robot
change, utiliser `arm_control/curobo/build_model.py` puis
`arm_control/curobo/calibrate_self.py` avec les nouvelles entrées et vérifier
physiquement les calibrations.

Le dossier cuRobo est créé par défaut dans `~/hack/curobo-src`. Pour un autre
emplacement : `CUROBO_ROOT=/chemin/neuf bash arm_control/curobo/restore.sh`,
puis définir `PLANNER_PY=/chemin/neuf/.venv/bin/python` au lancement.

## Démarrage manuel

Après connexion au réseau du robot, remplacer l'adresse par son IP actuelle :

```bash
ROBOT_WIFI_IP=172.20.10.3 bash ros2_robot/deploy/station.sh
# Ou sans fenêtre RViz :
ROBOT_WIFI_IP=172.20.10.3 bash ros2_robot/deploy/station.sh rviz:=false
```

L'interface est accessible sur http://127.0.0.1:8895/. Le domaine ROS est 43
par défaut ; le robot et le PC doivent utiliser le même domaine. Le script génère
le profil réseau à partir de l'interface qui rejoint le robot. Une adresse IP,
une interface Wi-Fi ou un accès SSH ne se restaurent pas à partir de l'ancien PC.

Cette sauvegarde porte sur le poste PC et ses données. L'état actuel du RDK n'a
pas été interrogé ni modifié : s'il contient des poses ou calibrations plus
récentes, elles ne figurent pas dans ce snapshot PC. Pour réinstaller le logiciel
du robot, suivre `ros2_robot/README.md` et `docs/ros2-runtime.md` ; le script
de déploiement préserve par défaut ses calibrations existantes.

## Sauvegarde privée et fichiers exclus de GitHub

Une archive locale complète du dossier `hack` a été créée séparément sur le PC,
y compris l'ancienne base Station et ses secrets. Elle doit rester privée.
Le dépôt public ne contient ni `station/station_data/`, ni sa clé de chiffrement,
ni le jeton `ros2_robot/.station_ws/token`, ni les journaux et captures caméra
historiques. Ces fichiers ne sont pas requis pour reconstruire le poste ROS actuel.
Les environnements virtuels, caches et résultats de compilation sont reconstruits.

Aucun fichier du dossier `hack` n'a été supprimé pendant cette sauvegarde.

## Vérifications

La restauration cuRobo a été validée le 19 novembre 2026 avec une installation
neuve : 101 paquets compatibles, opération CUDA, initialisation du Planner et
calcul de collision GPU, y compris compilation des kernels dans un cache vide.
Les vérifications spécifiques à cette sauvegarde portable sont consignées dans
`restore/VALIDATION.md`. Il ne s'agit pas d'un essai physique sur un autre PC
ou d'une validation de mouvement du robot.
