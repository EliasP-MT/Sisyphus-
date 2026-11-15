# Cible XYZ pour le bras du robot facteur

Contrôle local du **NormaCore ElRobot follower, 7 articulations + pince**, via
Station. Le contrôleur transforme une cible en mètres dans `base_link` en un
chemin articulaire temporisé. La base Kiwi reste immobile pendant ce mouvement.
La pince et les roues ne reçoivent aucune commande de ce module.

## Réutilisation vérifiée

- [URDF officiel ElRobot](https://github.com/norma-core/norma-core/blob/6b494f77fac366a878ae71222458b196f8400dee/hardware/elrobot/simulation/elrobot_follower.urdf) :
  `rev_motor_01..07`, jusqu’à `Gripper_Base_v1_1`, sans les branches de pince.
- [Exemple officiel Station](https://github.com/norma-core/norma-core/tree/6b494f77fac366a878ae71222458b196f8400dee/software/station/examples/st3215-remote-teleop-py) :
  protocole `st3215/inference`, sync-write, registres et encodage vérifiés.
  `station.py` réutilise directement la classe `Station` de `hand_control/control.py`.
- [IKPy](https://github.com/Phylliade/ikpy/tree/v3.4.2) : lecture URDF, FK et IK bornée ;
  aucune réécriture du solveur. Masque des articulations construit par leurs noms.
- [Ruckig](https://github.com/pantor/ruckig/tree/v0.12.2) : trajectoires synchronisées
  avec limites de vitesse, accélération et jerk ; aucune interpolation de secours.

Le SDK existant et l’URDF sont issus du même commit officiel
`6b494f77fac366a878ae71222458b196f8400dee`. La recherche dans ce dépôt officiel
n’a pas trouvé de planificateur cartésien ElRobot à appeler directement.

## Essai hors ligne

Depuis **la racine du worktree** contenant ce répertoire, Python 3.10+ :

```bash
bash arm_control/setup.sh
arm_control/.venv/bin/python arm_control/control.py fk --joints 0 0 0 0 0 0 0
arm_control/.venv/bin/python arm_control/control.py plan \
  --start 0 0 0 0 0 0 0 --target 0.02 0.23 0.25 \
  --output /tmp/elrobot-plan.json
arm_control/.venv/bin/python -m unittest discover -s arm_control -v
```

Ces commandes ne se connectent pas à Station et ne déplacent pas le bras.
L’installation utilise un environnement et un SDK locaux ignorés par Git ;
elle ne démarre pas Station. Si les dépendances et le SDK existent ailleurs,
utiliser leur Python et les options globales `--sdk CHEMIN --urdf CHEMIN`.
Les tests acceptent aussi `ARM_URDF=/chemin/elrobot_follower.urdf` et
`STATION_SDK=/chemin/norma-core`.

Les radians suivent exactement les zéros et axes URDF. À zéro, le support de
pince est à `(0.002925, 0.259566, 0.218892) m` dans `base_link` ; ce n’est pas la
pose mesurée du bras. Par défaut la cible concerne **le support de pince**.
Renseigner `tcp_xyz_m`, translation exprimée dans `Gripper_Base_v1_1`, pour viser
le centre de prise ou le bord de la lettre. Ses axes restent ceux du support.

## Points de passage et insertion

`--mode joint` résout l’IK aux cibles, puis suit des segments articulaires
Ruckig, avec arrêt à chaque point de passage. Le trajet du TCP n’est pas droit.
Ajouter autant de `--via X Y Z` que nécessaire, dans l’ordre de passage.

`--mode linear` échantillonne une ligne cartésienne (5 mm par défaut), conserve
l’orientation initiale, borne chaque variation IK et vérifie la position et
l’orientation de chaque échantillon de commande. Chaque sous-segment finit à
vitesse nulle : cette première version privilégie une approche lente et vérifiable.
`--rpy R P Y` fixe une orientation dans `base_link`, en radians, convention XYZ
extrinsèque de SciPy. Une orientation inaccessible fait échouer tout le plan.

Exemple d’approche droite proche de la pose zéro **du modèle** :

```bash
arm_control/.venv/bin/python arm_control/control.py plan \
  --start 0 0 0 0 0 0 0 --mode linear --target 0.008925 0.251566 0.223892
```

Pour la boîte aux lettres, donner un point d’approche devant la fente, puis
avancer selon sa normale mesurée. Un décalage vertical Z ne définit pas à lui
seul une insertion. Les coordonnées caméra ou monde doivent d’abord être
transformées vers `base_link` avec une calibration extrinsèque ; la navigation
Kiwi et cette transformation ne sont pas implémentées ici.

## Préparation au matériel

Copier `elrobot.example.json` vers `arm_control/elrobot.local.json` (ignoré par Git).
Le modèle fourni est volontairement incomplet : IDs `1..7` indicatifs,
`hardware_verified=false` et zéros/signes/limites absents.

1. Vérifier sur le robot le bus du bras et le moteur de chaque `rev_motor_01..07`.
   Le bus doit être explicite pour distinguer bras et roues Kiwi.
2. Figer les plages dans Station. Lire les encodeurs, sans écriture, avec :

   ```bash
   arm_control/.venv/bin/python arm_control/control.py \
     --config arm_control/elrobot.local.json --host localhost state
   ```

3. Mesurer pour chaque articulation `zero_ticks` (encodeur correspondant à
   **q=0 dans l’URDF**) et `sign` (+1 ou -1 selon l’axe URDF). Le milieu de la
   plage Station ne détermine pas ce zéro. Valider ces correspondances et le TCP
   sur des poses connues. Choisir `min_ticks` et `max_ticks` à l’intérieur des
   limites Station, avec une marge mécanique adaptée au montage.
4. Mettre `hardware_verified=true` après vérification. La conversion est
   `ticks = zero_ticks + sign × q_rad × 4096/(2π)` ; pas de modulo ni d’écrêtage.
   Seules les plages sans franchissement 4095/0 sont prises en charge.

Planifier depuis les positions mesurées **sans mouvement** :

```bash
arm_control/.venv/bin/python arm_control/control.py \
  --config arm_control/elrobot.local.json plan --from-station \
  --target 0.02 0.23 0.25 --output /tmp/elrobot-plan.json
```

Une fois le montage et le chemin validés sur place, l’ajout explicite de
`--execute` à cette commande autorise l’envoi. Le bras doit déjà être alimenté,
couple activé, immobile et sous le contrôle exclusif de ce programme.
L’exécuteur recalcule le plan complet depuis la mesure ; il ne rejoue pas de
fichier JSON. Les consignes converties sont toutes validées avant la première
écriture. Il vérifie le départ, les limites, la fraîcheur, les erreurs moteur,
le suivi, les retards et l’arrivée mesurée avant de signaler un succès.

## Limites de cette étape

**Aucun contrôle de collision ni contournement automatique** : le JSON indique
`collision_checked=false`. Les points de passage sont fournis par l’appelant ;
les boîtes, le support, la lettre, les câbles, le châssis Kiwi et les collisions
du bras avec lui-même ne sont pas modélisés. Une cible atteignable ne garantit
pas un chemin libre. Une scène géométrique et un planificateur de collision
seront nécessaires pour automatiser ce choix.

Les limites Ruckig concernent les consignes calculées ; le réseau, les servos
et leur discrétisation ne garantissent pas le jerk réel. Les tolérances de
2 mm/0.03 rad sont numériques et configurables, pas une précision matérielle
mesurée. Le programme arrête les envois sur erreur et conserve le dernier
objectif/couple : ce dernier petit déplacement peut finir ; une perte réseau
ne garantit pas un arrêt physique. Aucun essai sur le robot n’a été effectué.
