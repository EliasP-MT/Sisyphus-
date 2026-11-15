# Mise au point du robot facteur

Toutes les commandes ci-dessous partent de la racine du worktree
`.hackathon/worktrees/local`, sur `hackathon/local`. Installer une seule fois avec
`bash arm_control/setup.sh`. Station doit déjà fonctionner (`--tcp`) sur l’hôte
visé ; ajouter `--host IP` avant la sous-commande si nécessaire.

`discover`, `capture`, `fit-calibration`, `fit-tcp` et `check-pose` n’envoient aucune commande
moteur. `jog` et `nudge` prévisualisent par défaut ; `--execute` autorise un essai.
Les JSON de mesure doivent finir par `.local.json` pour rester ignorés par Git.
`--output` refuse d’écraser un fichier existant.

## 1. Identifier le bus et les articulations

```bash
arm_control/.venv/bin/python arm_control/commission.py discover
cp -n arm_control/elrobot.example.json arm_control/elrobot.local.json
```

`discover` affiche tous les bus et IDs, leurs encodeurs, plages, fraîcheur et
état de calibration. Repérer **le bus du bras**, distinct des roues Kiwi, puis
renseigner `bus` et les sept `motor_id` dans `elrobot.local.json`. Les numéros
1..7 de l’exemple ne prouvent pas la correspondance mécanique.

Pour identifier les moteurs, vérifier d’abord le câblage et les IDs dans
Station. Un opérateur reste près du robot, base immobilisée, sans autre
programme de commande, bras sans lettre et dégagé des objets. Les plages
doivent être figées dans Station. Préparer et vérifier le couple dans Station,
en soutenant le bras si nécessaire ; le script ne l’active ni ne le relâche.

Prévisualiser un essai puis, lorsque l’espace est libre, l’exécuter :

```bash
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/elrobot.local.json \
  jog --joint rev_motor_01 --delta-ticks 8
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/elrobot.local.json \
  jog --joint rev_motor_01 --delta-ticks 8 --execute
```

Un jog cible **un seul ID**, de 1 à 16 ticks (maximum ≈ 1.4°), à vitesse limitée
avec profil Ruckig. Il ne touche ni à la pince ni aux autres IDs. Observer quelle
articulation bouge et dans quel sens par rapport à son axe URDF ; corriger le
mapping si nécessaire. Répéter articulation par articulation. Il n’y a pas de
retour automatique : un éventuel retour est un autre jog explicite de -8 ticks.
Le sens encodeur positif n’est pas forcément le sens URDF positif.

## 2. Mesurer les zéros et les signes

Le logiciel ne peut pas identifier les zéros URDF à partir des seuls encodeurs.
Il faut des références physiques : repères sur les pièces, gabarit, rapporteur
ou mesure optique calibrée, comparés au montage/axes de l’URDF officiel.
**Ne pas appeler « zéro » une pose simplement pratique ni le milieu de la plage.**

Pour chaque pose connue, mesurer les sept angles URDF (radians), immobiliser
le bras, puis capturer les encodeurs. Remplacer Q1..Q7 par ces nombres mesurés :

```text
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/elrobot.local.json \
  --output arm_control/pose-a.local.json capture --angles-rad Q1 Q2 Q3 Q4 Q5 Q6 Q7
```

Faire plusieurs poses connues, idéalement en variant une articulation à la
fois. Pour **chaque articulation**, les références doivent couvrir au moins
0.1 rad et 32 ticks. Deux poses suffisent mathématiquement si toutes les
articulations varient suffisamment ; des poses supplémentaires permettent
de détecter des mesures erronées. Conserver aussi une pose indépendante pour
valider le résultat. Pas de passage d’encodeur 4095/0 dans ces références.

```bash
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/elrobot.local.json \
  --output arm_control/calibration-candidate.local.json fit-calibration \
  --references arm_control/pose-*.local.json
```

Le calcul ajuste le signe et le zéro de chaque moteur, refuse les références
incohérentes (résidu > 12 ticks), et prend l’intersection des plages Station
avec une marge de 20 ticks. Choisir une marge supérieure si le montage le
nécessite. Le seuil de 12 ticks est un contrôle de cohérence, pas une précision
garantie. Le fichier candidat reste `hardware_verified=false`.

Comparer le candidat à une pose indépendante enregistrée dans
`arm_control/validation.local.json` et exclue de l’ajustement :

```bash
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/calibration-candidate.local.json \
  check-pose --reference arm_control/validation.local.json
```

Le rapport donne l’erreur par rapport aux références (seuil articulaire 0.03 rad
ou seuil XYZ configuré), sans changer le statut du candidat. Vérifier aussi le
mapping et les limites mécaniques. Après cette validation humaine,
mettre `hardware_verified=true` dans le candidat puis l’utiliser comme `--config`.
Conserver l’ancien fichier ; le script ne le remplace pas.

## 3. Mesurer le point de prise (TCP)

Le support de pince de l’URDF n’est pas nécessairement le centre de la lettre.
Avec le mapping validé, mesurer ce même point physique en XYZ dans `base_link`
sur au moins trois poses avec des orientations différentes. Remplacer X Y Z
par ces mesures en mètres ; changer le nom de fichier pour chaque capture :

```text
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/calibration-candidate.local.json \
  --output arm_control/tcp-a.local.json capture --measured-tcp-xyz X Y Z
```

```bash
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/calibration-candidate.local.json \
  --output arm_control/tcp-candidate.local.json fit-tcp --references arm_control/tcp-*.local.json
```

Le script calcule le décalage dans le repère du support de pince, vérifie les
orientations et la cohérence des mesures (résidu maximal 5 mm), puis remet
`hardware_verified=false`. Vérifier le TCP sur une autre pose avant de valider
ce nouveau candidat. Ces mesures supposent la position du socle et les angles
déjà corrects ; elles ne les étalonnent pas simultanément.

## 4. Petit déplacement XYZ

Depuis une pose dégagée et le candidat validé :

```bash
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/tcp-candidate.local.json \
  nudge --delta-mm 2 0 0
arm_control/.venv/bin/python arm_control/commission.py --config arm_control/tcp-candidate.local.json \
  nudge --delta-mm 2 0 0 --execute
```

Le déplacement relatif est exprimé dans `base_link`, orientation conservée,
limité à 5 mm et 0.05 rad par articulation par rapport au départ. Un mouvement
nécessitant un plus grand changement articulaire est refusé. L’arrivée est
contrôlée avec les encodeurs ; vérifier aussi physiquement le déplacement avec
une règle ou une mesure externe, car une calibration fausse peut sembler juste
dans le modèle. La tolérance XYZ configurée est de 2 mm : un très petit essai
peut ne pas se distinguer de cette tolérance.

Sur défaut de suivi, état périmé, erreur servo ou retard, les envois cessent.
Le dernier objectif et le couple restent actifs ; un déplacement en cours peut
finir. Utiliser l’arrêt matériel prévu sur votre montage en cas de besoin.
Les contrôles logiciels et l’amplitude réduite ne certifient pas un chemin libre.

## 5. CAD et évitement des collisions

Le CAD détaillé de la boîte est utile, mais des dimensions mesurées suffisent
pour commencer avec des volumes simples. Fournir :

- STEP, ou STL/OBJ avec unités explicites, parois et **ouverture réelle** de la
  fente (position, largeur, hauteur, profondeur et normale d’insertion).
- Pose de la boîte dans `base_link` : XYZ et orientation, ou repères mesurables
  permettant de calculer cette transformation. Le CAD seul ne donne pas la
  position de l’objet réel. Si Kiwi se déplace, cette pose doit être mise à jour.
- Géométrie/position du support à lettres, du châssis Kiwi et des obstacles ;
  dimensions de la lettre tenue et configuration de pince pendant le trajet.
- Une marge tenant compte des erreurs de pose, du TCP, du jeu et de la flexion.

La prochaine étape serait de charger les géométries de collision de l’URDF,
la boîte et les objets dans un moteur de collision, puis d’utiliser un
planificateur existant pour trouver un trajet libre. Les requêtes de distance
de [PyBullet](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/getClosestPoints.py)
et un planificateur [OMPL](https://ompl.kavrakilab.org/OMPL_Primer.pdf) permettent
de séparer validation géométrique et recherche du chemin.
Il faut vérifier les collisions du bras avec lui-même, l’objet tenu et
l’environnement, y compris les mouvements entre configurations et la trajectoire
finale après temporisation. Pour la boîte, utiliser des parois séparées ou une
géométrie conservant les ouvertures : une enveloppe pleine fermerait la fente.

Cette étape n’est pas implémentée ici : tous les essais indiquent encore
`collision_checked=false`. Aucun essai réel n’a été lancé pendant le développement.
