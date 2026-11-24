# Évitement du châssis et du sac à lettres

Le fichier fourni est conservé **sans modification** sous
`assets/full_robot.step` (SHA-256
`cf0028e27277768bb504c611d7a2f959249d54ea7a5f956b3a922118bdbae0d8`).
Il contient neuf solides. L’import OpenCascade, avec unité de transfert mm
explicite puis conversion en mètres, donne une enveloppe d’environ
`0.144 × 0.211 × 0.103 m`. Confirmer que cela correspond au montage réel.
Les noms de composants sont des horodatages, pas des noms de liens du bras.
La [vue numérotée](assets/full_robot_parts.png) identifie les solides 0 à 8.

## Ce qui est implémenté

- Cinématique et géométries du bras : URDF officiel ElRobot, sept articulations.
- Châssis/sac : un maillage par solide STEP, avec placements d’assemblage
  conservés. Les pièces fixes utilisent des triangles concaves, sans transformer
  le sac creux en une enveloppe convexe pleine. OpenCascade vérifie aussi si un
  centre de géométrie du bras est à l’intérieur de la matière d’un solide fermé.
- Pièces CAD mobiles : rattachement à un lien URDF, avec transformation locale
  calculée dans la pose de référence. Leur enveloppe convexe sert de géométrie
  conservatrice. La géométrie URDF reste également présente.
- Auto-collisions entre groupes rigides non adjacents du bras ; contacts des
  mécanismes voisins et contacts internes de pince exclus. La pince est figée
  à `gripper_angle_rad` ; ses joints mimic sont synchronisés.
- Obstacles supplémentaires en volumes parallélépipédiques et lettre tenue
  optionnelle, attachée au TCP.
- En mode `joint`, recherche bornée de solutions IK alternatives si la première
  est en collision, puis liaison directe ou **OMPL RRTConnect**. Les détours sont
  temporisés avec Ruckig, avec arrêt aux sommets. Un échec de recherche est
  signalé ; aucune solution approximative n’est exécutée.
- En mode `linear`, vérification de chaque segment sans détour : une insertion
  droite bloquée est refusée.
- Vérification des segments après temporisation et **après arrondi en ticks**,
  empreinte de la scène/configuration, puis contrôle de la pose mesurée pendant
  l’exécution. Aucun plan non contrôlé n’est envoyé par l’exécuteur XYZ.

Bibliothèques réutilisées : [OpenCascade/OCP](https://github.com/CadQuery/OCP),
[PyBullet](https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/getClosestPoints.py),
[OMPL](https://ompl.kavrakilab.org/core/stateValidation.html), IKPy et Ruckig.

## Essai de géométrie hors matériel

L’installation `bash arm_control/setup.sh` inclut les dépendances de collision.
Les maillages dérivés et leur manifeste sont versionnés. Pour les régénérer :

```bash
arm_control/.venv/bin/python arm_control/cad.py arm_control/assets/full_robot.step \
  --output arm_control/assets/full_robot_collision
```

Depuis la racine du worktree, afficher la pose zéro et exporter une vue combinée :

```bash
arm_control/.venv/bin/python arm_control/control.py \
  --scene arm_control/robot_scene.example.json --preview /tmp/elrobot-scene.png \
  fk --joints 0 0 0 0 0 0 0
arm_control/.venv/bin/python arm_control/control.py \
  --scene arm_control/robot_scene.example.json plan --start 0 0 0 0 0 0 0 \
  --target 0.02 0.23 0.25 --output /tmp/elrobot-plan.json
```

**L’exemple est une scène de développement non recalée.** Sa translation et sa
rotation nulles n’affirment pas que l’origine CAD est celle du bras. Il rattache
provisoirement les neuf solides au socle. Le programme refuse son utilisation
sur le matériel tant que `hardware_verified=false`.

## Recaler et affecter les pièces

1. Vérifier la taille du modèle et les IDs dans la vue numérotée.
2. Pour les solides fixes du châssis et du sac, garder `parent_link=base_link`.
   Déplacer chaque pièce CAD du bras mobile vers son lien URDF réel, par exemple
   `Joint_02_1` ou `Gripper_Base_v1_1`. Le nom doit exister dans l’URDF. Chaque
   solide doit être affecté exactement une fois : rien n’est ignoré implicitement.
3. Mesurer la pose CAD → `base_link`. Renseigner `cad.xyz_m` et `cad.rpy_rad`,
   ou utiliser les points de référence ci-dessous. Pour des pièces CAD mobiles,
   renseigner aussi `reference_q_rad` : angles URDF correspondant à l’assemblage
   CAD, pas nécessairement tous nuls. La transformation locale suit ensuite le
   lien lorsque les angles changent.
4. Vérifier l’ouverture de pince modélisée, le TCP, les objets supplémentaires
   et les marges. `ignore_robot_links` dans une liaison CAD, `allowed_self_pairs`
   (noms de liens) et `allowed_cad_pairs` (IDs de solides) permettent d’exclure
   uniquement des contacts mécaniques intentionnels identifiés. Ne pas les
   utiliser pour faire disparaître une collision inexpliquée.
5. Exporter plusieurs aperçus avec `--preview` sur des poses mesurées. Vérifier
   aussi physiquement les dégagements. Après cette vérification, mettre
   `hardware_verified=true` dans une copie `robot_scene.local.json`.

Recalage par points : créer un fichier local contenant au moins trois repères
non alignés, en correspondance et répartis sur le châssis. Structure :

```json
{
  "cad_units": "mm",
  "cad_points": [[0, 0, 0], [100, 0, 0], [0, 100, 0]],
  "base_points_m": [[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0]]
}
```

Ces nombres illustrent le format : les remplacer par les mesures réelles.

```bash
arm_control/.venv/bin/python arm_control/register_scene.py \
  --scene arm_control/robot_scene.example.json \
  --measurements arm_control/landmarks.local.json \
  --output arm_control/robot_scene.local.json
```

L’ajustement utilise `scipy.spatial.transform.Rotation.align_vectors`, refuse
les points alignés ou un résidu maximal supérieur à 5 mm, et laisse la validation
matérielle à `false`. Un petit résidu n’exclut pas une erreur commune à toutes
les mesures ; vérifier avec d’autres repères et des poses de bras indépendantes.

## Lettre et obstacles

`boxes` contient des objets avec `name`, `size_m`, `xyz_m`, `rpy_rad` dans
`base_link`. `payload` accepte la même structure, mais sa pose est dans le
**repère du TCP** et suit le bras. Ajouter `ignore_robot_links` pour les surfaces
de pince qui touchent volontairement la lettre, et éventuellement
`ignore_cad_solids` pour les pièces CAD correspondantes. `payload=null` modélise
un bras sans lettre ; cette valeur ne détecte pas la géométrie d’une lettre tenue.

Le sac/châssis suit le socle. Cette scène locale n’assure pas l’évitement de
l’environnement pendant la navigation Kiwi. Les boîtes aux lettres extérieures
doivent avoir leur pose actualisée dans `base_link` après un déplacement.

## Exécution

L'exécution passe par la station ROS 2 : le planner worker charge la scène
(`station.sh` : variable `SCENE`), le robot exécute la trajectoire validée.
Voir `docs/ros2-runtime.md`.

## Limites pratiques

La vérification reste **discrète**, avec au plus `edge_step_rad` par axe entre
échantillons (0.005 rad par défaut). Les trajectoires nominales entre ces
échantillons ne font pas l’objet d’une preuve de non-collision continue ;
réduire ce pas pour les passages étroits et garder une marge adaptée.
Les valeurs par défaut sont 5 mm pour le CAD/les objets, 1 mm pour les
auto-collisions, et 0.2 mm de déflexion du maillage CAD. Elles ne remplacent pas
une mesure des erreurs de recalage, de flexion et de suivi sur le robot.

Les enveloppes convexes des liens URDF et des pièces CAD mobiles peuvent
refuser un passage pourtant réalisable. Les contacts internes de pince et les
groupes articulaires voisins ne sont pas contrôlés comme auto-collisions.
Les câbles et les déformations ne sont pas modélisés. Une collision détectée
pendant le suivi arrête l’envoi, mais la dernière consigne reçue peut finir.
Ce module n’est pas un arrêt matériel. Aucun essai réel n’a été lancé.
