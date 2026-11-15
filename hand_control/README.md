# Test webcam → bras ST3215

Ce test utilise MediaPipe Hands, OpenCV et le client Python officiel de NormaCore
(commit `6b494f77fac366a878ae71222458b196f8400dee`).
Source : https://github.com/norma-core/norma-core/tree/main/software/station/examples/st3215-remote-teleop-py

## Installation

Depuis la racine du projet, avec Python 3.10 (testé), Git et curl :

```bash
./hand_control/setup.sh
```

L'environnement Python et le SDK restent dans `hand_control/`, sans sudo.

## Webcam et Station

Dans cette VM, la webcam VMware est capturée directement par Station et ne crée
pas de `/dev/video0`. L'option `--camera-queue` réutilise ce flux sans conflit.
Garde l'instance Station habituelle (`./station --web --tcp`) lancée.
La queue ci-dessous a été vérifiée sur ta VM ; son nom peut changer si tu changes
de caméra. La fenêtre OpenCV nécessite une session graphique.

Pour une webcam OpenCV classique, omets `--camera-queue` et utilise `--camera 0`.
Si Station capture la même caméra, arrête-le puis relance-le avec
`--config ../hand_control/station-webcam.yaml --web --tcp` depuis `station/`.
Le script peut aussi tourner sur le PC hôte avec `--host IP_DE_LA_VM`.

## Aperçu sans mouvement

Depuis la racine du projet, dans un autre terminal :

```bash
hand_control/.venv/bin/python hand_control/control.py \
  --camera-queue usbvideo/5d04f7399229fb2350705502fc9e95e4
```

Une main suffit. Gauche/droite pilote la base, haut/bas l'épaule et
l'écartement pouce–index la pince. Il s'agit d'un mapping de trois articulations,
pas d'une reconstruction 3D ni d'une cinématique inverse. Le coude et les autres
articulations restent à leur position actuelle.

## Contrôle réel

Vérifie dans Station les IDs correspondant à la base, à l'épaule et à la pince.
Les IDs ci-dessous sont un exemple SO101, à adapter à ton bras. Les trois moteurs
doivent être détectés et leur calibration figée dans Station.

```bash
hand_control/.venv/bin/python hand_control/control.py \
  --camera-queue usbvideo/5d04f7399229fb2350705502fc9e95e4 \
  --live --bus 5B8E114718 --base 1 --shoulder 2 --gripper 6
```

- **Espace** : armer / mettre en pause. Le bras ne saute pas à une pose prédéfinie :
  les poses actuelles du bras et de la main deviennent les références.
- **Q ou Échap** : quitter.
- Main perdue, mesure ancienne (> 500 ms), moteur absent ou erreur servo : pause,
  puis Espace requis pour réarmer.

L'amplitude relative est de 300 ticks par largeur/hauteur d'image ou plage de
pincement, limitée par la calibration. La consigne progresse à 100 ticks/s ;
vitesse servo 100 et accélération 10. Les directions peuvent dépendre du montage
mécanique : commence près de la référence et vérifie chaque axe.

En pause et à la sortie, le dernier objectif et le couple restent actifs pour
éviter de laisser tomber un bras levé. Cela n'est pas un arrêt d'urgence matériel.
Une consigne déjà reçue peut terminer son petit déplacement. Une perte réseau ne
permet pas de garantir l'envoi d'un ordre d'arrêt au contrôleur.

Si aucun moteur n'est détecté, l'aperçu reste utilisable, mais le script refuse
l'armement. Il ne résout pas une absence d'alimentation des moteurs.

## Vérification

```bash
cd hand_control
python3 -m unittest -v
```

Les tests couvrent le pincement, les limites, la progression des consignes,
l'absence de saut initial et le refus des données absentes/périmées.
