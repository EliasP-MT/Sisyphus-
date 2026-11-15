# Assistant local
Mission : jumeau MuJoCo robot facteur, bras ElRobot, base holonomique Kiwi, FSM/navigation/SLAM sans vision ; vérifier les butées.
Statut : en cours
Base Git : d11b446 ; branche hackathon/local ; worktree .hackathon/worktrees/local
Chemins réservés : digital_twin/ ; arm_control/ uniquement si adaptation d’interface indispensable.
Réservations vérifiées : aucun autre rôle présent ; aucun changement préexistant.
SHA livré : à venir
Tests : à venir, simulation uniquement, aucune commande robot.
Risques : paramètres mécaniques Kiwi, masse/friction et montage CAD non mesurés ; LiDAR virtuel non confirmé sur matériel.
Prochaine action : vérifier outils officiels, créer modèle et capteurs simulés, intégrer FSM/navigation/SLAM puis valider des scénarios.
Simulation visuelle demandée : viewer MuJoCo lancé sur le bureau VM (:0), mission seed 3, temps réel ; fenêtre « MuJoCo : postal_elrobot_kiwi » confirmée visible, état STOW à 17.5 s.
Processus : session exec 86344 ; PID dans digital_twin/.runtime/mujoco-live.pid du worktree local ; aucune commande au matériel.
Correction visuelle demandée : STEP exact (9 solides, écart de sommets MuJoCo < 1.2e-8 m), pièces distinguées ; plateau Kiwi séparé et sac placé latéralement, sans lettres. Décalage/hauteur nominaux configurables, empreinte navigation 0.30 m. Compilation, rendu et translation 1 s vérifiés ; jumeau global toujours en cours, non livré comme calibré.

Mission courte : murs à 20 cm maximum ; réservé digital_twin/model.py, réservation locale existante, aucun autre rôle.
Résultat murs : 4 murs abaissés de 0.80 à 0.20 m, centre z=0.10 m ; compilation MuJoCo et assertions hauteur/base réussies. SHA différé : model.py préexistant non suivi de la mission globale en cours, ne pas embarquer ce travail dans un commit court. Aucun matériel commandé.
Mission caméra : centrer au démarrage sur le robot ; réservé digital_twin/run.py, aucun autre rôle présent.
