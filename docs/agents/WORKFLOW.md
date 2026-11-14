# Équipe

| Rôle | Responsabilité | Merge |
| --- | --- | --- |
| pilotage | Découpage, priorités, arbitrage des chemins, architecture transverse | Non |
| robot | Contrôle, capteurs, matériel et tests associés | Non |
| donnees | Station, collecte, jeux de données et outils associés | Non |
| integration | Revue, tests d'ensemble, intégration séquentielle | Oui, uniquement dans hackathon/integration |

Les trois premiers sont des interlocuteurs indépendants : l'utilisateur peut leur parler directement.
Pilotage ne transmet pas automatiquement les messages aux autres terminaux. BOARD.md et les fichiers de rôle sont la boîte de passage, consultée au début et à la fin d'une mission, sans polling.

## Contrat d'une mission

Pilotage inscrit dans BOARD.md : identifiant, objectif, responsable, chemins réservés, critères d'acceptation et dépendances. Le responsable vérifie les réservations avant toute modification. Le partage d'un même fichier nécessite un ordre explicite. Une tâche courte reste locale ; ne pas déléguer pour simplement lire ou chercher. Viser un échange de cadrage et un compte rendu par tâche. Le budget de tokens est une consigne, pas un quota techniquement garanti.

Chaque rôle tient son fichier : mission, statut (libre / en cours / prêt / bloqué), base Git, chemins, SHA livré, tests, risques, prochaine action. Un blocage produit une question précise puis l'agent attend. Les fichiers de coordination ne contiennent aucun secret.

## Intégration

1. Attendre un statut prêt et un commit propre ; examiner `git diff HEAD...hackathon/<role>` et les tests annoncés.
2. Vérifier que le worktree integration est propre. Mettre à jour les branches uniquement entre missions, jamais pendant le travail d'un autre agent.
3. L'intégrateur fusionne une branche à la fois avec `git merge --no-ff hackathon/<role>`, puis exécute les tests pertinents. En cas de conflit sémantique, demander au propriétaire ; ne pas inventer un comportement matériel.
4. En cas d'échec avant validation, ne pas publier ; documenter le problème et corriger ou abandonner un merge encore en cours avec `git merge --abort`.
5. Inscrire le SHA d'intégration et les résultats dans integration.md. Ne jamais réécrire l'historique partagé.
6. L'utilisateur décide quand publier ou intégrer au tronc. Depuis un tronc propre : `git merge --ff-only hackathon/integration`. Si le tronc a avancé, faire résoudre l'intégration par l'intégrateur avant de réessayer.

Les worktrees partent de HEAD : les modifications non commitées du dépôt principal ne sont pas disponibles aux agents. Commiter explicitement les fichiers utiles avant de démarrer une mission. Le bootstrap copie AGENTS.md si absent dans la base ; ce fichier peut ensuite être commité avec les règles de coordination.
