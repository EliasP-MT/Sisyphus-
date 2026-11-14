# Coordination du hackathon

- Lire `docs/agents/WORKFLOW.md` dans le dépôt de contrôle indiqué par HACKATHON_ROOT.
- Le rôle courant est HACKATHON_ROLE. Sans rôle, agir comme assistant local : ne pas intégrer de branches.
- Seul `integration` effectue les merges, dans son worktree. Ne jamais merger dans le dépôt de contrôle, pousser, déployer ou commander le robot sans demande explicite de l'utilisateur.
- Chaque agent travaille uniquement dans son worktree et sa branche `hackathon/<role>`.
- Ne pas modifier les fichiers d'un autre worktree. Ne pas toucher aux changements préexistants.
- Avant de coder : lire la mission, annoncer les chemins réservés dans son fichier de coordination et vérifier les réservations des autres rôles. En cas de conflit, attendre l'arbitrage de pilotage.
- Les fichiers partagés vivent dans `$HACKATHON_ROOT/.hackathon/coordination/`. Chaque rôle écrit uniquement son propre fichier ; pilotage écrit aussi BOARD.md.
- Ne pas lancer d'autres agents par défaut. Déléguer seulement une tâche indépendante, bornée, avec résultat attendu et budget explicite ; un seul sous-agent simultané par orchestrateur. Pas de délégation récursive.
- Économiser le contexte : recherches ciblées, extraits courts, tests proportionnés, compte rendu de 15 lignes maximum. Ne pas relire le dépôt entier ni boucler en attente.
- Aucun travail sans mission : attendre la demande humaine. Une mission terminée donne lieu à un commit ciblé, un SHA, les tests et les risques dans le fichier du rôle. Ne jamais `git add .`.
- Ne pas inclure `.hackathon/`, les secrets, données Station, SDK ou environnements virtuels dans les commits.
