# Session hackathon

Depuis le dépôt :

```bash
./scripts/hackathon setup   # prépare les branches et worktrees, sans lancer de modèle
./scripts/hackathon start   # vérifie les dépendances puis ouvre tmuxinator
```

Installer au préalable tmux, tmuxinator, lazygit et Codex CLI, puis authentifier Codex. Le lanceur indique la première dépendance manquante. Aucune installation système n'est effectuée automatiquement. La configuration reste dans le dépôt ; aucun fichier personnel dans ~/.config n'est requis.

La session comporte six fenêtres : pilotage (agent + terminal libre), robot, donnees, integration, git (lazygit sur la branche d'intégration), commandes (shell seul dans le dépôt principal). Avec le préfixe tmux par défaut `Ctrl-b` : `n` / `p` change de fenêtre, flèches change de panneau, `z` agrandit le panneau courant, `d` détache. Relancer start retrouve la session existante.

Chaque agent a sa branche `hackathon/<role>` et son répertoire `.hackathon/worktrees/<role>`. Git partage les commits entre worktrees, mais pas les fichiers modifiés. Les fichiers locaux existants dans le dépôt principal restent à leur place. Les worktrees réutilisent leurs branches aux lancements suivants ; ils ne sont jamais réinitialisés automatiquement.

Avant les premières missions, commiter les fichiers de configuration et le code à partager en les sélectionnant explicitement. Si setup a déjà été lancé, l'intégrateur peut fusionner ce commit puis les agents peuvent actualiser leurs branches, entre deux missions et avec des arbres propres. Les fichiers copiés AGENTS.md doivent être identiques avant cette fusion ; les commiter ou les mettre temporairement de côté pour éviter une collision de fichier non suivi.

Exemple à donner à pilotage : « Prépare deux missions indépendantes, robot et donnees, réserve les fichiers et écris les critères dans BOARD.md ; ne lance pas de sous-agent. » Puis dans robot : « Prends la mission robot de BOARD.md ». Enfin dans integration : « Examine les livraisons prêtes et intègre celles dont les tests passent ».

Lire [le workflow](agents/WORKFLOW.md) pour les responsabilités et le protocole de merge. Les échanges sont des fichiers Markdown locaux, pas un bus de messages automatique. Les agents attendent une mission ; ouvrir quatre sessions ne déclenche pas quatre développements. Le raisonnement est réglé sur medium ; aucun modèle précis n'est imposé. Ajuster la ligne Codex du lanceur selon les modèles disponibles et la difficulté. Les plafonds de délégation et les rapports courts limitent le gaspillage, mais ne constituent pas un plafond de facturation.

Lazygit permet d'inspecter les branches et les diffs. Laisser l'intégrateur réaliser les merges pour éviter deux opérations Git concurrentes dans son worktree. Ne pas lancer de commandes matérielles depuis plusieurs agents.

La configuration tensegrity n'était pas présente dans cet environnement : cette disposition est une proposition autonome, à ajuster si cette référence devient disponible.
