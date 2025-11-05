# Nouvelle Fonctionnalité: Configuration des Collisions

## Date: 2025-11-04

## 📋 Vue d'ensemble

Cette mise à jour remplace l'ancien dialogue de configuration des joints par une nouvelle interface dédiée à la configuration des self-collisions. Cette modification répond à un besoin important : **les limites de joints sont déjà dans l'URDF** et cuRobo peut les lire directement, mais **la configuration des collisions doit être définie manuellement**.

---

## 🎯 Objectif

Permettre aux utilisateurs de configurer facilement:
1. **Self-Collision Ignore Pairs**: Quelles paires de links ne doivent pas être vérifiées pour les collisions
2. **Self-Collision Buffers**: Marges de sécurité pour chaque link

---

## ✨ Nouvelle Interface: CollisionConfigDialog

### Fonctionnalités principales

#### 1. Sélection des Links
- Liste de tous les links du robot
- Sélection interactive d'un link pour le configurer

#### 2. Gestion des Ignore Pairs
- **Available Links List**: Links qui peuvent être ajoutés à la liste d'ignore
- **Ignored Links List**: Links actuellement ignorés pour le link sélectionné
- **Add to Ignore List**: Ajouter des links à ignorer
- **Remove from Ignore List**: Retirer des links de la liste d'ignore

#### 3. Configuration des Buffers
- Définition d'un buffer de collision pour chaque link
- Valeurs typiques: 0.0 à 0.01 mètres
- Mise à jour en temps réel

#### 4. Fonctionnalités Automatiques
- **Generate Adjacent Pairs**: Génère automatiquement les ignore pairs pour les links adjacents dans la chaîne cinématique
- **Clear All Ignores**: Efface toutes les ignore pairs

### Interface Utilisateur

```
┌─────────────────────────────────────────────────────────────────┐
│  Self-Collision Configuration                                   │
├─────────────────────────────────────────────────────────────────┤
│  Configure which link pairs should ignore self-collision...     │
├─────────────────┬────────────────────────┬────────────────────┤
│ Robot Links     │ Collision Ignore List  │ Collision Buffer   │
│                 │                        │                    │
│ □ base_link     │ Available:             │ Buffer (m):        │
│ □ link1         │ □ link2                │ ┌──────┐          │
│ ☑ link2         │ □ link3                │ │ 0.005│          │
│ □ link3         │ □ link4                │ └──────┘          │
│ □ link4         │                        │                    │
│ □ link5         │ ➜ Add to Ignore List   │ Buffer adds        │
│ □ link6         │                        │ safety margin...   │
│                 │ Currently ignored:     │                    │
│                 │ ☑ link1                │                    │
│                 │ ☑ link3                │                    │
│                 │                        │                    │
│                 │ ✖ Remove from List     │                    │
└─────────────────┴────────────────────────┴────────────────────┘
│ 🔗 Generate Adjacent Pairs    🗑 Clear All Ignores             │
│                                            [Cancel]  [   OK   ] │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔄 Changements Apportés

### 1. Nouveaux Fichiers

#### `include/curobo_robot_setup/collision_config_dialog.hpp`
- Déclaration de la classe `CollisionConfigDialog`
- Interface pour configurer les self-collisions
- Méthodes pour générer automatiquement les pairs adjacentes

#### `src/collision_config_dialog.cpp`
- Implémentation complète du dialogue
- Logique de gestion des ignore pairs
- Algorithme de détection des links adjacents
- Interface utilisateur avec Qt

### 2. Fichiers Modifiés

#### `include/curobo_robot_setup/curobo_config.hpp`
**Avant:**
```cpp
struct JointConfig {
  bool active = true;
  double pos_min = -3.14;
  double pos_max = 3.14;
  double vel_max = 1.0;
  double acc_max = 1.0;
  double jerk_max = 1.0;
  std::vector<std::string> ignore_collisions;
};
```

**Après:**
```cpp
struct JointConfig {
  bool active = true;
  // Note: All joint limits are read from URDF by cuRobo
};
```

**Raison**: Simplification - les limites viennent de l'URDF.

#### `src/curobo_robot_setup_panel.cpp`
- Remplacement de `JointConfigDialog` par `CollisionConfigDialog`
- Mise à jour de `onConfigureJoints()` pour utiliser le nouveau dialogue
- Mise à jour de `updateConfigTab()` pour afficher le nombre d'ignore pairs
- Import changé: `#include "collision_config_dialog.hpp"`

#### `CMakeLists.txt`
- Ajout de `collision_config_dialog.cpp` aux sources
- Ajout de `collision_config_dialog.hpp` aux headers

### 3. Fichiers Conservés (rétrocompatibilité)

- `joint_config_dialog.hpp` et `.cpp` sont conservés dans le code
- Peuvent être réutilisés si besoin futur
- Pas de rupture de compatibilité

---

## 📖 Guide d'Utilisation

### Étape 1: Ouvrir la Configuration

1. Charger un URDF dans le plugin
2. Aller dans l'onglet **Configuration**
3. Cliquer sur **Configure Joints** (le bouton a gardé son nom mais ouvre maintenant le dialogue de collision)

### Étape 2: Générer les Pairs Adjacentes (Recommandé)

1. Cliquer sur **🔗 Generate Adjacent Pairs**
2. Le système détecte automatiquement les links connectés par des joints
3. Ajoute les ignore pairs pour éviter les faux positifs de collision

**Résultat typique:**
```yaml
self_collision_ignore:
  base_link: [link1]
  link1: [base_link, link2]
  link2: [link1, link3]
  link3: [link2, link4]
  link4: [link3, link5]
  link5: [link4, link6]
  link6: [link5]
```

### Étape 3: Ajustements Manuels

Si certains links ne peuvent jamais se toucher (par exemple, link2 et link6 dans certaines configurations):

1. Sélectionner **link2** dans la liste des links
2. Sélectionner **link6** dans la liste "Available links"
3. Cliquer sur **➜ Add to Ignore List**

### Étape 4: Configurer les Buffers (Optionnel)

Pour chaque link critique:

1. Sélectionner le link
2. Ajuster le buffer (0.0 à 0.01 m)
3. Valeurs typiques:
   - `0.0`: Pas de marge supplémentaire
   - `0.005`: Marge conservatrice (5mm)
   - `0.01`: Marge très conservatrice (10mm)

### Étape 5: Sauvegarder

1. Cliquer sur **OK** dans le dialogue
2. Cliquer sur **Save YAML** dans l'onglet Configuration
3. Le fichier YAML contient maintenant les configurations de collision

---

## 📄 Format YAML Généré

```yaml
robot_cfg:
  kinematics:
    # ... URDF and other config ...

    collision_sphere_buffer: 0.01

    self_collision_ignore:
      base_link: [link1]
      link1: [link2, link3]
      link2: [link3, link5]
      link3: [link4, link5, link6]
      link4: [link5, link6]
      link5: [link6]

    self_collision_buffer:
      link1: 0.0
      link2: 0.005
      link3: 0.0
      link4: 0.0
      link5: 0.005
      link6: 0.0

    cspace:
      joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
      # Note: Joint limits are read from URDF, not from config file
```

---

## 💡 Avantages de cette Approche

### 1. Séparation des Responsabilités
- **URDF**: Contient la géométrie, les joints, les limites
- **Config cuRobo**: Contient les sphères de collision et les règles de collision

### 2. Moins de Redondance
- Pas besoin de dupliquer les limites de joints
- Source unique de vérité: l'URDF

### 3. Interface Plus Claire
- Focus sur ce qui doit être configuré manuellement
- Moins de champs à remplir
- Moins d'erreurs potentielles

### 4. Génération Automatique
- Détection automatique des links adjacents
- Gain de temps significatif
- Moins d'oublis

### 5. Meilleure Expérience Utilisateur
- Interface visuelle intuitive
- Feedback immédiat
- Undo/Redo via Remove from List

---

## 🔍 Algorithme de Détection des Links Adjacents

```cpp
Pour chaque joint dans le robot:
  Si le joint n'est pas FIXED:
    parent = joint.parent_link
    child = joint.child_link

    Ajouter child à ignore_pairs[parent]
    Ajouter parent à ignore_pairs[child]
```

**Pourquoi ignorer les adjacents?**
- Les links connectés par un joint ne peuvent pas se toucher physiquement
- Vérifier leur collision est un gaspillage de calcul
- Peut causer des faux positifs

---

## ⚙️ Détails Techniques

### Structure de Données

```cpp
struct SelfCollisionConfig {
  // Map: link_name -> list of links to ignore
  std::map<std::string, std::vector<std::string>> ignore_pairs;

  // Map: link_name -> buffer distance
  std::map<std::string, double> buffer_distances;

  // Global collision sphere buffer
  double collision_sphere_buffer = 0.0;
};
```

### Méthodes Principales

```cpp
class CollisionConfigDialog {
  // Génère automatiquement les pairs adjacentes
  std::map<std::string, std::vector<std::string>> generateAdjacentIgnorePairs();

  // Vérifie si deux links sont adjacents
  bool areLinksAdjacent(const std::string& link1, const std::string& link2);

  // Met à jour l'affichage des ignore lists
  void updateIgnoreList();

  // Ajoute des links à la liste d'ignore
  void onAddIgnorePair();

  // Retire des links de la liste d'ignore
  void onRemoveIgnorePair();
};
```

---

## 🧪 Tests Recommandés

### Test 1: Génération des Pairs Adjacentes

1. Charger un URDF simple (6 DOF)
2. Cliquer "Generate Adjacent Pairs"
3. Vérifier que chaque link ignore ses voisins directs
4. Sauvegarder et vérifier le YAML

**Résultat attendu:**
```yaml
self_collision_ignore:
  base_link: [link1]
  link1: [base_link, link2]
  link2: [link1, link3]
  # etc.
```

### Test 2: Ajout Manuel

1. Sélectionner link2
2. Ajouter link5 à ignore
3. Vérifier que link5 apparaît dans ignored links
4. Sauvegarder et vérifier

**Résultat attendu:**
```yaml
self_collision_ignore:
  link2: [link1, link3, link5]  # link5 ajouté manuellement
```

### Test 3: Buffers

1. Sélectionner link1
2. Définir buffer à 0.005
3. Répéter pour d'autres links
4. Sauvegarder et vérifier

**Résultat attendu:**
```yaml
self_collision_buffer:
  link1: 0.005
  link2: 0.0
  # etc.
```

### Test 4: Clear All

1. Configurer plusieurs ignore pairs
2. Cliquer "Clear All Ignores"
3. Vérifier que toutes les pairs sont effacées
4. Confirmer avec dialog de confirmation

---

## 📚 Mise à Jour de la Documentation

### TUTORIAL.md
Mettre à jour la section "Setting Up Self-Collision Avoidance" avec:
- Nouveau dialogue de configuration
- Bouton "Generate Adjacent Pairs"
- Screenshots de la nouvelle interface

### QUICKSTART.md
Mettre à jour l'étape 3 (configuration) avec:
- Mention du nouveau dialogue
- Recommandation d'utiliser "Generate Adjacent Pairs"

### README.md
Mettre à jour la section "Configuration" pour mentionner:
- La configuration des self-collisions
- Les limites de joints lues depuis l'URDF

---

## 🎓 FAQ

**Q: Pourquoi ne pas configurer les limites de joints?**
**R**: cuRobo lit les limites directement depuis l'URDF. Pas besoin de les dupliquer dans la configuration.

**Q: Que fait "Generate Adjacent Pairs"?**
**R**: Détecte automatiquement les links connectés par des joints et les ajoute aux ignore pairs.

**Q: Dois-je toujours utiliser "Generate Adjacent Pairs"?**
**R**: Oui, c'est recommandé comme point de départ. Vous pouvez ensuite ajouter manuellement d'autres pairs.

**Q: Quelle valeur de buffer utiliser?**
**R**: 0.0 pour la plupart des links. 0.005-0.01 pour les links critiques où vous voulez plus de marge.

**Q: Puis-je avoir des ignore pairs asymétriques?**
**R**: Oui, vous pouvez configurer link1 → ignore link2 sans que link2 → ignore link1.

**Q: Comment réinitialiser la configuration?**
**R**: Utilisez "Clear All Ignores" puis "Generate Adjacent Pairs" pour repartir à zéro.

---

## 🚀 Prochaines Améliorations Possibles

1. **Visualisation des Ignore Pairs**
   - Afficher graphiquement quels links s'ignorent
   - Coloration dans RViz2

2. **Validation Intelligente**
   - Avertir si deux links proches ne s'ignorent pas
   - Suggérer des ignore pairs basés sur la géométrie

3. **Templates de Configuration**
   - Sauvegarder des configurations réutilisables
   - Bibliothèque de configs pour robots communs

4. **Import/Export Partiel**
   - Importer seulement les ignore pairs d'un fichier
   - Exporter seulement les buffers

---

## ✅ Checklist de Migration

Si vous utilisez une version antérieure:

- [ ] Recompiler le package
- [ ] Ouvrir le dialogue de configuration
- [ ] Utiliser "Generate Adjacent Pairs"
- [ ] Vérifier le YAML généré
- [ ] Tester avec cuRobo
- [ ] Mettre à jour vos scripts/workflows

---

**Note**: Les anciens fichiers de configuration YAML restent compatibles. Le chargement d'un vieux fichier fonctionne toujours.

**Fin du document**
