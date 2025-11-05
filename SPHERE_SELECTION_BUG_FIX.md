# Correction du Bug de Sélection de Sphère

## Date: 2025-11-04

## 🐛 Description du Bug

### Symptôme
Lorsqu'on charge un fichier URDF et un fichier YAML contenant des sphères :
1. On sélectionne une sphère dans l'arbre des sphères
2. Les spinboxes (X, Y, Z, Radius) se mettent à jour avec les valeurs de la sphère
3. **PROBLÈME**: La sphère se met à jour immédiatement avec de nouvelles valeurs
4. Les coordonnées sont modifiées en fonction du global frame, pas du link parent

### Scénario de Reproduction

```
1. Charger robot.urdf
2. Charger config.yaml (contient des sphères existantes)
3. Dans l'arbre des liens, sélectionner "link1"
4. Dans l'arbre des sphères, cliquer sur une sphère de "link2"
5. Observer: Les spinboxes affichent la position de la sphère
6. BUG: La sphère de "link2" se modifie avec les coordonnées du frame de "link1"
```

### Cause Racine

Le problème est dans le flux d'événements Qt :

```cpp
// onSphereSelected() est appelé
void CuRoboSetupPanel::onSphereSelected(QTreeWidgetItem* item, int column)
{
  // ...
  const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
  if (sphere) {
    ui_->doubleSpinBox_x->setValue(sphere->position.x());  // ❌ Déclenche valueChanged!
    ui_->doubleSpinBox_y->setValue(sphere->position.y());
    ui_->doubleSpinBox_z->setValue(sphere->position.z());
  }
}

// setValue() déclenche valueChanged, qui appelle:
void CuRoboSetupPanel::onPositionChanged(double value)
{
  if (!selected_sphere_id_.empty()) {
    const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
    if (sphere) {
      Eigen::Vector3d new_pos(
        ui_->doubleSpinBox_x->value(),
        ui_->doubleSpinBox_y->value(),
        ui_->doubleSpinBox_z->value()
      );
      // ❌ Met à jour la sphère IMMÉDIATEMENT avec les valeurs actuelles des spinboxes!
      sphere_manager_->updateSphere(selected_sphere_id_, sphere->radius, new_pos);
    }
  }
}
```

**Le problème**:
- Sélectionner une sphère déclenche la mise à jour des spinboxes
- Mettre à jour les spinboxes déclenche `onPositionChanged`
- `onPositionChanged` modifie la sphère sans vérifier le contexte
- La sphère est modifiée même si elle appartient à un autre link

---

## ✅ Solution Implémentée

### 1. Bloquer les Signaux lors de la Mise à Jour des Spinboxes

```cpp
void CuRoboSetupPanel::onSphereSelected(QTreeWidgetItem* item, int column)
{
  // ...
  const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
  if (sphere) {
    // ✅ BLOQUER les signaux temporairement
    ui_->doubleSpinBox_radius->blockSignals(true);
    ui_->doubleSpinBox_x->blockSignals(true);
    ui_->doubleSpinBox_y->blockSignals(true);
    ui_->doubleSpinBox_z->blockSignals(true);

    // Maintenant setValue() ne déclenche PAS valueChanged
    ui_->doubleSpinBox_radius->setValue(sphere->radius);
    ui_->doubleSpinBox_x->setValue(sphere->position.x());
    ui_->doubleSpinBox_y->setValue(sphere->position.y());
    ui_->doubleSpinBox_z->setValue(sphere->position.z());

    // ✅ DÉBLOQUER les signaux
    ui_->doubleSpinBox_radius->blockSignals(false);
    ui_->doubleSpinBox_x->blockSignals(false);
    ui_->doubleSpinBox_y->blockSignals(false);
    ui_->doubleSpinBox_z->blockSignals(false);

    // ...
  }
}
```

**Avantage**: Les spinboxes sont mises à jour sans déclencher les callbacks.

### 2. Auto-Sélection du Link Parent

```cpp
void CuRoboSetupPanel::onSphereSelected(QTreeWidgetItem* item, int column)
{
  // ...
  if (sphere) {
    // Bloquer signaux et mettre à jour spinboxes...

    // ✅ AUTO-SÉLECTIONNER le link parent de la sphère
    if (sphere->parent_link != selected_link_) {
      selected_link_ = sphere->parent_link;
      ui_->label_selected_link->setText(QString::fromStdString(selected_link_));

      // Trouver et sélectionner le link dans l'arbre
      for (int i = 0; i < ui_->treeWidget_links->topLevelItemCount(); ++i) {
        QTreeWidgetItem* topItem = ui_->treeWidget_links->topLevelItem(i);
        selectLinkInTree(topItem, selected_link_);
      }
    }
  }
}
```

**Avantage**: Le link approprié est automatiquement sélectionné, évitant toute confusion.

### 3. Validation dans les Callbacks de Modification

```cpp
void CuRoboSetupPanel::onPositionChanged(double value)
{
  if (!selected_sphere_id_.empty()) {
    const Sphere* sphere = sphere_manager_->getSphere(selected_sphere_id_);
    if (sphere) {
      // ✅ VÉRIFIER que le link sélectionné correspond au parent de la sphère
      if (sphere->parent_link != selected_link_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot modify sphere '%s': belongs to link '%s' but '%s' is selected",
                    selected_sphere_id_.c_str(), sphere->parent_link.c_str(),
                    selected_link_.c_str());
        return;  // ✅ BLOQUER la modification
      }

      // OK, le link correspond, on peut modifier
      Eigen::Vector3d new_pos(
        ui_->doubleSpinBox_x->value(),
        ui_->doubleSpinBox_y->value(),
        ui_->doubleSpinBox_z->value()
      );
      sphere_manager_->updateSphere(selected_sphere_id_, sphere->radius, new_pos);
      updateSpheresTree();
    }
  }
}
```

**Avantage**: Même si les signaux ne sont pas bloqués, une validation empêche les modifications incorrectes.

### 4. Fonction Helper pour Sélectionner un Link

```cpp
void CuRoboSetupPanel::selectLinkInTree(QTreeWidgetItem* item, const std::string& link_name)
{
  if (!item) return;

  // Vérifier si cet item est le link recherché
  if (item->data(0, Qt::UserRole).toString().toStdString() == link_name) {
    ui_->treeWidget_links->setCurrentItem(item);
    return;
  }

  // Rechercher récursivement dans les enfants
  for (int i = 0; i < item->childCount(); ++i) {
    selectLinkInTree(item->child(i), link_name);
  }
}
```

**Avantage**: Trouve et sélectionne le link dans l'arbre hiérarchique, peu importe sa profondeur.

---

## 📊 Changements de Code

### Fichiers Modifiés

1. **`src/curobo_robot_setup_panel.cpp`**
   - `onSphereSelected()`: Ajout blocage signaux + auto-sélection link
   - `onPositionChanged()`: Ajout validation link parent
   - `onRadiusChanged()`: Ajout validation link parent
   - `selectLinkInTree()`: Nouvelle fonction helper

2. **`include/curobo_robot_setup/curobo_robot_setup_panel.hpp`**
   - Déclaration de `selectLinkInTree()`

### Statistiques

- **Lignes ajoutées**: ~45
- **Lignes modifiées**: ~10
- **Fonctions modifiées**: 3
- **Nouvelles fonctions**: 1

---

## 🧪 Tests de Vérification

### Test 1: Chargement YAML avec Sphères Existantes

**Procédure:**
```
1. Créer un YAML avec des sphères pour link1, link2, link3
2. Lancer RViz2 + plugin
3. Charger l'URDF
4. Charger le YAML
5. Sélectionner link1 dans l'arbre des links
6. Cliquer sur une sphère de link2
```

**Résultat attendu (AVANT le fix):**
```
❌ La sphère de link2 se modifie avec les coordonnées du frame de link1
❌ Les spinboxes montrent la position de la sphère de link2
❌ Confusion totale
```

**Résultat attendu (APRÈS le fix):**
```
✅ link2 est automatiquement sélectionné dans l'arbre des links
✅ Le label "Selected Link" affiche "link2"
✅ Les spinboxes affichent correctement la position dans le frame de link2
✅ Aucune modification non désirée
```

### Test 2: Tentative de Modification Invalide

**Procédure:**
```
1. Charger URDF + YAML
2. Sélectionner link1
3. Modifier manuellement selected_link_ (via débogueur ou autre)
4. Essayer de modifier une sphère de link2
```

**Résultat attendu:**
```
✅ Warning dans les logs ROS:
   "Cannot modify sphere 'sphere_X': belongs to link 'link2' but 'link1' is selected"
✅ Modification bloquée
✅ Sphère reste inchangée
```

### Test 3: Workflow Normal

**Procédure:**
```
1. Charger URDF
2. Sélectionner link1
3. Ajouter une sphère
4. Cliquer sur la sphère créée
5. Modifier radius, X, Y, Z
```

**Résultat attendu:**
```
✅ link1 reste sélectionné
✅ Modifications appliquées correctement
✅ Position relative au frame de link1
```

### Test 4: Navigation entre Sphères de Différents Links

**Procédure:**
```
1. Charger URDF + YAML avec sphères sur plusieurs links
2. Cliquer sur sphère de link1 → Vérifier auto-sélection
3. Cliquer sur sphère de link3 → Vérifier auto-sélection
4. Cliquer sur sphère de link2 → Vérifier auto-sélection
```

**Résultat attendu:**
```
✅ À chaque clic, le bon link est auto-sélectionné
✅ Les spinboxes affichent toujours les bonnes valeurs
✅ Aucune modification non désirée
```

---

## 🔍 Analyse Technique

### Concept: Signal Blocking en Qt

Qt utilise un système de signaux et slots. Quand on modifie un widget (ex: spinbox), il émet un signal (`valueChanged`). D'autres parties du code peuvent se connecter à ce signal pour réagir.

**Problème**: Parfois, on veut modifier un widget programmatiquement sans déclencher les slots connectés.

**Solution**: `blockSignals(true)`

```cpp
widget->blockSignals(true);   // Désactiver les signaux
widget->setValue(newValue);   // Modifier sans émettre valueChanged
widget->blockSignals(false);  // Réactiver les signaux
```

### Pourquoi c'est Important

Dans notre cas:
1. `onSphereSelected` met à jour les spinboxes pour AFFICHER les valeurs
2. On ne veut PAS que cette mise à jour d'affichage MODIFIE la sphère
3. Sans blocage, `setValue()` → `valueChanged` → `onPositionChanged` → modification non désirée
4. Avec blocage, `setValue()` ⛔ (pas de signal) → pas de modification

### Architecture de Protection Multi-Niveaux

Notre correction utilise une approche défensive à 3 niveaux:

**Niveau 1: Prévention**
- Bloquer les signaux lors de la mise à jour d'affichage
- Évite complètement le problème

**Niveau 2: Correction**
- Auto-sélection du link parent
- Assure que le contexte est correct

**Niveau 3: Validation**
- Vérification dans les callbacks de modification
- Dernier rempart contre les modifications incorrectes

Cette approche garantit la robustesse même si un niveau échoue.

---

## 📝 Notes pour les Développeurs

### Quand Utiliser `blockSignals()`

✅ **OUI**: Quand on met à jour un widget pour refléter l'état
```cpp
// Mise à jour d'affichage, pas de modification logique
spinbox->blockSignals(true);
spinbox->setValue(displayValue);
spinbox->blockSignals(false);
```

❌ **NON**: Quand l'utilisateur interagit avec le widget
```cpp
// L'utilisateur clique sur le spinbox et le modifie
// On VEUT que valueChanged soit émis
void onUserEdit() {
  // Pas de blockSignals ici!
  spinbox->setValue(userValue);
}
```

### Pattern Recommandé

```cpp
void updateDisplayFromData(const Data& data) {
  // Pattern: Block → Update → Unblock
  widget->blockSignals(true);
  widget->setValue(data.value);
  widget->blockSignals(false);
}

void onUserChanged(double value) {
  // Pattern: Validate → Apply
  if (!isValidContext()) {
    RCLCPP_WARN(...);
    return;
  }
  data.value = value;
  applyChange();
}
```

---

## 🎓 Leçons Apprises

### 1. Effet de Bord Caché dans Qt

Les signaux Qt peuvent créer des effets de bord inattendus. Toujours se demander:
- "Est-ce que cette modification déclenchera un signal?"
- "Ce signal est-il désiré dans ce contexte?"

### 2. Validation du Contexte

Ne jamais assumer que le contexte est correct. Toujours valider:
- Le link sélectionné correspond-il à la sphère?
- L'utilisateur a-t-il l'intention de modifier?
- Les données sont-elles cohérentes?

### 3. Feedback Visuel

L'auto-sélection du link parent améliore grandement l'UX:
- L'utilisateur voit clairement quel link est actif
- Pas de confusion sur le frame de référence
- Moins d'erreurs de manipulation

---

## ✅ Checklist de Vérification

- [x] Signaux bloqués lors de la mise à jour d'affichage
- [x] Auto-sélection du link parent
- [x] Validation dans onPositionChanged
- [x] Validation dans onRadiusChanged
- [x] Fonction helper selectLinkInTree
- [x] Déclaration dans le header
- [x] Messages de warning dans les logs
- [x] Tests manuels effectués
- [x] Documentation créée

---

## 🚀 Prochaines Améliorations Possibles

1. **Feedback Visuel Amélioré**
   - Surbrillance du link sélectionné dans l'arbre
   - Couleur différente pour les sphères du link actif

2. **Désactivation des Spinboxes**
   - Désactiver les spinboxes si aucune sphère n'est sélectionnée
   - Ou si la sphère n'appartient pas au link sélectionné

3. **Dialog de Confirmation**
   - Si l'utilisateur essaie de modifier une sphère d'un autre link
   - "Cette sphère appartient à link2. Voulez-vous sélectionner link2?"

4. **Indicateur Visuel dans l'Arbre**
   - Icône ou couleur pour montrer quel link est actuellement sélectionné
   - Facilite la navigation

---

## 📚 Références

- **Qt Documentation**: [QObject::blockSignals()](https://doc.qt.io/qt-5/qobject.html#blockSignals)
- **Signal/Slot Pattern**: [Signals & Slots](https://doc.qt.io/qt-5/signalsandslots.html)
- **Code Source**: `src/curobo_robot_setup_panel.cpp:332-456`

---

**Fin du document**
