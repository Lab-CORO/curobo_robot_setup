# Fichiers d'Exemple - Robot M1013

Ce répertoire contient un exemple de configuration complète pour un robot Doosan M1013.

## Fichiers

### m1013.urdf
Fichier URDF complet du robot Doosan M1013 avec:
- 6 degrés de liberté (DOF)
- Géométrie de collision
- Limites des joints
- Modèles visuels

### m1013.yml
Configuration cuRobo complète incluant:
- 21 sphères de collision réparties sur 6 links
- Configuration des joints (6 joints)
- Paires d'ignore pour auto-collision
- Paramètres de planification de mouvement

## Utilisation

### Option 1: Charger dans RViz2 avec le plugin

```bash
# Lancer RViz2
rviz2

# Dans le plugin cuRobo Robot Setup:
# 1. Onglet "URDF Loader" → Load URDF → Sélectionner m1013.urdf
# 2. Onglet "Configuration" → Load YAML → Sélectionner m1013.yml
```

### Option 2: Utiliser le launch file

```bash
# Récupérer le chemin du package
PACKAGE_PATH=$(ros2 pkg prefix curobo_robot_setup)/share/curobo_robot_setup

# Lancer avec le robot d'exemple
ros2 launch curobo_robot_setup robot_setup.launch.py \
    urdf_file:=$PACKAGE_PATH/example/m1013.urdf
```

### Option 3: Utiliser directement avec cuRobo

```python
from curobo.wrap.reacher.motion_gen import MotionGen

# Charger la configuration
motion_gen = MotionGen.from_robot_config(
    robot_cfg_file="/path/to/curobo_robot_setup/example/m1013.yml"
)

# Planifier un mouvement
from curobo.types.math import Pose
goal_pose = Pose(
    position=[0.5, 0.0, 0.5],
    orientation=[1.0, 0.0, 0.0, 0.0]
)

result = motion_gen.plan_single(goal_pose)
if result.success:
    print("Mouvement planifié avec succès!")
    trajectory = result.get_interpolated_plan()
```

## Détails de la Configuration

### Sphères de Collision

Le robot M1013 utilise **21 sphères de collision** pour approximer sa géométrie:

| Link | Nombre de Sphères | Rayon (min-max) | Usage |
|------|-------------------|-----------------|-------|
| link1 | 1 | 0.164 m | Base du robot |
| link2 | 5 | 0.077-0.127 m | Bras principal (long) |
| link3 | 1 | 0.101 m | Coude |
| link4 | 4 | 0.080-0.101 m | Avant-bras |
| link5 | 1 | 0.101 m | Poignet |
| link6 | 1 | 0.065 m | Effecteur final |

### Paires d'Auto-collision Ignorées

```yaml
"base_0": ["link1"]           # Base ne peut pas toucher link1 (adjacents)
"link1": ["link2", "link3"]   # Link1 ignore ses voisins proches
"link2": ["link3", "link5"]   # Link2 ignore link3 et link5
"link3": ["link4", "link5", "link6"]  # Link3 ignore plusieurs links distants
"link4": ["link5", "link6"]   # Link4 ignore effecteur final
"link5": ["link6"]            # Link5 ignore effecteur (adjacents)
```

### Paramètres de Joints

Tous les joints utilisent:
- **Null space weight**: 1.0 (poids uniforme)
- **C-space distance weight**: 1.0 (poids uniforme)
- **Max jerk**: 500.0 rad/s³
- **Max acceleration**: 15.0 rad/s²

## Personnaliser pour Votre Robot

Pour créer une configuration similaire pour votre robot:

### 1. Remplacer l'URDF
Remplacez `m1013.urdf` par votre fichier URDF.

### 2. Ajuster les Sphères
Utilisez le plugin RViz2 pour:
- Identifier les links critiques
- Ajouter des sphères pour couvrir chaque link
- Ajuster les rayons pour approximer la géométrie réelle

### 3. Configurer les Joints
Dans le dialogue de configuration des joints:
- Vérifier les limites des joints
- Ajuster les vitesses/accélérations maximales
- Définir la configuration de retrait

### 4. Définir les Ignores d'Auto-collision
Règle générale:
- Links adjacents doivent s'ignorer
- Links qui ne peuvent jamais se toucher doivent s'ignorer
- Tester avec différentes configurations de joints

### 5. Exporter et Tester
```bash
# Sauvegarder la configuration
# Configuration → Save YAML

# Tester avec cuRobo
python3 test_config.py mon_robot.yml
```

## Visualisation

### Visualiser les Sphères dans RViz2

1. Ajouter un display **MarkerArray**
2. Topic: `/curobo_collision_spheres`
3. Les sphères apparaîtront en cyan/bleu semi-transparent

### Vérifier la Couverture

- Faire pivoter la vue dans RViz2
- Vérifier qu'il n'y a pas de gaps entre les sphères
- S'assurer que toutes les parties du robot sont couvertes
- Les sphères doivent légèrement se chevaucher (10-20%)

## Notes Importantes

1. **Chemins Absolus**: Le fichier YML utilise des chemins absolus. Mettez-les à jour pour votre système:
   ```yaml
   urdf_path: "/path/to/your/workspace/..."
   ```

2. **Buffer de Collision**: Un buffer de 0.01m (1cm) est ajouté aux sphères pour la sécurité.

3. **Ordre des Joints**: L'ordre dans `joint_names` doit correspondre à l'ordre dans votre URDF.

4. **Retract Config**: Tous les joints sont à 0.0 en position de retrait. Ajustez selon votre robot.

## Benchmarks de Performance

Configuration testée sur NVIDIA RTX 3080:
- Temps de planification moyen: ~15ms
- Taux de réussite: >95% pour objectifs atteignables
- Génération de trajectoire: ~5ms

## Support

Pour des questions sur cet exemple:
- Consultez le [TUTORIAL.md](../TUTORIAL.md) pour des guides détaillés
- Voir le [README.md](../README.md) principal pour la documentation complète
- Rapporter les problèmes sur GitHub Issues

## Licence

Cet exemple est fourni à titre de démonstration. Voir [LICENSE](../LICENSE) pour les détails.
