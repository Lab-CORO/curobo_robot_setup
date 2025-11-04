# Guide de Démarrage Rapide - cuRobo Robot Setup

Ce guide vous permet de démarrer rapidement avec le plugin cuRobo Robot Setup pour RViz2.

## 📋 Prérequis

- ROS 2 Humble ou plus récent
- RViz2 installé
- Un fichier URDF de votre robot

## 🚀 Installation Rapide

### 1. Installer les dépendances

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    qtbase5-dev \
    libyaml-cpp-dev
```

### 2. Cloner et compiler

```bash
# Créer un workspace ROS 2 si nécessaire
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Cloner le dépôt
git clone https://github.com/Lab-CORO/curobo_robot_setup.git

# Compiler
cd ~/ros2_ws
colcon build --packages-select curobo_robot_setup

# Sourcer le workspace
source ~/ros2_ws/install/setup.bash
```

## 🎯 Utilisation Rapide - Méthode 1: Avec Launch File

### Lancer avec votre URDF

```bash
ros2 launch curobo_robot_setup robot_setup.launch.py urdf_file:=/chemin/vers/votre/robot.urdf
```

Cela lancera automatiquement:
- RViz2 avec configuration par défaut
- Robot State Publisher pour publier les TF
- Joint State Publisher GUI pour contrôler les joints
- Le plugin cuRobo Robot Setup

### Exemple avec le robot de démonstration

```bash
# Avec le fichier d'exemple inclus
ros2 launch curobo_robot_setup robot_setup.launch.py \
    urdf_file:=$(ros2 pkg prefix curobo_robot_setup)/share/curobo_robot_setup/example/m1013.urdf
```

## 🎯 Utilisation Rapide - Méthode 2: Manuel

### 1. Lancer RViz2

```bash
rviz2
```

### 2. Ajouter le plugin

1. Cliquez sur **Panels** → **Add New Panel**
2. Sélectionnez **curobo_robot_setup/CuRoboSetupPanel**
3. Le panel apparaît à gauche

### 3. Configurer RViz2 pour la visualisation

#### a) Ajouter RobotModel display

1. Cliquez sur **Add** (en bas du panneau Displays)
2. Sélectionnez **RobotModel**
3. Dans les propriétés de RobotModel:
   - **Description Topic**: `/robot_description`
   - **Fixed Frame**: Choisir le link de base de votre robot (ex: `base_link`)

#### b) Ajouter MarkerArray pour les sphères

1. Cliquez sur **Add**
2. Sélectionnez **MarkerArray**
3. Dans les propriétés:
   - **Topic**: `/curobo_collision_spheres`

### 4. Charger votre URDF

1. Dans le plugin, allez dans l'onglet **URDF Loader**
2. Cliquez sur **Load URDF**
3. Sélectionnez votre fichier `.urdf`
4. Le robot devrait apparaître dans RViz2

## 📝 Workflow en 5 Minutes

### Étape 1: Charger l'URDF (30 secondes)
```
URDF Loader → Load URDF → Sélectionner fichier → OK
```

### Étape 2: Ajouter des sphères de collision (2 minutes)

1. Onglet **Sphere Editor**
2. Cliquer sur un link dans l'arbre (ex: "link1")
3. Ajuster le rayon (ex: 0.08 m)
4. Ajuster la position (X, Y, Z)
5. Cliquer **Add Sphere**
6. Répéter pour couvrir tout le link
7. Répéter pour tous les links importants

**Astuce:** Les sphères doivent se chevaucher légèrement (10-20%)

### Étape 3: Configurer les joints (1 minute)

1. Onglet **Configuration**
2. Cliquer **Configure Joints**
3. Vérifier que les limites sont correctes
4. Cliquer **Select All** pour activer tous les joints
5. Cliquer **OK**

### Étape 4: Définir base/end-effector (30 secondes)

1. Onglet **Configuration**
2. **Base Link**: Sélectionner le link racine (ex: `base_link`)
3. **End Effector Link**: Sélectionner le link final (ex: `tool0`, `ee_link`)

### Étape 5: Sauvegarder (30 secondes)

1. Cliquer **Save YAML**
2. Choisir l'emplacement
3. Nommer le fichier (ex: `mon_robot_curobo.yml`)
4. Sauvegarder

✅ **Terminé!** Vous avez maintenant une configuration cuRobo prête à l'emploi.

## 🧪 Tester avec l'Exemple

Essayez avec le robot d'exemple fourni:

### Option 1: Via launch file
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch curobo_robot_setup robot_setup.launch.py \
    urdf_file:=$(ros2 pkg prefix curobo_robot_setup)/share/curobo_robot_setup/example/m1013.urdf
```

### Option 2: Charger manuellement
1. Lancez RViz2 et ajoutez le plugin
2. Chargez `example/m1013.urdf`
3. Chargez `example/m1013.yml` pour voir une configuration complète

## 🎨 Personnalisation de la Vue RViz2

### Configuration recommandée

1. **Fixed Frame**: `base_link` (ou le link racine de votre robot)
2. **Background Color**: Gris clair pour meilleur contraste
3. **Grid**: Activé, 10 cellules, 1m de taille

### Sauvegarder votre configuration RViz

```
File → Save Config As → ~/mon_robot_rviz_config.rviz
```

Pour réutiliser:
```bash
rviz2 -d ~/mon_robot_rviz_config.rviz
```

## 🔧 Dépannage Rapide

### Le robot n'apparaît pas dans RViz2

**Solution:**
1. Vérifiez que RobotModel est ajouté aux Displays
2. Vérifiez que le Fixed Frame correspond au base_link de votre robot
3. Dans RobotModel, vérifiez que Description Topic = `/robot_description`
4. Regardez les logs dans le terminal pour les erreurs

### Les sphères ne sont pas visibles

**Solution:**
1. Ajoutez un display MarkerArray
2. Topic: `/curobo_collision_spheres`
3. Vérifiez que vous avez ajouté au moins une sphère

### Erreur "Failed to parse URDF"

**Solution:**
1. Vérifiez la syntaxe URDF: `check_urdf votre_robot.urdf`
2. Assurez-vous que tous les fichiers mesh sont accessibles
3. Vérifiez les chemins relatifs dans votre URDF

### Le plugin n'apparaît pas dans RViz2

**Solution:**
```bash
# Re-compiler
cd ~/ros2_ws
colcon build --packages-select curobo_robot_setup --symlink-install

# Re-sourcer
source ~/ros2_ws/install/setup.bash

# Vérifier l'installation
ros2 pkg list | grep curobo_robot_setup

# Lancer RViz2
rviz2
```

## 📚 Prochaines Étapes

### Pour aller plus loin:

1. **Tutorial complet**: Consultez [TUTORIAL.md](TUTORIAL.md) pour une utilisation avancée
2. **Documentation**: Lisez [README.md](README.md) pour tous les détails
3. **Bugs connus**: Voir [BUGS_REPORT.md](BUGS_REPORT.md) pour les problèmes identifiés

### Utiliser avec cuRobo

Une fois la configuration créée:

```python
from curobo.wrap.reacher.motion_gen import MotionGen

# Charger votre configuration
motion_gen = MotionGen.from_robot_config(
    robot_cfg_file="mon_robot_curobo.yml"
)

# Planifier un mouvement
result = motion_gen.plan_single(goal_pose)
```

## 💡 Conseils Pro

1. **Commencez simple**: Configurez d'abord un seul link avec quelques sphères
2. **Testez souvent**: Sauvegardez et chargez régulièrement pour vérifier
3. **Utilisez des noms descriptifs**: Nommez vos fichiers de manière claire
4. **Documentez**: Ajoutez des commentaires sur vos choix de configuration
5. **Versionnez**: Gardez plusieurs versions de vos configurations

## 🤝 Besoin d'Aide?

- **Issues GitHub**: https://github.com/Lab-CORO/curobo_robot_setup/issues
- **Documentation cuRobo**: https://github.com/NVlabs/curobo
- **ROS 2 Docs**: https://docs.ros.org/

---

**Bon démarrage avec cuRobo Robot Setup! 🚀**
