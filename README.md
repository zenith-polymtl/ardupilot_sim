# Simulation avec Ardupilot, Gazebo et ROS2

Ce guide explique étape par étape comment installer les programmes requis pour notre projet : **WSL**, **ROS 2 Humble**, **Gazebo**, **Mission Planner** et **Ardupilot**. Des placeholders sont laissés pour insérer des images et des liens vers les sites officiels.


## 🖥️ Installation de WSL (Windows Subsystem for Linux)

### 👉 Étape 1 : Activer WSL
1. **Ouvrir un terminal** (PowerShell) et exécuter la commande suivante :
   ```bash
   wsl --install -d ubuntu-22.04
   wsl --update
   ```

## 🖥️ Installation de ROS2, Gazebo, Ardupilot et SITL pour Ardupilot 

### 👉 Étape 2 : Installation du script dans Ubuntu

   ```bash
   git clone https://github.com/zenith-polymtl/ardupilot_sim.git
   cd ardupilot_sim
   chmod +x install_ardupilot_ros2.sh
   ./install_ardupilot_ros2.sh
   ```

En cas de manque de mémoire lors de l'execution du script (si votre ordinateur n'a que 8 gb de RAM), executé cette commande dans Powershell.
   ```bash
"[wsl2]`nmemory=8GB`nswap=16GB" | Out-File -FilePath "$env:USERPROFILE\.wslconfig" -Encoding UTF8
   ```
## 🖥️ Utilisation de ROS2, Gazebo, Ardupilot et SITL pour Ardupilot 

### 👉 Étape 3 : Lancer une simulation exemple

   ```bash
   cd ardu_ws
   source install/setup.bash
   ros2 launch ardupilot_gz_bringup iris_runway.launch.py
   ```

### 👉 Étape 4 : Connection via Mavproxy

   ```bash
   mavproxy.py --console --map --aircraft test --master=:14550
   ```

### 👉 Exemple de commandes utiles

   Cette commande permet de voir les nodes et topics ROS2 envoyé par Ardupilot et Gazebo
   ```bash
   source ~/ardu_ws/install/setup.bash
   # See the node appear in the ROS graph
   ros2 node list
   # See which topics are exposed by the node
   ros2 node info /ardupilot_dds
   # Echo a topic published from ArduPilot
   ros2 topic echo /ap/geopose/filtered
   ```

### 👉 Exemple de contrôle du drone via Mavproxy

For example, change to GUIDED mode, arm the throttle, and then takeoff:

   ```bash
   mode guided
   arm throttle
   takeoff 40
   ```

Change to CIRCLE mode and set the radius to 2000cm

   ```bash
   rc 3 1500
   mode circle
   param set circle_radius 2000
   ```

When you’re ready to land you can set the mode to RTL (or LAND):

   ```bash
   mode rtl
   ```

### 👉 Exemple de contrôle du drone via Mission Planner sur Windows
1. Aller chercher l'IP de la sortie de la machine virtuel WSL2
   Sur Powershell:
   ```powershell
   ipconfig
   ```
   Vous devez trouver l'address IPv4 pour la carte réseau: Carte Ethernet vEthernet (WSL (Hyper-V firewall)) et prenez-là en note
   
2. Lancer Mavproxy comme à l'étape 4:
   ```bash
   mavproxy.py --console --map --copter test --master=:14550 --out=udp:[ip trouvé à l'étape précédente]:14550
   ```
3. Connectez-vous avec Mission Planner via UDP et le port 14550

   ![Connection à Mission Planner](https://ardupilot.org/dev/_images/MissionPlanner_Connect_UDP.jpg)


### 👉 En cas de problèmes:

Voici de commande utile dans PowerShell avec Bash

En cas où Gazebo et la Simulation ne boot plus
   ```bash
   wsl --shutdown
   ```

En cas d'avoir besoin de réinstaller WSL2 et Ubuntu, utiliser cette commande:

   ```bash
   wsl --unregister Ubuntu-22.04
   ```

## 🔗 Liens Utiles

- 🌐 **ROS 2 Humble Installation** : [Site officiel ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- 🌐 **Docs Ardupilot Development** : [ArduPilot Development Site](https://ardupilot.org/dev/index.html)
- 🌐 **Docs Ardupilot Gazebo** : [ROS 2 with Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- 🌐 **Docs Ardupilot ROS2** : [ROS 2 with SITL](https://ardupilot.org/plane/docs/parameters.html)
- 🌐 **Commandes Mavproxy** : [Mavproxy cheatsheet](https://ardupilot.org/mavproxy/docs/getting_started/cheatsheet.html)
- 🌐 **Liste de tous les paramètres d'ardupilot** : [Complete Parameter List](https://ardupilot.org/dev/docs/ros2-sitl.html)
- 🌐 **Docs Gazebo** : [Gazebo Binary Installation](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- 🌐 **Mission Planner** : [Installing Mission Planner](https://ardupilot.org/planner/docs/mission-planner-installation.html)

---

✨ **Bon courage pour l'installation !** 🚀
