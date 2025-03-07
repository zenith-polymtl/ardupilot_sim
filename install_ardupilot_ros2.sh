#!/bin/bash

# Exit on error
set -e

# Set base directory and workspace
BASE_DIR="$(pwd)"
WORKSPACE_DIR="$BASE_DIR/ardu_ws"
LOG_FILE="$BASE_DIR/installation_log.txt"

# Set up logging
exec > >(tee -a "$LOG_FILE") 2>&1

# Function to print section headers
print_section() {
    echo "===================================================="
    echo ">>> $1"
    echo "===================================================="
}

# Function to check command success
check_status() {
    if [ $? -eq 0 ]; then
        echo "✓ Success: $1"
    else
        echo "✗ Error: $1 failed"
        exit 1
    fi
}

# WSL-specific checks and setup
print_section "Checking WSL Environment"
if grep -q Microsoft /proc/version; then
    echo "WSL detected. Performing WSL-specific setup..."
    
    # Check for X server
    echo "Note: For GUI applications in WSL, you need an X server like VcXsrv running on Windows."
    echo "Please make sure you have one installed and running."
    
    # Set DISPLAY if not already set
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0
        echo "export DISPLAY=:0" >> ~/.bashrc
    fi
    
    # WSL2-specific network settings if needed
    if grep -q "WSL2" /proc/version; then
        echo "WSL2 detected. Setting additional configurations..."
        # Add any WSL2-specific configuration here if needed
    fi
fi

# Prevent interactive prompts during installation
export DEBIAN_FRONTEND=noninteractive

# Set environment variables
print_section "Setting Environment Variables"
export ROS_DISTRO=humble
export ROS_ROOT=/opt/ros/humble
export LANG=en_US.UTF-8
export PYTHONUNBUFFERED=1
export GZ_VERSION=harmonic

# Add these to .bashrc for persistence
echo "export ROS_DISTRO=humble" >> ~/.bashrc
echo "export ROS_ROOT=/opt/ros/humble" >> ~/.bashrc
echo "export LANG=en_US.UTF-8" >> ~/.bashrc
echo "export GZ_VERSION=harmonic" >> ~/.bashrc

# Install basic tools and dependencies
print_section "Installing Basic Tools and Dependencies"
sudo apt-get update
sudo apt-get install -y \
    curl \
    git \
    gnupg \
    lsb-release \
    locales \
    software-properties-common \
    sudo \
    wget
check_status "Basic tools installation"

# Set up locale
print_section "Setting Up Locale"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
check_status "Locale setup"

# Add ROS2 repository
print_section "Adding ROS2 Repository"
sudo apt-get update && sudo apt-get install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
check_status "ROS2 repository setup"

# Add Gazebo repository
print_section "Adding Gazebo Repository"
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
check_status "Gazebo repository setup"

# Install ROS2 Humble
print_section "Installing ROS2 Humble"
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures
check_status "ROS2 Humble installation"

# Install Gazebo Harmonic
print_section "Installing Gazebo Harmonic"
sudo apt-get update && sudo apt-get install -y gz-harmonic
check_status "Gazebo Harmonic installation"

# Create workspace directory
print_section "Creating Workspace"
mkdir -p ${WORKSPACE_DIR}/src
check_status "Workspace creation"

# Set up Java and Micro-XRCE-DDS-Gen
print_section "Setting Up Java and Micro-XRCE-DDS-Gen"
sudo apt-get update && sudo apt-get install -y default-jre
cd ${WORKSPACE_DIR}
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
check_status "Micro-XRCE-DDS-Gen installation"

# Add Micro-XRCE-DDS-Gen to PATH
echo "export PATH=\$PATH:${WORKSPACE_DIR}/Micro-XRCE-DDS-Gen/scripts" >> ~/.bashrc
export PATH="${PATH}:${WORKSPACE_DIR}/Micro-XRCE-DDS-Gen/scripts"

# Set up ArduPilot workspace
print_section "Setting Up ArduPilot Workspace"
cd ${WORKSPACE_DIR}
sudo apt-get update
sudo apt-get install -y python3-vcstool
vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
check_status "ArduPilot workspace setup"

# Initialize and update rosdep
print_section "Setting Up rosdep"
sudo apt-get install -y python3-rosdep
sudo rosdep init || true  # May fail if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
check_status "rosdep setup"

# Clone and build ArduPilot
print_section "Building ArduPilot"
cd ${WORKSPACE_DIR}/src/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
export PATH="$PATH:$HOME/.local/bin"
./waf configure --board sitl --enable-dds
./waf copter
check_status "ArduPilot build"

# Import Gazebo integration
print_section "Setting Up Gazebo Integration"
cd ${WORKSPACE_DIR}
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
rosdep update
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
check_status "Gazebo integration setup"

# Source ROS2 environment and build packages
print_section "Building ROS2 Packages"
source ${ROS_ROOT}/setup.bash
cd ${WORKSPACE_DIR}
colcon build --packages-up-to ardupilot_dds_tests
check_status "ArduPilot DDS tests build"

colcon build --packages-up-to ardupilot_sitl
check_status "ArduPilot SITL build"

colcon build --packages-up-to ardupilot_gz_bringup
check_status "ArduPilot Gazebo bringup build"

# Add sourcing to .bashrc
print_section "Finalizing Setup"
echo "source ${ROS_ROOT}/setup.bash" >> ~/.bashrc
echo "source ${WORKSPACE_DIR}/install/setup.bash" >> ~/.bashrc

# Create launch script
cat > ${WORKSPACE_DIR}/run_simulation.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source $(pwd)/install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
EOF
chmod +x ${WORKSPACE_DIR}/run_simulation.sh

print_section "Installation Complete!"
echo "To run the simulation, execute the following commands:"
echo "  cd ${WORKSPACE_DIR}"
echo "  ./run_simulation.sh"
echo ""
echo "Or you can manually run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo "  ros2 launch ardupilot_gz_bringup iris_runway.launch.py"
echo ""
echo "Note for WSL users: Make sure your X server is running on Windows for GUI applications."
echo "Installation log is available at: ${LOG_FILE}"
