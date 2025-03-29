#!/bin/bash

# Exit on error (but we'll handle specific command errors manually)
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

# Function to check command success - now checks actual exit code
check_status() {
    local exit_code=$1
    local message=$2
    
    if [ $exit_code -eq 0 ]; then
        echo "✓ Success: $message"
        return 0
    else
        echo "✗ Error: $message failed (exit code: $exit_code)"
        return 1
    fi
}

# Function to run a command and check its status
run_and_check() {
    local command_description=$1
    shift # Remove the first argument
    
    echo "Running: $command_description"
    "$@"
    local exit_code=$?
    
    check_status $exit_code "$command_description"
    return $exit_code
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
# Check if they're already there before adding
if ! grep -q "export ROS_DISTRO=humble" ~/.bashrc; then
    echo "export ROS_DISTRO=humble" >> ~/.bashrc
fi
if ! grep -q "export ROS_ROOT=/opt/ros/humble" ~/.bashrc; then
    echo "export ROS_ROOT=/opt/ros/humble" >> ~/.bashrc
fi
if ! grep -q "export LANG=en_US.UTF-8" ~/.bashrc; then
    echo "export LANG=en_US.UTF-8" >> ~/.bashrc
fi
if ! grep -q "export GZ_VERSION=harmonic" ~/.bashrc; then
    echo "export GZ_VERSION=harmonic" >> ~/.bashrc
fi

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
check_status $? "Basic tools installation"

# Set up locale
print_section "Setting Up Locale"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
check_status $? "Locale setup"

# Clean up duplicate ROS2 repository entries
print_section "Cleaning Up Duplicate APT Sources"
if [ -f /etc/apt/sources.list.d/ros2-latest.list ] && [ -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "Detected duplicate ROS2 repository entries. Cleaning up..."
    sudo rm -f /etc/apt/sources.list.d/ros2-latest.list
    echo "Removed /etc/apt/sources.list.d/ros2-latest.list"
fi

# Add ROS2 repository (only if it doesn't exist)
print_section "Adding ROS2 Repository"
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo apt-get update && sudo apt-get install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    check_status $? "ROS2 repository setup"
else
    echo "ROS2 repository already configured in /etc/apt/sources.list.d/ros2.list"
fi

# Add Gazebo repository (only if it doesn't exist)
print_section "Adding Gazebo Repository"
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    check_status $? "Gazebo repository setup"
else
    echo "Gazebo repository already configured in /etc/apt/sources.list.d/gazebo-stable.list"
fi

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
check_status $? "ROS2 Humble installation"

# Install Gazebo Harmonic
print_section "Installing Gazebo Harmonic"
sudo apt-get update && sudo apt-get install -y gz-harmonic
check_status $? "Gazebo Harmonic installation"

# Create workspace directory
print_section "Creating Workspace"
mkdir -p ${WORKSPACE_DIR}/src
check_status $? "Workspace creation"

# Set up Java and Micro-XRCE-DDS-Gen
print_section "Setting Up Java and Micro-XRCE-DDS-Gen"
sudo apt-get update && sudo apt-get install -y default-jre
cd ${WORKSPACE_DIR}

# Check if Micro-XRCE-DDS-Gen already exists
if [ -d "Micro-XRCE-DDS-Gen" ]; then
    echo "Micro-XRCE-DDS-Gen directory already exists."
    echo "Options:"
    echo "  1. Use existing directory"
    echo "  2. Remove and recreate"
    read -p "Enter choice (1/2): " micro_choice
    
    if [ "$micro_choice" = "2" ]; then
        echo "Removing existing Micro-XRCE-DDS-Gen directory..."
        rm -rf Micro-XRCE-DDS-Gen
        git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
        cd Micro-XRCE-DDS-Gen
        ./gradlew assemble
        micro_status=$?
    else
        echo "Using existing Micro-XRCE-DDS-Gen directory..."
        cd Micro-XRCE-DDS-Gen
        # Pull latest changes
        git pull
        # Make sure submodules are up to date
        git submodule update --init --recursive
        # Rebuild
        ./gradlew assemble
        micro_status=$?
    fi
else
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
    cd Micro-XRCE-DDS-Gen
    ./gradlew assemble
    micro_status=$?
fi
check_status $micro_status "Micro-XRCE-DDS-Gen installation"

# Add Micro-XRCE-DDS-Gen to PATH if not already there
if ! grep -q "export PATH=\$PATH:${WORKSPACE_DIR}/Micro-XRCE-DDS-Gen/scripts" ~/.bashrc; then
    echo "export PATH=\$PATH:${WORKSPACE_DIR}/Micro-XRCE-DDS-Gen/scripts" >> ~/.bashrc
fi
export PATH="${PATH}:${WORKSPACE_DIR}/Micro-XRCE-DDS-Gen/scripts"

# Set up ArduPilot workspace
print_section "Setting Up ArduPilot Workspace"
cd ${WORKSPACE_DIR}
sudo apt-get update
sudo apt-get install -y python3-vcstool

# Check if we already have the ArduPilot repository
if [ -d "src/ardupilot" ]; then
    echo "ArduPilot repository already exists."
    echo "Options:"
    echo "  1. Use existing repository"
    echo "  2. Update existing repository"
    echo "  3. Remove and recreate"
    read -p "Enter choice (1/2/3): " ardu_choice
    
    if [ "$ardu_choice" = "3" ]; then
        echo "Removing existing ArduPilot workspace..."
        rm -rf src
        mkdir -p src
        vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
        ardu_status=$?
    elif [ "$ardu_choice" = "2" ]; then
        echo "Updating existing ArduPilot workspace..."
        cd src
        vcs pull
        ardu_status=$?
        cd ..
    else
        echo "Using existing ArduPilot workspace..."
        ardu_status=0
    fi
else
    vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
    ardu_status=$?
fi
check_status $ardu_status "ArduPilot workspace setup"

# Initialize and update rosdep
print_section "Setting Up rosdep"
sudo apt-get install -y python3-rosdep
if ! rosdep --version > /dev/null 2>&1; then
    sudo rosdep init || true  # May fail if already initialized
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y
check_status $? "rosdep setup"

# Clone and build ArduPilot
print_section "Building ArduPilot"
cd ${WORKSPACE_DIR}/src/ardupilot

# Check if the waf script has already been configured
if [ ! -f ".lock-waf_linux_build" ]; then
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y
    export PATH="$PATH:$HOME/.local/bin"
    ./waf configure --board sitl --enable-dds
    ./waf copter
    ardu_build_status=$?
else
    echo "ArduPilot already configured. Running build only."
    export PATH="$PATH:$HOME/.local/bin"
    ./waf copter
    ardu_build_status=$?
fi
check_status $ardu_build_status "ArduPilot build"

# Import Gazebo integration
print_section "Setting Up Gazebo Integration"
cd ${WORKSPACE_DIR}

# Check if Gazebo integration is already imported


rosdep update

check_status $gz_status "Gazebo integration setup"

# Source ROS2 environment and build packages


cd ${WORKSPACE_DIR}

print_section "Installing Gazebo"
sudo apt-get update
sudo apt-get install curl lsb-release gnupg -y
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic -y
rosdep install --from-paths src --ignore-src -r -y
# Build packages with proper exit code checking 
echo "Building ardupilot_dds_tests..."
colcon build --packages-up-to ardupilot_dds_tests
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src --force
print_section "Building ROS2 Packages"
build_dds_status=$?
# Note stderr output is common and acceptable
if [ $build_dds_status -eq 0 ]; then
    echo "✓ Success: ArduPilot DDS tests build succeeded (stderr warnings are normal)"
else
    echo "✗ Error: ArduPilot DDS tests build failed with exit code $build_dds_status"
fi


# Add sourcing to .bashrc if not already there
print_section "Finalizing Setup"
if ! grep -q "source ${ROS_ROOT}/setup.bash" ~/.bashrc; then
    echo "source ${ROS_ROOT}/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ${WORKSPACE_DIR}/install/setup.bash" ~/.bashrc; then
    echo "source ${WORKSPACE_DIR}/install/setup.bash" >> ~/.bashrc
fi

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
