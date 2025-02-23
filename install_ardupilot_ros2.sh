#!/bin/bash

# Set base directory and workspace
BASE_DIR="$(pwd)"
WORKSPACE_DIR="$BASE_DIR/ardu_ws"

# Set up logging
LOG_FILE="$BASE_DIR/installation_log.txt"
exec 1> >(tee -a "$LOG_FILE") 2>&1

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

# Function to setup environment variables
setup_environment() {
    print_section "Setting up environment variables"
    # ROS2 environment
    export ROS_DISTRO=humble
    export ROS_ROOT=/opt/ros/humble
    
    # Source ROS2 setup files with error checking
    if [ -f "$ROS_ROOT/setup.bash" ]; then
        source "$ROS_ROOT/setup.bash"
        echo "✓ Sourced ROS2 setup.bash"
    else
        echo "✗ Error: ROS2 setup.bash not found"
        exit 1
    fi
    
    # Source workspace setup if it exists
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
        echo "✓ Sourced workspace setup.bash"
    fi
    
    # Update PATH for current session
    if [[ ":$PATH:" != *":$WORKSPACE_DIR/Micro-XRCE-DDS-Gen/scripts:"* ]]; then
        export PATH="$PATH:$WORKSPACE_DIR/Micro-XRCE-DDS-Gen/scripts"
    fi
}

# Prompt for installation options
echo "Please select an option:"
echo "1. Run complete installation (including all builds)"
echo "2. Run only colcon build steps"
read -p "Enter your choice (1 or 2): " choice

case $choice in
    1)
        full_installation=true
        ;;
    2)
        full_installation=false
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

# Function for colcon build with enhanced logging
do_colcon_build() {
    local build_target=$1
    local log_file=$2
    
    print_section "Building $build_target"
    setup_environment
    
    # Run colcon build with enhanced console output
    colcon build --packages-up-to $build_target \
        --event-handlers console_direct+ \
        --executor sequential \
        --symlink-install \
        2>&1 | tee "$log_file"
    
    check_status "Build of $build_target"
}

if [ "$full_installation" = true ]; then

    # Update system
    print_section "Updating System Packages"
    sudo apt-get update && sudo apt-get upgrade -y
    check_status "System update"

    # Setup ArduPilot environment
    print_section "Setting up ArduPilot Environment"
    sudo apt-get install git gitk git-gui -y
    check_status "Git installation"

    # Install ROS2
    print_section "Installing ROS2 Humble"
    # Set up locale
    sudo apt update && sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    check_status "Locale setup"

    # Add repositories and keys
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Install ROS2 packages
    sudo apt update && sudo apt-get upgrade -y
    sudo apt install ros-humble-desktop ros-dev-tools -y
    check_status "ROS2 core installation"

    # Install Python dependencies
    print_section "Installing Python Dependencies"
    sudo apt install -y \
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
    check_status "Python dependencies installation"

    # Set up ROS2 environment
    echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
    setup_environment

    # Set up ArduPilot workspace
    print_section "Setting up ArduPilot Workspace"
    mkdir -p "$WORKSPACE_DIR/src"
    cd "$WORKSPACE_DIR"
    vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
    check_status "Workspace initialization"

    # Initialize rosdep
    sudo apt update
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    check_status "rosdep initialization"

    # Install Java and Micro-XRCE-DDS-Gen
    print_section "Installing Micro-XRCE-DDS-Gen"
    sudo apt install default-jre -y
    cd "$WORKSPACE_DIR"
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
    cd Micro-XRCE-DDS-Gen
    ./gradlew assemble
    check_status "Micro-XRCE-DDS-Gen installation"

    # Add to PATH and make sure it's available immediately
    echo "export PATH=\$PATH:$WORKSPACE_DIR/Micro-XRCE-DDS-Gen/scripts" >> "$HOME/.bashrc"
    export PATH="$PATH:$WORKSPACE_DIR/Micro-XRCE-DDS-Gen/scripts"

    # Verify microxrceddsgen is available
    if command -v microxrceddsgen >/dev/null 2>&1; then
        echo "✓ microxrceddsgen is successfully installed and in PATH"
    else
        echo "✗ Error: microxrceddsgen is not available in PATH"
        echo "Creating symbolic link..."
        sudo ln -sf "$WORKSPACE_DIR/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen" /usr/local/bin/microxrceddsgen
        chmod +x "$WORKSPACE_DIR/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen"
    fi

    print_section "Downloading ardupilot"
    cd "$WORKSPACE_DIR/src/ardupilot"
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    export PATH="$PATH:$HOME/.local/bin"
    ./waf configure --board sitl --enable-dds
    ./waf copter
    check_status "ArduPilot copter install"

    # Build ArduPilot packages
    print_section "Building ArduPilot Packages"
    cd "$WORKSPACE_DIR"
    setup_environment
    colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+ 2>&1 | tee colcon_build_dds.log
    check_status "ArduPilot DDS build"

    # Install Gazebo
    print_section "Installing Gazebo"
    sudo apt-get update
    sudo apt-get install curl lsb-release gnupg -y
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install gz-harmonic -y
    check_status "Gazebo installation"

    # Build SITL
    print_section "Building SITL"
    cd "$WORKSPACE_DIR"
    do_colcon_build "ardupilot_sitl" "colcon_build_sitl.log"
    check_status "SITL build"

    # Set up Gazebo integration
    print_section "Setting up Gazebo Integration"
    cd "$WORKSPACE_DIR"
    vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
    export GZ_VERSION=harmonic
    setup_environment
    sudo apt update
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    check_status "Gazebo integration setup"

    # Final build
    print_section "Final Build"
fi

# Colcon build steps (run regardless of installation choice)
cd "$WORKSPACE_DIR"
setup_environment

# Build ArduPilot packages (to be added later if necessary)
# do_colcon_build "ardupilot_dds_tests" "colcon_build_dds.log"

# Build SITL
# do_colcon_build "ardupilot_sitl" "colcon_build_sitl.log"

# Build Gazebo integration
do_colcon_build "ardupilot_gz_bringup" "colcon_build_gz.log"

sudo apt autoremove -y
print_section "Build Process Complete!"
echo "Please check the following log files for detailed build information:"
echo "- $WORKSPACE_DIR/colcon_build_dds.log"
echo "- $WORKSPACE_DIR/colcon_build_sitl.log"
echo "- $WORKSPACE_DIR/colcon_build_gz.log"
echo "Please source your ~/.bashrc file or restart your terminal"





