#!/bin/bash
set -e
# This script installs and compiles the entire beam robotics software stack
# Running this script with some parts already installed should be fine
#
# Arguments:
# -y = skips the prompt
# -l = link only, creates all symlinks for the various parts of the project
# -u = unlink only, removes all symlinks


# Specify location of installation scripts
INSTALL_SCRIPTS=$"$HOME/software/beam_install_scripts"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Ensure that Beam install scripts are installed
if [ -d $INSTALL_SCRIPTS ]; then
    echo "Beam install scripts found"
else
    echo "Cloning Beam install scripts into:"
    echo $INSTALL_SCRIPTS
    echo "Make sure they are not install somewhere else."
    if [ ! -d "$HOME/software" ]; then
      mkdir -p "$HOME/software"
    fi
    git clone git@github.com:BEAMRobotics/beam_install_scripts.git $INSTALL_SCRIPTS
    echo "Success"
fi

# Set the repo directory as an environment variable
export REPO_DIR=$(dirname "$SCRIPT_DIR")

# get UBUNTU_CODENAME, ROS_DISTRO, CATKIN_DIR
source $INSTALL_SCRIPTS/identify_environment.bash

: ${SYMLINKS_REPO_DIR:=$REPO_DIR}


main()
{
    install_routine $1
}

install_routine()
{
    sudo -v
    if [ -z "$ARG_NO_MENU" ] && [ -z "$CONTINUOUS_INTEGRATION" ]; then
        menu
    fi

    # source catkin setup script
    source $INSTALL_SCRIPTS/catkin_setup.bash
    
    unlink_routine
    catkin_clean

    # submodule_init

    bash $INSTALL_SCRIPTS/ros_install.bash
    create_catkin_ws

    link_routine
    bash $INSTALL_SCRIPTS/rosdeps_install.bash

    # Import functions to install required dependencies
    source $INSTALL_SCRIPTS/beam_dependencies_install.bash


    # Ensure wget is available
    sudo apt-get install -qq wget  > /dev/null
    # Install dependencies
    #install_ceres
    #install_pcl
    install_geographiclib
    #install_gtsam
    #install_libwave

    if [[ $1 = 'robot' ]]; then
        echo 'Installing drivers for robot'
        cd $( dirname "$REPO_DIR")
        if [ -d 'ros_drivers' ]; then
            echo 'pulling most recent master'
            cd ros_drivers
            git pull origin master
            cd ..
        else
            echo "Cloning Beam install scripts"
            git clone git@github.com:BEAMRobotics/ros_drivers.git
        fi
        bash $INSTALL_SCRIPTS/robot_hardware_install.bash
    fi

    compile

    echo "Beam robotics installation completed. Please open a new terminal to re-source environment variables."
    if [ -z "$CONTINUOUS_INTEGRATION" ]; then
        notify-send "Beam Robotics installation completed"
    fi
}

link_routine()
{
    ln -sfn "$SYMLINKS_REPO_DIR" "$CATKIN_DIR/src"
    echo "Symlink to $SYMLINKS_REPO_DIR created successfully"
}

unlink_routine()
{
    # Need to remove just the symlink for the linked repo
    REPO_BASE_NAME=$(basename "$SYMLINKS_REPO_DIR")
    rm -f "$CATKIN_DIR/src/$REPO_BASE_NAME"
    echo "Symlink $CATKIN_DIR/src/$REPO_BASE_NAME removed successfully"
}

menu()
{
    echo "Running this script will delete your /build /devel and /logs folders in your $CATKIN_DIR directory and re-build them."
    echo "Also, this script assumes the following things:"
    echo "  - Your ROS version is $ROS_DISTRO"
    echo "  - Your catkin workspace is located at: $CATKIN_DIR"
    echo "  - Catkin tools is installed"
    echo "  - Your bashrc sources $CATKIN_DIR/devel/setup.bash"
    echo "If any of the above assumptions are not true, the following script will make them so."
    echo "Do you wish to continue? (y/n):"

    while read ans; do
        case "$ans" in
            y) break;;
            n) exit; break;;
            *) echo "(y/n):";;
        esac
    done
}

catkin_clean()
{
    rm -rf "$CATKIN_DIR/devel"
    rm -rf "$CATKIN_DIR/build"
    rm -rf "$CATKIN_DIR/install"
    rm -rf "$CATKIN_DIR/logs"
    rm -f "$CATKIN_DIR/.catkin_workspace"
    echo "Catkin workspace cleaned"
}

submodule_init()
{
    git submodule -q update --init --recursive
}

compile()
{
    cd "$CATKIN_DIR"
    source /opt/ros/$ROS_DISTRO/setup.bash
    if [ -z "$CONTINUOUS_INTEGRATION" ]; then
        catkin build
    else
        if [ -n "$CIRCLECI" ]; then
            # Build libwave by itself first, since the job is so large
            catkin build --no-status -j2 libwave
            catkin build --no-status --mem-limit 6G
        else
            catkin build --no-status
        fi
    fi
}

env_setup()
{
    # ROS environment setup
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source /opt/ros/kinetic/setup.bash
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    echo "ROS_PACKAGE_PATH=/home/$USER/catkin_ws/src:/opt/ros/kinetic/share:/$ROS_PACKAGE_PATH" >> ~/.bashrc
}


main $1
