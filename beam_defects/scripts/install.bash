#!/bin/bash
set -e
# This script installs and compiles the entire beam robotics software stack
# Running this script with some parts already installed should be fine

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

main()
{
    install_routine $1
}

install_routine()
{
    sudo -v

    # Import functions to install required dependencies
    source $INSTALL_SCRIPTS/beam_dependencies_install.bash

    sudo apt-get update
    # Ensure wget is available
    sudo apt-get install -qq wget > /dev/null

    # Install dependencies
    #install_ceres
    install_pcl
    install_catch2
    # Compile code
    compileCPP

    echo "Beam defects installation completed."
    if [ -z "$CONTINUOUS_INTEGRATION" ]; then
        notify-send "Beam Robotics installation completed"
    fi
}

compileCPP()
{
  cd $REPO_DIR
  cmake .
  make -j99
}

main $1
