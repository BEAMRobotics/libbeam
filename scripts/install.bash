#!/bin/bash
set -e
# This script installs and compiles libbeam
# Running this script with some parts already installed should be fine

# Get important directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRC_DIR="${SCRIPT_DIR//'/libbeam/scripts'}"

# set global variables
INSTALL_LIBBEAM_LOCALLY=0
INSTALL_OPENCV4=1
INSTALL_OPENCV4_LOCALLY=0
INSTALL_CERES=1

IGNORE_BEAM_CALIBRATION=0
IGNORE_BEAM_COLORIZE=0
IGNORE_BEAM_CONTAINERS=0
IGNORE_BEAM_CV=0
IGNORE_BEAM_DEFECTS=0
IGNORE_BEAM_DEPTH=0
IGNORE_BEAM_FILTERING=0
IGNORE_BEAM_MAPPING=0
IGNORE_BEAM_MATCHING=0
IGNORE_BEAM_OPTIMIZATION=0
IGNORE_BEAM_UTILS=0

main()
{
    set_inputs $@
    menu
    install_routine 
}

set_inputs()
{
    for ARGUMENT in "$@"
    do
        KEY=$(echo $ARGUMENT | cut -f1 -d=)
        VALUE=$(echo $ARGUMENT | cut -f2 -d=)   

        case "$KEY" in
                INSTALL_LIBBEAM_LOCALLY) INSTALL_LIBBEAM_LOCALLY=${VALUE} ;;
                INSTALL_OPENCV4) INSTALL_OPENCV4=${VALUE} ;;
                INSTALL_OPENCV4_LOCALLY) INSTALL_OPENCV4_LOCALLY=${VALUE} ;;
                INSTALL_CERES) INSTALL_CERES=${VALUE} ;;
                IGNORE_BEAM_CALIBRATION) IGNORE_BEAM_CALIBRATION=${VALUE} ;;
                IGNORE_BEAM_COLORIZE) IGNORE_BEAM_COLORIZE=${VALUE} ;;
                IGNORE_BEAM_CONTAINERS) IGNORE_BEAM_CONTAINERS=${VALUE} ;;
                IGNORE_BEAM_CV) IGNORE_BEAM_CV=${VALUE} ;;
                IGNORE_BEAM_DEFECTS) IGNORE_BEAM_DEFECTS=${VALUE} ;;
                IGNORE_BEAM_DEPTH) IGNORE_BEAM_DEPTH=${VALUE} ;;
                IGNORE_BEAM_FILTERING) IGNORE_BEAM_FILTERING=${VALUE} ;;
                IGNORE_BEAM_MAPPING) IGNORE_BEAM_MAPPING=${VALUE} ;;
                IGNORE_BEAM_MATCHING) IGNORE_BEAM_MATCHING=${VALUE} ;;
                IGNORE_BEAM_OPTIMIZATION) IGNORE_BEAM_OPTIMIZATION=${VALUE} ;;
                INSTALL_OPENCV4) IGNORE_BEAM_UTILS=${VALUE} ;;
                *) echo "INVALID ARGUMENT.";   
        esac    
    done

    if(( $INSTALL_LIBBEAM_LOCALLY != 1 && $INSTALL_LIBBEAM_LOCALLY != 0 ))
    then
        echo "Invalid INSTALL_LIBBEAM_LOCALLY parameter. Required: 1 or 0. Exiting"
        exit
    fi

    if(( $INSTALL_OPENCV4 != 1 && $INSTALL_OPENCV4 != 0 ))
    then
        echo "Invalid INSTALL_OPENCV4 parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $INSTALL_CERES != 1 && $INSTALL_CERES != 0 ))
    then
        echo "Invalid INSTALL_CERES parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $INSTALL_OPENCV4_LOCALLY != 1 && $INSTALL_OPENCV4_LOCALLY != 0 ))
    then
        echo "Invalid INSTALL_OPENCV4_LOCALLY parameter. Required: 1 or 0. Exiting"
        exit
    fi

    if(( $IGNORE_BEAM_CALIBRATION != 1 && $IGNORE_BEAM_CALIBRATION != 0 ))
    then
        echo "Invalid IGNORE_BEAM_CALIBRATION parameter. Required: 1 or 0. Exiting"
        exit       
    fi
    
    if(( $IGNORE_BEAM_COLORIZE != 1 && $IGNORE_BEAM_COLORIZE != 0 ))
    then
        echo "Invalid IGNORE_BEAM_COLORIZE parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_CONTAINERS != 1 && $IGNORE_BEAM_CONTAINERS != 0 ))
    then
        echo "Invalid IGNORE_BEAM_CONTAINERS parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_CV != 1 && $IGNORE_BEAM_CV != 0 ))
    then
        echo "Invalid IGNORE_BEAM_CV parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_DEFECTS != 1 && $IGNORE_BEAM_DEFECTS != 0 ))
    then
        echo "Invalid IGNORE_BEAM_DEFECTS parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_DEPTH != 1 && $IGNORE_BEAM_DEPTH != 0 ))
    then
        echo "Invalid IGNORE_BEAM_DEPTH parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_FILTERING != 1 && $IGNORE_BEAM_FILTERING != 0 ))
    then
        echo "Invalid IGNORE_BEAM_FILTERING parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_MAPPING != 1 && $IGNORE_BEAM_MAPPING != 0 ))
    then
        echo "Invalid IGNORE_BEAM_MAPPING parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_MATCHING != 1 && $IGNORE_BEAM_MATCHING != 0 ))
    then
        echo "Invalid IGNORE_BEAM_MATCHING parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_OPTIMIZATION != 1 && $IGNORE_BEAM_OPTIMIZATION != 0 ))
    then
        echo "Invalid IGNORE_BEAM_OPTIMIZATION parameter. Required: 1 or 0. Exiting"
        exit       
    fi

    if(( $IGNORE_BEAM_UTILS != 1 && $IGNORE_BEAM_UTILS != 0 ))
    then
        echo "Invalid IGNORE_BEAM_UTILS parameter. Required: 1 or 0. Exiting"
        exit       
    fi
}

menu()
{
    echo "* Running this script will install libbeam and all dependencies to your system "
    echo "  if they are not currently available including:"
    echo "  * ROS (and all required ROS depenencies)"
    echo "  * Catch2"
    echo "  * Eigen3"
    echo "  * Ceres"
    echo "  * PCL"
    echo "  * OpenCV4"
    echo "  * Gflags"
    echo "  * nlohmann json"
    echo ""
    echo "* libbeam depends on OpenCV4 for certain modules (e.g., beam_cv), however, "
    echo "  if you are not using all modules, you may be able to get away with older versions. "
    echo "  To disable OpenCV4 install set the parameter INSTALL_OPENCV4=0"
    echo ""
    echo "* You may also chose to not install ceres if you are using modules that do not "
    echo "  depend on ceres, by setting INSTALL_CERES=0 "
    echo ""
    echo "* You may chose to install libbeam and certain dependencies locally by setting the "
    echo "  appropriate params (see below). Installing these locally means that we assume "
    echo "  libbeam has been cloned to some catkin workspace (e.g., ~/catkin_ws/src/) and "
    echo "  therefore we will not run make install on any of those packages. Instead, you "
    echo "  will need to run catkin build in the workspace after this script has completed."
    echo "  See the source directory below for where your the dependent source code will be saved."
    echo ""
    echo "* Parameters are currently set to:"
    echo "  INSTALL_LIBBEAM_LOCALLY: $INSTALL_LIBBEAM_LOCALLY"
    echo "  INSTALL_OPENCV4: $INSTALL_OPENCV4"
    echo "  INSTALL_OPENCV4_LOCALLY: $INSTALL_OPENCV4_LOCALLY"
    echo "  INSTALL_CERES: $INSTALL_CERES"
    echo ""
    echo "* Found script directory: $SCRIPT_DIR"
    echo "* Set source directory to: $SRC_DIR"
    echo ""
    echo "Would you like to continue? (y/n)"

    while read ans; do
        case "$ans" in
            y) break;;
            n) exit; break;;
            *) echo "Invalid input (y/n):";;
        esac
    done
}

install_routine()
{
    sudo -v

    # Ensure wget is available
    sudo apt-get install -qq wget  > /dev/null

    get_install_scripts

    # get UBUNTU_CODENAME and ROS_DISTRO
    source $INSTALL_SCRIPTS/identify_environment.bash

    # Import functions to install required dependencies, and get ROS distro and ubuntu name
    source $INSTALL_SCRIPTS/beam_dependencies_install.bash
        
    bash $INSTALL_SCRIPTS/ros_install.bash

    bash $INSTALL_SCRIPTS/rosdeps_install.bash

    # Install dependencies
    install_cmake
    install_catch2
    install_eigen3
    install_pcl
    install_json
    install_gflags

    if(( $INSTALL_CERES == 0 ))
    then
        echo "Not installing ceres"
    else 
        echo "installing ceres"
        install_ceres
    fi

    if(( $INSTALL_OPENCV4 == 0 ))
    then
        echo "Not installing opencv4"
    else 
        echo "installing opencv4"
        export OPENCV_SRC_PATH=$SRC_DIR
        export INSTALL_OPENCV4_LOCALLY=$INSTALL_OPENCV4_LOCALLY
        install_opencv4
    fi

    if(( $INSTALL_LIBBEAM_LOCALLY == 1 ))
    then
        echo "Not installing libbeam to system. "
        echo "You will need to build your catkin workspace."
    else 
        echo "building libbeam"
        cd $SRC_DIR
        cd libbeam
        mkdir -p build 
        cd build
        cmake .. -DCMAKE_IGNORE_BEAM_CALIBRATION=$IGNORE_BEAM_CALIBRATION \
        -DCMAKE_IGNORE_BEAM_COLORIZE=$IGNORE_BEAM_COLORIZE \
        -DCMAKE_IGNORE_BEAM_CONTAINERS=$IGNORE_BEAM_CONTAINERS \
        -DCMAKE_IGNORE_BEAM_CV=$IGNORE_BEAM_CV \
        -DCMAKE_IGNORE_BEAM_DEFECTS=$IGNORE_BEAM_DEFECTS \
        -DCMAKE_IGNORE_BEAM_DEPTH=$IGNORE_BEAM_DEPTH \
        -DCMAKE_IGNORE_BEAM_FILTERING=$IGNORE_BEAM_FILTERING \
        -DCMAKE_IGNORE_BEAM_MAPPING=$IGNORE_BEAM_MAPPING \
        -DCMAKE_IGNORE_BEAM_MATCHING=$IGNORE_BEAM_MATCHING \
        -DCMAKE_IGNORE_BEAM_OPTIMIZATION=$IGNORE_BEAM_OPTIMIZATION \
        -DCMAKE_IGNORE_BEAM_UTILS=$IGNORE_BEAM_UTILS
        make_with_progress -j$NUM_PROCESSORS
        
        echo "installing libbeam to system"
        sudo make install > /dev/null

    fi

    echo "libbeam installation completed. "
    echo "Please run the following steps if appropriate:"
    echo "* open a new terminal to re-source environment variables (if the bashrc was changed)"
    echo "* build your catkin workspace using: cd /path_to/catkin_ws && catkin build"
}

get_install_scripts()
{
    # Ensure that Beam install scripts are installed
    INSTALL_SCRIPTS="$SRC_DIR/beam_install_scripts"
    if [ -d $INSTALL_SCRIPTS ]; then
        echo "Beam install scripts found. Pulling most recent version of master."
        cd $INSTALL_SCRIPTS 
        git pull origin master
    else
        echo "Cloning Beam install scripts into:"
        echo $SRC_DIR
        cd $SRC_DIR 
        git clone https://github.com/BEAMRobotics/beam_install_scripts.git
    fi
}

main $@
