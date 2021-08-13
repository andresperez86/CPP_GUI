cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export PX4_HOME_LAT=10.447081433440465
export PX4_HOME_LON=-75.39116931309206
roslaunch cpp_package celsia_mavros_posix_sitl.launch
