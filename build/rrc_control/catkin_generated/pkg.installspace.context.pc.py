# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/home/rahul/catkin_ws/devel/include/eigen3".split(';') if "${prefix}/include;/home/rahul/catkin_ws/devel/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;mav_msgs;nav_msgs;roscpp;rospy;sensor_msgs;message_runtime;serial_comm;mav_trajectory_generation_ros;msg_check".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpid_position_controller;-lsmc_position_controller;-lasmc_position_controller;-lautde_position_controller;-lreconfig_pid_controller".split(';') if "-lpid_position_controller;-lsmc_position_controller;-lasmc_position_controller;-lautde_position_controller;-lreconfig_pid_controller" != "" else []
PROJECT_NAME = "rrc_control"
PROJECT_SPACE_DIR = "/home/rahul/catkin_ws/install"
PROJECT_VERSION = "2.1.2"
