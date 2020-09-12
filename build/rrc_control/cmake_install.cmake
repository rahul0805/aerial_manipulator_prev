# Install script for directory: /home/rahul/catkin_ws/src/control/controller-master

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rahul/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rahul/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rahul/catkin_ws/install" TYPE PROGRAM FILES "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rahul/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rahul/catkin_ws/install" TYPE PROGRAM FILES "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rahul/catkin_ws/install/setup.bash;/home/rahul/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rahul/catkin_ws/install" TYPE FILE FILES
    "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/setup.bash"
    "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rahul/catkin_ws/install/setup.sh;/home/rahul/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rahul/catkin_ws/install" TYPE FILE FILES
    "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/setup.sh"
    "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rahul/catkin_ws/install/setup.zsh;/home/rahul/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rahul/catkin_ws/install" TYPE FILE FILES
    "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/setup.zsh"
    "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/rahul/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/rahul/catkin_ws/install" TYPE FILE FILES "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrc_control/cmake" TYPE FILE FILES "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/rrc_control-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/share/roseus/ros/rrc_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/python2.7/dist-packages/rrc_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/python2.7/dist-packages/rrc_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/rrc_control.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrc_control/cmake" TYPE FILE FILES "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/rrc_control-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrc_control/cmake" TYPE FILE FILES
    "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/rrc_controlConfig.cmake"
    "/home/rahul/catkin_ws/build/rrc_control/catkin_generated/installspace/rrc_controlConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrc_control" TYPE FILE FILES "/home/rahul/catkin_ws/src/control/controller-master/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrc_control/launch" TYPE DIRECTORY FILES "/home/rahul/catkin_ws/src/control/controller-master/launch/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rrc_control/resource" TYPE DIRECTORY FILES "/home/rahul/catkin_ws/src/control/controller-master/resource/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpid_position_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpid_position_controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpid_position_controller.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/libpid_position_controller.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpid_position_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpid_position_controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpid_position_controller.so"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpid_position_controller.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsmc_position_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsmc_position_controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsmc_position_controller.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/libsmc_position_controller.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsmc_position_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsmc_position_controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsmc_position_controller.so"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsmc_position_controller.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libasmc_position_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libasmc_position_controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libasmc_position_controller.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/libasmc_position_controller.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libasmc_position_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libasmc_position_controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libasmc_position_controller.so"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libasmc_position_controller.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libautde_position_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libautde_position_controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libautde_position_controller.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/libautde_position_controller.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libautde_position_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libautde_position_controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libautde_position_controller.so"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libautde_position_controller.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/libreconfig_pid_controller.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/pid_position_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/pid_position_controller_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/pid_position_controller_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrc_control" TYPE EXECUTABLE FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/rrc_control/pid_position_controller_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/pid_position_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/pid_position_controller_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/pid_position_controller_node"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/rrc_control/lib:/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/pid_position_controller_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/asmc_position_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/asmc_position_controller_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/asmc_position_controller_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrc_control" TYPE EXECUTABLE FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/rrc_control/asmc_position_controller_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/asmc_position_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/asmc_position_controller_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/asmc_position_controller_node"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/rrc_control/lib:/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/asmc_position_controller_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/autde_position_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/autde_position_controller_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/autde_position_controller_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrc_control" TYPE EXECUTABLE FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/rrc_control/autde_position_controller_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/autde_position_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/autde_position_controller_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/autde_position_controller_node"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/rrc_control/lib:/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/autde_position_controller_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/smc_position_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/smc_position_controller_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/smc_position_controller_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrc_control" TYPE EXECUTABLE FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/rrc_control/smc_position_controller_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/smc_position_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/smc_position_controller_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/smc_position_controller_node"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/rrc_control/lib:/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/smc_position_controller_node")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/waypoint_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/waypoint_publisher")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/waypoint_publisher"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrc_control" TYPE EXECUTABLE FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/rrc_control/waypoint_publisher")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/waypoint_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/waypoint_publisher")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/waypoint_publisher"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/waypoint_publisher")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_example")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_example"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrc_control" TYPE EXECUTABLE FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/rrc_control/hovering_example")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_example")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_example"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_example")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_smooth" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_smooth")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_smooth"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrc_control" TYPE EXECUTABLE FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/rrc_control/hovering_smooth")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_smooth" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_smooth")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_smooth"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/hovering_smooth")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/points_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/points_publisher")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/points_publisher"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rrc_control" TYPE EXECUTABLE FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/rrc_control/points_publisher")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/points_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/points_publisher")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/points_publisher"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rrc_control/points_publisher")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/rahul/catkin_ws/devel/.private/rrc_control/lib/libreconfig_pid_controller.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so"
         OLD_RPATH "/home/rahul/catkin_ws/devel/.private/serial_comm/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib:/home/rahul/catkin_ws/devel/.private/mav_trajectory_generation/lib:/home/rahul/catkin_ws/devel/.private/yaml_cpp_catkin/lib:/home/rahul/catkin_ws/devel/.private/glog_catkin/lib:/home/rahul/catkin_ws/devel/.private/nlopt/lib:/home/rahul/catkin_ws/devel/.private/mav_visualization/lib:/opt/ros/kinetic/lib:/home/rahul/catkin_ws/devel/.private/eigen_checks/lib:/home/rahul/catkin_ws/devel/.private/msg_check/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreconfig_pid_controller.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rrc_control" TYPE DIRECTORY FILES "/home/rahul/catkin_ws/src/control/controller-master/include/rrc_control/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/rahul/catkin_ws/build/rrc_control/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/rahul/catkin_ws/build/rrc_control/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
