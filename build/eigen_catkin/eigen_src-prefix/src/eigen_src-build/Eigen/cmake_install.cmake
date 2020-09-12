# Install script for directory: /home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rahul/catkin_ws/devel/.private/eigen_catkin")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/StdDeque"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Eigenvalues"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Householder"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Dense"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/SPQRSupport"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/UmfPackSupport"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Core"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Cholesky"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Eigen"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Sparse"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/SVD"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/CholmodSupport"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/SparseCholesky"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/SparseQR"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/SuperLUSupport"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Jacobi"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/IterativeLinearSolvers"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/LU"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/PaStiXSupport"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/QtAlignedMalloc"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/StdList"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/Geometry"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/OrderingMethods"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/SparseCore"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/MetisSupport"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/StdVector"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/PardisoSupport"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/QR"
    "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/SparseLU"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/rahul/catkin_ws/build/eigen_catkin/eigen_src-prefix/src/eigen_src/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

