#ifndef SERIAL_COMM_SERIAL_COMM_H
#define SERIAL_COMM_SERIAL_COMM_H


#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>
#include <Eigen/Eigen>


#define BAUD_RATE B19200
   
class SerialComm{
public:

  SerialComm(); 
  void sendSerialMsg(const mav_msgs::ActuatorsPtr msg);
  void sendSerial(Eigen::VectorXd velocities);

private:
  int fd;

  Eigen::Vector3d w1;
  Eigen::Vector3d w2;
  Eigen::Vector3d w3;
  Eigen::Vector3d w4;

  // ros::NodeHandle nh;
  // ros::Subscriber sub; 
};



#endif