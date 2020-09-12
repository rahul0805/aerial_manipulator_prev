
#include "serial_comm.h"


SerialComm::SerialComm() {
  fd = open( "/dev/ttyUSB0", O_WRONLY| O_NONBLOCK | O_NDELAY );
  struct termios options;
  cfsetospeed(&options, BAUD_RATE);
  options.c_cflag |= (CLOCAL | CREAD);
  tcsetattr(fd, TCSANOW, &options);


  ROS_INFO("Serial node active");

  if (fd < 0)
  {
    ROS_INFO_STREAM("Serial communication failed");
  }

  w1 << 918.96891, 0.048367, 7.26316e-6;
  w2 << 905.01605, 0.050611, 7.045246e-6;
  w3 << 885.098643, 0.060828, 6.130089e-6;
  w4 << 1123.49376, 0.00845071, 9.486336e-6;
}

void SerialComm::sendSerial(Eigen::VectorXd velocities){
  velocities = 5*velocities;
  Eigen::Vector3d v1, v2, v3, v4;
  v1 << 1, velocities[0], pow(velocities[0], 2);
  v2 << 1, velocities[1], pow(velocities[1], 2);
  v3 << 1, velocities[2], pow(velocities[2], 2);
  v4 << 1, velocities[3], pow(velocities[3], 2);

  int velocities_[4];

  velocities_[0] = v1.dot(w1);
  velocities_[1] = v2.dot(w2);
  velocities_[2] = v3.dot(w3);
  velocities_[3] = v4.dot(w4);
  
  std::string v;
  v.append(std::to_string(velocities_[0]));
  v.append(",");
  v.append(std::to_string(velocities_[1]));
  v.append(",");
  v.append(std::to_string(velocities_[2]));
  v.append(",");
  v.append(std::to_string(velocities_[3]));
  v.append(",1\n");

  const char* velocity_str = v.c_str();
  // ROS_INFO_STREAM(velocity_str);  
  int n_written = write( fd, velocity_str, sizeof(velocity_str) -1 );
}


void SerialComm::sendSerialMsg(const mav_msgs::ActuatorsPtr msg){
  // std::vector<double> velocities = msg->angular_velocities;
  // Eigen::VectorXd velocities_(velocities.data());

  Eigen::VectorXd velocities_;
  velocities_ << msg->angular_velocities[0], msg->angular_velocities[1], 
                  msg->angular_velocities[2], msg->angular_velocities[3];
  sendSerial(velocities_);
}