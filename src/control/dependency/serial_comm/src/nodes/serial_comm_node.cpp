
#include <ros/ros.h>
#include "serial_comm.h"


class SerialCommNode{
  public:
    SerialCommNode(){
      sub = nh.subscribe("/pelican/command/motor_speed", 1, &SerialComm::sendSerialMsg, &sc);
    }

  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    SerialComm sc;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_comm_node");
  SerialCommNode scn;
  ros::spin();
}