#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "msg_check_node.h"

namespace msg_check {
	MsgCheckNode::MsgCheckNode(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh), private_nh_(private_nh) {
		plot_data_pub_ = nh_.advertise<msg_check::PlotDataMsg>("/data_out", 1);
		box_pub_ = nh_.advertise<msg_check::BoxMsg>("/box_data", 1);
		sendMsg();

	}

	MsgCheckNode::~MsgCheckNode() {}

	void MsgCheckNode::sendMsg() {
		checker_.writeMsg(&data_out_);
		data_out_.thrust = 5;
		plot_data_pub_.publish(data_out_);
		box_data_.mass = 10;
		box_pub_.publish(box_data_);
		ROS_INFO("Published message");
	}


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "msg_check_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  msg_check::MsgCheckNode msg_check_node(nh, private_nh);


  ros::spin();

  return 0;
}