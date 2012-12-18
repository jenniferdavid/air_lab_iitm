#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"

#include <iostream>
#include <cmath>
#include <boost/lexical_cast.hpp>

void displayCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud_msg) {
	double sonar_readings[8];
	for (int i = 0; i < 8; ++i) {
		sonar_readings[i] = sqrt(pow(point_cloud_msg->points[i].x, 2) + pow(point_cloud_msg->points[i].y, 2) + pow(point_cloud_msg->points[i].z, 2));
		std::string s = "Sonar " + boost::lexical_cast<std::string>(i) + ": " + boost::lexical_cast<std::string>(sonar_readings[i]);
		ROS_INFO("%s", s.c_str());
	}
	std::cout<<std::endl;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "display_sonar_readings");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber = nodeHandle.subscribe("/RosAria/sonar", 1000, displayCallback);

	ros::spin();
	return 0;
}
