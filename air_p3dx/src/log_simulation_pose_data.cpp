#include <iostream>
#include <string>
#include <fstream>
#include <QtGui/qapplication.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class LogSimulationPoseData {
public:
	LogSimulationPoseData(std::string, std::string);
	virtual ~LogSimulationPoseData();
	void logPoseData();
	void poseCallback(const nav_msgs::Odometry::ConstPtr&);
private:
	std::ifstream map_file;
	std::ofstream log_file;
	std::ofstream log_file_tmp;
};

LogSimulationPoseData::LogSimulationPoseData(std::string map_file_name,
		std::string log_file_name) :
		map_file(map_file_name), log_file(log_file_name), log_file_tmp(
				"/tmp/pose_data.txt") {
}

LogSimulationPoseData::~LogSimulationPoseData() {
	map_file.close();
	log_file.close();
	log_file_tmp.close();
}

void LogSimulationPoseData::logPoseData() {
	// initial pose of the robot
	float initial_x0 = 0.0;
	float initial_y0 = 0.0;
	float initial_theta0 = 0.0;
	std::string line;
	while (std::getline(map_file, line)) {

	}

}

void LogSimulationPoseData::poseCallback(
		const nav_msgs::Odometry::ConstPtr& pose_msg) {
	log_file_tmp << pose_msg->pose.pose.position.x;
	log_file_tmp << " ";
	log_file_tmp << pose_msg->pose.pose.position.y;
	log_file_tmp << " ";
	log_file_tmp << pose_msg->pose.pose.position.z;
	log_file_tmp << " \n";
}

int main(int argc, char **argv) {
	std::string map_file_name;
	std::string log_file_name;
	std::cin >> map_file_name;

	LogSimulationPoseData *logSimulationPoseData = new LogSimulationPoseData(
			map_file_name, log_file_name);

	ros::init(argc, argv, "log_simulation_pose_data");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber = nodeHandle.subscribe("/RosAria/pose", 1000,
			logSimulationPoseData->poseCallback);
	ros::spin();

	delete logSimulationPoseData;
	return 0;
}
