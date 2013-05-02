#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Twist.h"

#include <cmath>

#include <floatfann.h>
#include <fann_cpp.h>

fann_type g_input[8];
fann_type *g_output;

void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud_msg)
{
    for (int i = 0; i < 8; ++i)
        g_input[i] = sqrt(pow(point_cloud_msg->points[i].x, 2) + pow(point_cloud_msg->points[i].y, 2) + pow(point_cloud_msg->points[i].z, 2));
}

int main(int argc, char **argv)
{
    FANN::neural_net net;
    net.create_from_file("/home/jalan/ros_workspace/ashish_repo/p3dx_rosaria/src/neural_network_controller_float.net");

    ros::init(argc, argv, "nn_controller");
    ros::NodeHandle nodeHandle1;
    ros::NodeHandle nodeHandle2;
    ros::Subscriber subscriber = nodeHandle1.subscribe("/RosAria/sonar", 1000, sonarCallback);
    ros::Publisher publisher = nodeHandle2.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1000);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist twist;
    twist.linear.x = 0.1;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;

    while (ros::ok())
    {
        g_output = net.run(g_input);
        twist.angular.z = *g_output;

        publisher.publish(twist);

        ros::spinOnce();
        loop_rate.sleep();
    }

    net.destroy();

    return 0;
}

