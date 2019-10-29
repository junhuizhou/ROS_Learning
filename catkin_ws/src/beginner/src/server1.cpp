#include "ros/ros.h"
#include "beginner/service1.h"

bool add(beginner::service1::Request &req, beginner::service1::Response &res)
{
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server1");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("add_two_ints",add);
    ROS_INFO("ready to add two ints.");
    ros::spin();

    return 0;
}