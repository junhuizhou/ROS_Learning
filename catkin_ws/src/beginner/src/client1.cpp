#include <cstdlib>
#include "ros/ros.h"
#include "beginner/service1.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client1");
    if(argc != 3)
    {
        ROS_INFO("usage: client1");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner::service1>("add_two_ints");
    beginner::service1 srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if(client.call(srv))
    {
        ROS_INFO("sum: %d", (long int)srv.response.sum);
    }
    else
    {
        ROS_INFO("failed to call service add_two_ints");
        return 1;
    }
    
    return 0;
}