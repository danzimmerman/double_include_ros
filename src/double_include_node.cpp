#include <ros/ros.h>
#include <double_include/nodecode.h>

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("double_include_node executable works!");

    double_include::LibWrapper libwrap; 

    libwrap.getSomethingSrvCallback();

    //ROS_INFO_STREAM("The fake library function gimme_seventeen() returns" << FakeHeaderOnlyNamespace::gimme_seventeen());

    while (ros::ok())
    {
        ros::spin();
    }

    ROS_INFO_STREAM("Shutting down, ros::ok() returned false.");

    return 0;
}