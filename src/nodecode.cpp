#include <double_include/nodecode.h> 

void double_include::LibWrapper::getSomethingSrvCallback()
{
    ROS_INFO_STREAM("Callback fired from inside nodecode.cpp");
}