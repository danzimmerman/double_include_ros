#include <double_include/nodecode.h> 

void double_include::LibWrapper::getSomethingSrvCallback()
{
    Eigen::Matrix3d mat;
    mat.setIdentity();
    ROS_INFO_STREAM("Callback responds with a matrix:\n" << mat);
}