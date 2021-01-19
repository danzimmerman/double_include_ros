#ifndef DOUBLE_INCLUDE_NODECODE_H
#define DOUBLE_INCLUDE_NODECODE_H

#include <ros/ros.h>
#include <double_include/FakeHeaderOnlyLibrary.h>

namespace double_include
{

class LibWrapper
{
    public:
        void getSomethingSrvCallback();
};

}

#endif //DOUBLE_INCLUDE_NODECODE_H include guard