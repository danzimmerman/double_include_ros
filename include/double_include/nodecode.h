#pragma once

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