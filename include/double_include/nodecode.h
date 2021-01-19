#ifndef DOUBLE_INCLUDE_NODECODE_H
#define DOUBLE_INCLUDE_NODECODE_H

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <boost/dll/import.hpp>
#include <Eigen/Core>

namespace double_include
{

class LibWrapper
{
    public:
        //registerSolver(DHSolver* solver);
        void getSomethingSrvCallback();
};

}

#endif //DOUBLE_INCLUDE_NODECODE_H include guard