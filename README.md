# Minimal Reproduction - `catkin` Double Include Issue

This repository is a ROS package called `double_include` which is a minimal reproduction of a double-include issue.

**Problem**: `catkin_make` can't build the package if `FakeHeaderOnlyLibrary.h` is included via `nodecode.h`.

`nodecode.h` is included in `nodecpp.cpp` and `double_include_node.cpp`. 

```
LINK Pass 1: command "C:\Long\Windows\Path\link.exe /nologo /millionoptions"
failed (exit code 1169) with the following output:

nodecode.cpp.obj : error LNK2005: "int __cdecl FakeHeaderOnlyNamespace::gimme_seventeen(void)" (?gimme_seventeen@FakeHeaderOnlyNamespace@@YAHXZ) already defined in double_include_node.cpp.obj

C:\Code\ros\workspaces\fake_ws\devel\lib\double_include\double_include_node.exe : fatal error LNK1169: one or more multiply defined symbols found

```

# Branches

Clone this repo to your catkin workspace with 

```git
git clone https://github.com/danzimmerman/double_include_ros.git double_include
``` 

This repo has several branches:
 * `main` - Working `double_include` package without inclusion of `FakeHeaderOnlyLibrary.h`
 * `broken-include-guard` - Broken package with classic include guards.
 * `broken-pragma-once` - Broken package with `#pragma once` guard.
 * `wip-fixes` - Work in progress to fix. 

The failure is the same with classic `#ifndef DOUBLE_INCLUDE_WHATEVER_H` include guards or with `#pragma once` (this branch).

The double include is happening via `#include <double_include/nodecode.h>`. This problem doesn't happen if the library is a "professional" header-only library like `Eigen`. 

Source tree:

```
C:\CODE\ROS\WORKSPACES\FAKE_WS\SRC
│   CMakeLists.txt
│
└───double_include
    │   CMakeLists.txt
    │   package.xml
    │   README.md
    │
    ├───include
    │   └───double_include
    │           FakeHeaderOnlyLibrary.h
    │           nodecode.h
    │
    └───src
            double_include_node.cpp
            nodecode.cpp
```
# Cross-Platform

Same error on Ubuntu 18.04 with ROS Melodic, so it's not just a windows-specific problem. 

# `wip-fixes`

Work in progress. 

Relevant links:
 * Similar question [here](https://answers.ros.org/question/261570/proper-way-of-including-headers-in-catkin-c/)
 * First big long answer [here](https://stackoverflow.com/questions/14909997/why-arent-my-include-guards-preventing-recursive-inclusion-and-multiple-symbol)
 * Stack Overflow [Q&A](https://stackoverflow.com/questions/14425262/why-include-guards-do-not-prevent-multiple-function-definitions/14425299#14425299)
 * https://en.wikipedia.org/wiki/One_Definition_Rule
 * Why does OpenCL [not break the ODR](https://stackoverflow.com/questions/15960641/why-does-the-opencl-cl-hpp-header-only-wrapper-not-break-the-one-definition-ru)?

 ODR:
 > Every program shall contain exactly one definition of every non-inline function or variable that is odr-used in that program; no diagnostic required. The definition can appear explicitly in the program, it can be found in the standard or a user-defined library, or (when appropriate) it is implicitly defined (see 12.1, 12.4 and 12.8). An inline function shall be defined in every translation unit in which it is odr-used.

 ❓ Why does this work on Visual Studio and XCode, then?

A key takeaway from [Why does OpenCL not break the ODR?](https://stackoverflow.com/questions/15960641/why-does-the-opencl-cl-hpp-header-only-wrapper-not-break-the-one-definition-ru)

> A function defined **within a class definition** is an **inline function**. The inline specifier shall not appear on a block scope function declaration. If the inline specifier is used in a friend declaration, that declaration shall be a definition or the function shall have previously been declared inline.

If I make `FakeHeaderOnlyLibrary.h` contain only a class:

```cpp
#pragma once
namespace FakeHeaderOnlyNamespace
{
    
class FakeHeaderOnlyClass
{
public:
    int gimme_seventeen(void)
        {
            return 17;
        }
};
    
}
```

Then it compiles and runs:

```bash
c:\Code\ros\workspaces\fake_ws>rosrun double_include double_include_node
[ INFO] [1611081939.131564500]: double_include_node executable works!
[ INFO] [1611081939.131893900]: Callback fired from inside nodecode.cpp
[ INFO] [1611081939.132094200]: The fake library function gimme_seventeen() returns17
[ INFO] [1611081939.132300900]: Shutting down, ros::ok() returned false.
```

Fixes:
 * [Anon namespaces or `static`?](https://stackoverflow.com/questions/154469/unnamed-anonymous-namespaces-vs-static-functions)