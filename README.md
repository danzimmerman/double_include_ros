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