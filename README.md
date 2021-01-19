# Minimal Reproduction - `catkin` Double Include Issue

**Problem**: Including the library `FakeHeaderOnlyLibrary.h` which defines a single function `gimme_seventeen()` fails if `nodecode.h` is included in both `nodecpp.cpp` and `double_include_node.cpp`. 

```
LINK Pass 1: command "C:\Long\Windows\Path\link.exe /nologo /millionoptions"
failed (exit code 1169) with the following output:

nodecode.cpp.obj : error LNK2005: "int __cdecl FakeHeaderOnlyNamespace::gimme_seventeen(void)" (?gimme_seventeen@FakeHeaderOnlyNamespace@@YAHXZ) already defined in double_include_node.cpp.obj

C:\Code\ros\workspaces\fake_ws\devel\lib\double_include\double_include_node.exe : fatal error LNK1169: one or more multiply defined symbols found

```

The failure is the same with classic `#ifndef DOUBLE_INCLUDE_WHATEVER_H` include guards or with `#pragma once` (this branch).

