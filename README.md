# TechTheTime-LowLevel

## Requirement

The projects in this repository are compatible with STM32CubeIDE. No libraries are to be installed manually.

## Workspace structure

The workspace contains a STM32CubeIDE project for every boards in the robot. In addition to their separate code, the boards share common source code located in [TechTheTime-Shared](https://github.com/Club-INTech/TechTheTime-Shared).

## Configuration

First of all, enter the repository root and run the configuration script :
```
./configure
```

Then, you must add the projects in the workspace. Open STM32CubeIDE then go to `File > Switch Workspace > Other` and select the root of the repository as the workspace.

Then go to `File > Import > Existing Projects into Workspace` and select the root directory as the repository folder. You should then be able to select the three projects of the repository (lib, Master and TechTheTime-Shared) and add them to the workspace.
