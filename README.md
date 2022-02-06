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

## Updating the shared repository

### Pulling the latest version

If you have an outdated version of the shared repository and you need to update it, just run the same command as configuration :
```
./update
```

### Pushing your modification of the shared library

If you have modified your local copy of the shared repository, you must go into the `TechTheTime-Shared` folder and push your change. This step is essential, because you will not be able to push the main repository otherwise.

## Update the dependencies

### Update the submodules

When pulling the latest version of the repository, your local versions of the submobules will not be automatically updated. In that case, just run `./configure`.

### Compile the latest version of the controller

Likewise, you need to recompile the controller each time you are pulling from a branch whose version of the controller sources is different from yours. In that case, run `./update` to get the latest version of the shared repository then run `./configure`.
