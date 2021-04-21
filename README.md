# Crazyflie firmware

This repository contains the source code for the crazyflie firmware. We are using it in order interact with the drones and also to implements new settings.

The code is used as basics and you can find it in the github repository of BitCraze. You can find it here:
```
https://github.com/bitcraze/crazyflie-firmware
```

You can find all the codes that we've added to manage the drones in the `projects` folder.

Please check the `README.md` present it each subfolder to know more about each work.

### Development

```sh
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Debug -D CMAKE_TOOLCHAIN_FILE=stm32f4-clang_plus_gcc.cmake ..
```

### Production

```sh
mkdir build-gcc
cd build-gcc
cmake -D CMAKE_BUILD_TYPE=MinSizeRel -D CMAKE_TOOLCHAIN_FILE=stm32f4-gcc.cmake ..
```
