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
