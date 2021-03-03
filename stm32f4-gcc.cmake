include("${CMAKE_CURRENT_LIST_DIR}/stm32f4.cmake")
string(REPLACE ";" "-" triple "${triple}")

set(CMAKE_SYSROOT "/usr/lib/${triple}")

set(CMAKE_C_COMPILER "${triple}-gcc")
set(CMAKE_CXX_COMPILER "${triple}-g++")

set(gcc_flags "-mfp16-format=ieee")
set(CMAKE_ASM_FLAGS "${gcc_flags}")
set(CMAKE_C_FLAGS "${gcc_flags}")
set(CMAKE_CXX_FLAGS "${gcc_flags}")
set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs --specs=nano.specs")
