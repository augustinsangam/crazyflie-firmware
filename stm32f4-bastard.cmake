include("${CMAKE_CURRENT_LIST_DIR}/stm32f4.cmake")
string(REPLACE ";" "-" triple "${triple}")
string(REPLACE ";" "/" multi "${multi}")

set(CMAKE_SYSROOT "/usr/lib/${triple}")

if(NOT DEFINED GCC_VERSION)
	set(GCC_VERSION "9.2.1")
endif()

# Clang
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_FLAGS "-stdlib=libstdc++ -isystem /usr/include/newlib/c++/${GCC_VERSION}/${triple}/${multi}")
# GCC
#set(CMAKE_C_COMPILER "${triple}-gcc")
#set(CMAKE_CXX_COMPILER "${triple}-g++")
#set(CMAKE_C_FLAGS "-mfp16-format=ieee")

set(CMAKE_C_COMPILER_TARGET "${triple}")
set(CMAKE_CXX_COMPILER_TARGET "${triple}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fshort-enums")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fshort-enums")

# We cannot specify explicitely linker, neither C linker nor C++ linker
# So let’s hack CMake!
set(CMAKE_C_LINK_FLAGS "${triple}-gcc")
set(CMAKE_CXX_LINK_FLAGS "${triple}-g++")
set(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs --specs=nano.specs")
set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_LINK_FLAGS> <LINK_FLAGS> -o <TARGET> <OBJECTS> <LINK_LIBRARIES>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> -o <TARGET> <OBJECTS> <LINK_LIBRARIES>")
