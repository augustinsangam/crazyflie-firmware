include("${CMAKE_CURRENT_LIST_DIR}/stm32f4.cmake")
string(REPLACE ";" "-" triple "${triple}")

set(CMAKE_SYSROOT "/opt/stm32f4")

set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

set(CMAKE_ASM_COMPILER_TARGET "${triple}")
set(CMAKE_C_COMPILER_TARGET "${triple}")
set(CMAKE_CXX_COMPILER_TARGET "${triple}")

set(CMAKE_ASM_FLAGS "-Wno-unused-command-line-argument")
set(CMAKE_C_FLAGS "-fshort-enums")
set(CMAKE_CXX_FLAGS "-fshort-enums -stdlib=libstdc++")
set(CMAKE_EXE_LINKER_FLAGS "-nodefaultlibs -fuse-ld=bfd")

set(a "${CMAKE_SYSROOT}/lib/crti.o ${CMAKE_SYSROOT}/lib/crtbegin.o ${CMAKE_SYSROOT}/lib/crt0.o")
set(b "${CMAKE_SYSROOT}/lib/crtend.o ${CMAKE_SYSROOT}/lib/crtn.o")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> -o <TARGET> ${a} <OBJECTS> <LINK_LIBRARIES> ${b}")
set(CMAKE_CXX_STANDARD_LIBRARIES "-l stdc++_nano -l supc++_nano -l c_nano -l m -l gcc -l nosys")

#TRIPLE='arm-none-eabi'
#MULTI=$("${TRIPLE}-gcc" -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -print-multi-directory)
#SYSROOT="/usr/lib/${TRIPLE}"
#sudo mkdir bin include lib
#for f in "${SYSROOT}/bin"/*; do sudo ln -s "$f" bin; done
#for f in "${SYSROOT}/include"/* "${SYSROOT}/include/c++/9.2.1/${TRIPLE}/${MULTI}"/*; do sudo ln -fs "$f" include; done
#for f in "/usr/lib/gcc/${TRIPLE}/9.2.1/${MULTI}"/* "${SYSROOT}/lib/${MULTI}"/*; do sudo ln -s "$f" lib; done
