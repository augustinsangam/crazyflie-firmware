set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(triple "arm" "none" "eabi")
set(multi "thumb" "v7e-m+fp" "hard")
set(cross "-mthumb" "-mcpu=cortex-m4" "-mfloat-abi=hard" "-mfpu=fpv4-sp-d16")
add_compile_options("${cross}")
add_link_options("${cross}")
add_compile_definitions(ARM_MATH_CM4 __FPU_PRESENT=1)
