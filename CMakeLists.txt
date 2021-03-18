cmake_minimum_required(VERSION 3.16)
project(cf VERSION 2021.1 LANGUAGES ASM C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#################################### CUSTOM ####################################

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
	OUTPUT "version.c"
	COMMAND "${Python3_EXECUTABLE}"
	        "./tools/make/versionTemplate.py"
	        "./src/utils/src/version.vtpl"
	        "${CMAKE_CURRENT_BINARY_DIR}/version.c"
	WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cf_fw")
add_custom_target(version_c DEPENDS "version.c")

#################################### VENDOR ####################################

add_library(common INTERFACE)
target_compile_options(common INTERFACE
	"-fno-math-errno" "-fno-strict-aliasing"
	"-ffunction-sections" "-fdata-sections")
target_link_options(common INTERFACE "LINKER:--gc-sections")
target_include_directories(common INTERFACE "./cf_fw/src/config")

add_library(warnings INTERFACE)
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
	target_compile_options(warnings INTERFACE "-Wall" "-Wextra" "-Wpedantic")
	target_compile_options(warnings INTERFACE
		"-Wcast-qual"
		"-Wconversion"
		"-Wdouble-promotion"
		"-Wformat=2"
		"-Wmissing-declarations"
		"-Wshadow"
		"-Wstrict-overflow")
endif()

add_library(cmsis INTERFACE)
target_include_directories(cmsis INTERFACE "./cf_fw/vendor/CMSIS/CMSIS/Core/Include")

add_library(arm_math STATIC)
target_link_libraries(arm_math PRIVATE common)
target_link_libraries(arm_math PRIVATE cmsis)
target_include_directories(arm_math PUBLIC "./cf_fw/vendor/CMSIS/CMSIS/DSP/Include")
target_include_directories(arm_math PRIVATE "./cf_fw/vendor/CMSIS/CMSIS/DSP/PrivateInclude")
target_sources(arm_math PRIVATE
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_abs_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_add_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_add_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_add_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_add_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_and_u16.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_and_u32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_and_u8.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_mult_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_negate_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_not_u16.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_not_u32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_not_u8.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_offset_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_or_u16.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_or_u32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_or_u8.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_scale_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_shift_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_shift_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_shift_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_sub_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_xor_u16.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_xor_u32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BasicMathFunctions/arm_xor_u8.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/CommonTables/arm_common_tables.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/CommonTables/arm_const_structs.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/CommonTables/arm_mve_tables.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_conj_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_conj_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_conj_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_dot_prod_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_dot_prod_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_squared_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_real_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_real_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mult_real_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ControllerFunctions/arm_pid_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ControllerFunctions/arm_pid_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ControllerFunctions/arm_pid_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ControllerFunctions/arm_pid_reset_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ControllerFunctions/arm_pid_reset_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ControllerFunctions/arm_pid_reset_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ControllerFunctions/arm_sin_cos_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/ControllerFunctions/arm_sin_cos_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_boolean_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_braycurtis_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_canberra_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_chebyshev_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_cityblock_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_correlation_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_cosine_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_dice_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_euclidean_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_hamming_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_jaccard_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_jensenshannon_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_kulsinski_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_minkowski_distance_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_rogerstanimoto_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_russellrao_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_sokalmichener_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_sokalsneath_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/DistanceFunctions/arm_yule_distance.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_cos_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_cos_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_sin_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_sin_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_sqrt_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_sqrt_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_vexp_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FastMathFunctions/arm_vlog_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_32x64_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_32x64_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_fast_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_fast_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df1_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_df2T_init_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_stereo_df2T_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_biquad_cascade_stereo_df2T_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_fast_opt_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_fast_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_fast_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_opt_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_opt_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_fast_opt_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_fast_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_fast_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_opt_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_opt_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_partial_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_conv_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_fast_opt_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_fast_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_fast_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_opt_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_opt_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_correlate_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_fast_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_fast_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_fast_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_fast_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_lattice_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_init_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_fir_sparse_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_iir_lattice_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_norm_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/FilteringFunctions/arm_lms_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cmplx_mult_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_fast_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_entropy_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_entropy_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_kullback_leibler_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_kullback_leibler_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_logsumexp_dot_prod_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_logsumexp_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_max_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_max_no_idx_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_max_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_max_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_max_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_mean_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_min_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_min_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_min_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_min_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_power_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_power_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_power_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_power_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_rms_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_rms_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_rms_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_std_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_std_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_std_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_var_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_var_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/StatisticsFunctions/arm_var_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_barycenter_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_bitonic_sort_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_bubble_sort_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_heap_sort_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_insertion_sort_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_merge_sort_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_merge_sort_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_quick_sort_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_selection_sort_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_sort_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_sort_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_spline_interp_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_spline_interp_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SupportFunctions/arm_weighted_sum_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SVMFunctions/arm_svm_linear_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.S" # or .c
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix2_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_dct4_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_dct4_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_dct4_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_dct4_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_dct4_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_dct4_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_init_f32.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_init_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_init_q31.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_q15.c"
	"./cf_fw/vendor/CMSIS/CMSIS/DSP/Source/TransformFunctions/arm_rfft_q31.c")

add_library(dw1000 STATIC)
target_include_directories(dw1000 PUBLIC "./cf_fw/vendor/libdw1000/inc")
target_sources(dw1000 PRIVATE
	"./cf_fw/vendor/libdw1000/src/libdw1000.c"
	"./cf_fw/vendor/libdw1000/src/libdw1000Spi.c")

add_library(stm32f4xx STATIC)
target_link_libraries(stm32f4xx PRIVATE common cmsis warnings)
target_compile_definitions(stm32f4xx PUBLIC
	HSE_VALUE=8000000
	STM32F4XX
	STM32F40_41xxx
	USE_STDPERIPH_DRIVER)
target_include_directories(stm32f4xx PUBLIC
	"./cf_fw/src/lib/CMSIS/STM32F4xx/Include"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/inc")
target_sources(stm32f4xx PRIVATE
	"./cf_fw/src/lib/CMSIS/STM32F4xx/Source/system_stm32f4xx.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_misc.c"
	"./cf_fw/src/lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c")

add_library(cf2 STATIC)

add_dependencies(cf2 version_c)
target_link_libraries(cf2 PUBLIC cmsis stm32f4xx)
target_link_libraries(cf2 PRIVATE common warnings)
target_link_libraries(cf2 PRIVATE arm_math dw1000)
target_link_libraries(cf2 PRIVATE m)

target_compile_options(cf2 PUBLIC "-Wno-address-of-packed-member")

target_include_directories(cf2 PUBLIC
	"./cf_fw/vendor/FreeRTOS/include"
	"./cf_fw/vendor/FreeRTOS/portable/GCC/ARM_CM4F")
target_sources(cf2 PRIVATE
	"./cf_fw/vendor/FreeRTOS/portable/GCC/ARM_CM4F/port.c"
	"./cf_fw/vendor/FreeRTOS/portable/MemMang/heap_4.c"
	"./cf_fw/vendor/FreeRTOS/list.c"
	"./cf_fw/vendor/FreeRTOS/queue.c"
	"./cf_fw/vendor/FreeRTOS/tasks.c"
	"./cf_fw/vendor/FreeRTOS/timers.c")

target_compile_definitions(cf2 PRIVATE APP_ENABLED APP_STACKSIZE=300)

target_compile_definitions(cf2 PRIVATE
	"BLOBS_LOC=\"${CMAKE_CURRENT_SOURCE_DIR}/blobs/\"")

set(ESTIMATOR "any")
set(CONTROLLER "Any")
set(POWER_DISTRIBUTION "stock")
set(UART2_LINK OFF)

target_compile_definitions(cf2 PRIVATE CRAZYFLIE_FW)
target_compile_definitions(cf2 PRIVATE
	SENSOR_INCLUDED_BMI088_BMP388
	SENSOR_INCLUDED_BMI088_SPI_BMP388
	SENSOR_INCLUDED_MPU9250_LPS25H)
target_compile_definitions(cf2 PRIVATE
	"ESTIMATOR_NAME=${ESTIMATOR}Estimator"
	"CONTROLLER_NAME=ControllerType${CONTROLLER}"
	"POWER_DISTRIBUTION_TYPE_${POWER_DISTRIBUTION}")

# utils
target_include_directories(cf2 PUBLIC
	"./cf_fw/src/utils/interface"
	"./cf_fw/src/utils/interface/lighthouse")
target_include_directories(cf2 PRIVATE "./cf_fw/src/utils/interface/tdoa")
target_sources(cf2 PRIVATE
	"./cf_fw/src/utils/src/kve/kve.c"
	"./cf_fw/src/utils/src/kve/kve_storage.c"
	"./cf_fw/src/utils/src/lighthouse/lighthouse_calibration.c"
	"./cf_fw/src/utils/src/lighthouse/lighthouse_geometry.c"
	"./cf_fw/src/utils/src/lighthouse/ootx_decoder.c"
	"./cf_fw/src/utils/src/lighthouse/pulse_processor.c"
	"./cf_fw/src/utils/src/lighthouse/pulse_processor_v1.c"
	"./cf_fw/src/utils/src/lighthouse/pulse_processor_v2.c"
	"./cf_fw/src/utils/src/tdoa/tdoaEngine.c"
	"./cf_fw/src/utils/src/tdoa/tdoaStats.c"
	"./cf_fw/src/utils/src/tdoa/tdoaStorage.c"
	"./cf_fw/src/utils/src/FreeRTOS-openocd.c"
	"./cf_fw/src/utils/src/cfassert.c"
	"./cf_fw/src/utils/src/clockCorrectionEngine.c"
	"./cf_fw/src/utils/src/configblockeeprom.c"
	"./cf_fw/src/utils/src/cpuid.c"
	"./cf_fw/src/utils/src/crc.c" # TODO: remove in next release
#	"./cf_fw/src/utils/src/crc32.c"
	"./cf_fw/src/utils/src/crc_bosch.c" # TODO: remove in next release
	"./cf_fw/src/utils/src/debug.c"
	"./cf_fw/src/utils/src/eprintf.c"
	"./cf_fw/src/utils/src/filter.c"
	"./cf_fw/src/utils/src/num.c"
	"./cf_fw/src/utils/src/rateSupervisor.c"
	"./cf_fw/src/utils/src/sleepus.c"
	"./cf_fw/src/utils/src/statsCnt.c")
target_sources(cf2 PRIVATE "version.c")

# deck
target_include_directories(cf2 PUBLIC "./cf_fw/src/deck/interface")
target_include_directories(cf2 PRIVATE "./cf_fw/src/deck/drivers/interface")
target_sources(cf2 PRIVATE
	"./cf_fw/src/deck/api/deck_analog.c"
	"./cf_fw/src/deck/api/deck_constants.c"
	"./cf_fw/src/deck/api/deck_digital.c"
	"./cf_fw/src/deck/api/deck_spi.c"
	"./cf_fw/src/deck/api/deck_spi3.c"
	"./cf_fw/src/deck/core/deck.c"
	"./cf_fw/src/deck/core/deck_drivers.c"
	"./cf_fw/src/deck/core/deck_info.c"
	"./cf_fw/src/deck/core/deck_test.c"
	"./cf_fw/src/deck/drivers/src/test/exptest.c"
	"./cf_fw/src/deck/drivers/src/test/exptestBolt.c"
	"./cf_fw/src/deck/drivers/src/test/exptestRR.c"
	"./cf_fw/src/deck/drivers/src/activeMarkerDeck.c"
	"./cf_fw/src/deck/drivers/src/bigquad.c"
	"./cf_fw/src/deck/drivers/src/buzzdeck.c"
	"./cf_fw/src/deck/drivers/src/cppmdeck.c"
	"./cf_fw/src/deck/drivers/src/flowdeck_v1v2.c"
	"./cf_fw/src/deck/drivers/src/gtgps.c"
	"./cf_fw/src/deck/drivers/src/ledring12.c"
	"./cf_fw/src/deck/drivers/src/lighthouse.c"
	"./cf_fw/src/deck/drivers/src/locodeck.c"
	"./cf_fw/src/deck/drivers/src/lpsTdoa2Tag.c"
	"./cf_fw/src/deck/drivers/src/lpsTdoa3Tag.c"
	"./cf_fw/src/deck/drivers/src/lpsTwrTag.c"
	"./cf_fw/src/deck/drivers/src/multiranger.c"
	"./cf_fw/src/deck/drivers/src/oa.c"
	"./cf_fw/src/deck/drivers/src/usddeck.c"
	"./cf_fw/src/deck/drivers/src/zranger.c"
	"./cf_fw/src/deck/drivers/src/zranger2.c")
if(UART2_LINK)
	target_compile_definitions(cf2 private UART2_LINK_COMM)
else()
	target_sources(cf2 PRIVATE "./cf_fw/src/deck/drivers/src/aideck.c")
endif()

# drivers
target_include_directories(cf2 PUBLIC "./cf_fw/src/drivers/interface")
target_include_directories(cf2 PRIVATE "./cf_fw/src/drivers/bosch/interface")
target_sources(cf2 PRIVATE
	"./cf_fw/src/drivers/bosch/src/bmi055_accel.c"
	"./cf_fw/src/drivers/bosch/src/bmi055_gyro.c"
	"./cf_fw/src/drivers/bosch/src/bmi088_accel.c"
	"./cf_fw/src/drivers/bosch/src/bmi088_fifo.c"
	"./cf_fw/src/drivers/bosch/src/bmi088_gyro.c"
	"./cf_fw/src/drivers/bosch/src/bmi160.c"
	"./cf_fw/src/drivers/bosch/src/bmm150.c"
	"./cf_fw/src/drivers/bosch/src/bmp280.c"
	"./cf_fw/src/drivers/bosch/src/bmp3.c"
	"./cf_fw/src/drivers/bosch/src/bstdr_comm_support.c"
	"./cf_fw/src/drivers/src/ak8963.c"
	"./cf_fw/src/drivers/src/cppm.c"
	"./cf_fw/src/drivers/src/diskio.c"
	"./cf_fw/src/drivers/src/eeprom.c"
	"./cf_fw/src/drivers/src/exti.c"
	"./cf_fw/src/drivers/src/fatfs_sd.c"
	"./cf_fw/src/drivers/src/i2c_drv.c"
	"./cf_fw/src/drivers/src/i2cdev_f405.c"
	"./cf_fw/src/drivers/src/led_f405.c"
	"./cf_fw/src/drivers/src/lh_bootloader.c"
	"./cf_fw/src/drivers/src/lps25h.c"
	"./cf_fw/src/drivers/src/maxsonar.c"
	"./cf_fw/src/drivers/src/motors.c"
	"./cf_fw/src/drivers/src/mpu6500.c"
	"./cf_fw/src/drivers/src/nvic.c"
	"./cf_fw/src/drivers/src/pca9685.c"
	"./cf_fw/src/drivers/src/piezo.c"
	"./cf_fw/src/drivers/src/pmw3901.c"
	"./cf_fw/src/drivers/src/swd.c"
	"./cf_fw/src/drivers/src/uart1.c"
	"./cf_fw/src/drivers/src/uart2.c"
	"./cf_fw/src/drivers/src/uart_syslink.c"
	"./cf_fw/src/drivers/src/vl53l0x.c"
	"./cf_fw/src/drivers/src/vl53l1x.c"
	"./cf_fw/src/drivers/src/watchdog.c"
	"./cf_fw/src/drivers/src/ws2812_cf2.c")

# hal
target_include_directories(cf2 PUBLIC "./cf_fw/src/hal/interface")
target_sources(cf2 PRIVATE
	"./cf_fw/src/hal/src/amg8833.c"
	"./cf_fw/src/hal/src/buzzer.c"
	"./cf_fw/src/hal/src/freeRTOSdebug.c"
	"./cf_fw/src/hal/src/ledseq.c"
	"./cf_fw/src/hal/src/ow_common.c"
	"./cf_fw/src/hal/src/ow_syslink.c"
	"./cf_fw/src/hal/src/pca9555.c"
	"./cf_fw/src/hal/src/pca95x4.c"
	"./cf_fw/src/hal/src/pm_stm32f4.c" # TODO: pm_${CPU}
	"./cf_fw/src/hal/src/proximity.c"
	"./cf_fw/src/hal/src/radiolink.c"
	"./cf_fw/src/hal/src/sensors.c"
	"./cf_fw/src/hal/src/sensors_bmi088_bmp388.c"
	"./cf_fw/src/hal/src/sensors_bmi088_spi_bmp388.c"
	"./cf_fw/src/hal/src/sensors_mpu9250_lps25h.c"
	"./cf_fw/src/hal/src/storage.c"
	"./cf_fw/src/hal/src/syslink.c"
	"./cf_fw/src/hal/src/usb.c"
	"./cf_fw/src/hal/src/usb_bsp.c"
	"./cf_fw/src/hal/src/usbd_desc.c"
	"./cf_fw/src/hal/src/usblink.c"
	"./cf_fw/src/hal/src/usec_time.c")

# modules
target_include_directories(cf2 PUBLIC "./cf_fw/src/modules/interface")
target_include_directories(cf2 PRIVATE
	"./cf_fw/src/modules/interface/kalman_core"
	"./cf_fw/src/modules/interface/lighthouse")
target_sources(cf2 PRIVATE
#	"./cf_fw/src/modules/src/kalman_core/kalman_core.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_absolute_height.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_distance.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_flow.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_pose.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_position.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_sweep_angles.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_tdoa.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_tof.c"
#	"./cf_fw/src/modules/src/kalman_core/mm_yaw_error.c"
	"./cf_fw/src/modules/src/lighthouse/lighthouse_core.c"
	"./cf_fw/src/modules/src/lighthouse/lighthouse_deck_flasher.c"
	"./cf_fw/src/modules/src/lighthouse/lighthouse_position_est.c"
#	"./cf_fw/src/modules/src/lighthouse/lighthouse_storage.c"
	"./cf_fw/src/modules/src/app_channel.c"
	"./cf_fw/src/modules/src/app_handler.c"
	"./cf_fw/src/modules/src/attitude_pid_controller.c"
	"./cf_fw/src/modules/src/collision_avoidance.c"
	"./cf_fw/src/modules/src/comm.c"
	"./cf_fw/src/modules/src/commander.c"
	"./cf_fw/src/modules/src/console.c"
	"./cf_fw/src/modules/src/controller.c"
	"./cf_fw/src/modules/src/controller_indi.c"
	"./cf_fw/src/modules/src/controller_mellinger.c"
	"./cf_fw/src/modules/src/controller_pid.c"
	"./cf_fw/src/modules/src/crtp.c"
	"./cf_fw/src/modules/src/crtp_commander.c"
	"./cf_fw/src/modules/src/crtp_commander_generic.c"
	"./cf_fw/src/modules/src/crtp_commander_high_level.c"
	"./cf_fw/src/modules/src/crtp_commander_rpyt.c"
	"./cf_fw/src/modules/src/crtp_localization_service.c"
	"./cf_fw/src/modules/src/crtpservice.c"
	"./cf_fw/src/modules/src/estimator.c"
	"./cf_fw/src/modules/src/estimator_complementary.c"
	"./cf_fw/src/modules/src/estimator_kalman.c"
	"./cf_fw/src/modules/src/extrx.c"
#	"./cf_fw/src/modules/src/health.c"
	"./cf_fw/src/modules/src/kalman_core.c" # TODO: remove in next release
	"./cf_fw/src/modules/src/kalman_supervisor.c"
	"./cf_fw/src/modules/src/log.c"
	"./cf_fw/src/modules/src/mem.c"
	"./cf_fw/src/modules/src/msp.c"
	"./cf_fw/src/modules/src/outlierFilter.c"
	"./cf_fw/src/modules/src/param.c"
	"./cf_fw/src/modules/src/peer_localization.c"
	"./cf_fw/src/modules/src/pid.c"
	"./cf_fw/src/modules/src/planner.c"
	"./cf_fw/src/modules/src/platformservice.c"
	"./cf_fw/src/modules/src/position_controller_indi.c"
	"./cf_fw/src/modules/src/position_controller_pid.c"
	"./cf_fw/src/modules/src/position_estimator_altitude.c"
	"./cf_fw/src/modules/src/power_distribution_${POWER_DISTRIBUTION}.c"
	"./cf_fw/src/modules/src/pptraj.c"
	"./cf_fw/src/modules/src/pptraj_compressed.c"
	"./cf_fw/src/modules/src/queuemonitor.c"
	"./cf_fw/src/modules/src/range.c"
	"./cf_fw/src/modules/src/sensfusion6.c"
	"./cf_fw/src/modules/src/sitaw.c"
	"./cf_fw/src/modules/src/sound_cf2.c"
	"./cf_fw/src/modules/src/stabilizer.c"
	"./cf_fw/src/modules/src/static_mem.c"
	"./cf_fw/src/modules/src/sysload.c"
	"./cf_fw/src/modules/src/system.c"
	"./cf_fw/src/modules/src/tdoaEngineInstance.c"
	"./cf_fw/src/modules/src/trigger.c"
	"./cf_fw/src/modules/src/worker.c")

# lib/fatfs
target_include_directories(cf2 PRIVATE "./cf_fw/src/lib/FatFS")
target_sources(cf2 PRIVATE
	"./cf_fw/src/lib/FatFS/ff.c"
	"./cf_fw/src/lib/FatFS/ffunicode.c"
	"./cf_fw/src/lib/FatFS/syscall.c")

# lib/vl53l1
target_include_directories(cf2 PRIVATE
	"./cf_fw/src/lib/vl53l1"
	"./cf_fw/src/lib/vl53l1/core/inc")
target_sources(cf2 PRIVATE
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_api.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_api_calibration.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_api_core.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_api_debug.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_api_preset_modes.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_api_strings.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_core.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_core_support.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_error_strings.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_register_funcs.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_silicon_core.c"
	"./cf_fw/src/lib/vl53l1/core/src/vl53l1_wait.c")

# platform
target_include_directories(cf2 PRIVATE "./cf_fw/src/platform")
target_sources(cf2 PRIVATE
	"./cf_fw/src/platform/platform.c"
	"./cf_fw/src/platform/platform_cf2.c"
	"./cf_fw/src/platform/platform_stm32f4.c"
	"./cf_fw/src/platform/platform_utils.c")

target_sources(cf2 PRIVATE
	"./cf_fw/src/init/main.c"
	"./cf_fw/src/init/startup_stm32f40xx.s")

target_include_directories(cf2 PRIVATE
	"./cf_fw/src/lib/STM32_USB_Device_Library/Core/inc")
target_sources(cf2 PRIVATE
	"./cf_fw/src/lib/STM32_USB_Device_Library/Core/src/usbd_core.c"
	"./cf_fw/src/lib/STM32_USB_Device_Library/Core/src/usbd_ioreq.c"
	"./cf_fw/src/lib/STM32_USB_Device_Library/Core/src/usbd_req.c")

target_include_directories(cf2 PRIVATE
	"./cf_fw/src/lib/STM32_USB_OTG_Driver/inc")
target_sources(cf2 PRIVATE
	"./cf_fw/src/lib/STM32_USB_OTG_Driver/src/usb_core.c"
	"./cf_fw/src/lib/STM32_USB_OTG_Driver/src/usb_dcd.c"
	"./cf_fw/src/lib/STM32_USB_OTG_Driver/src/usb_dcd_int.c")

add_subdirectory("./projects")
