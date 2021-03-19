option(CLOAD "Enable cload bootloader" ON)
if(CLOAD)
	set(LINKER_SCRIPT "FLASH_CLOAD.ld")
	set(LOAD_ADDRESS 0x8004000)
else()
	set(LINKER_SCRIPT "FLASH.ld")
	set(LOAD_ADDRESS 0x8000000)
endif()

set(OPENOCD_INTERFACE "interface/stlink-v2.cfg")
set(OPENOCD_TARGET "target/stm32f4x_stlink.cfg")

#find_program(OpenOCD_EXECUTABLE openocd)
#if(OpenOCD_EXECUTABLE)
#	set(FULL_OUTPUT_BIN "${CMAKE_CURRENT_BINARY_DIR}/${OUTPUT_NAME}.bin")
#
#	add_custom_target(flash
#		COMMAND "${OpenOCD_EXECUTABLE}"
#		        -d 2
#		        -f "${OPENOCD_INTERFACE}" -f "${OPENOCD_TARGET}"
#		        -c init -c targets -c "reset halt"
#		        -c "flash write_image erase ${FULL_OUTPUT_BIN} ${LOAD_ADDRESS} bin"
#		        -c "reset run" -c shutdown
#		DEPENDS cf2_bin
#		WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cf_fw")
#
#	add_custom_target(verify
#		COMMAND "${OpenOCD_EXECUTABLE}"
#		        -d 2
#		        -f "${OPENOCD_INTERFACE}" -f "${OPENOCD_TARGET}"
#		        -c init -c targets -c "reset halt"
#		        -c "verify_image ${FULL_OUTPUT_BIN} ${LOAD_ADDRESS} bin"
#		        -c "reset run" -c shutdown
#		DEPENDS cf2_bin
#		WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cf_fw")
#endif()

function(add_cf_executable target)
	add_executable(${target})
	set_target_properties(${target} PROPERTIES SUFFIX ".elf")
	target_link_libraries(${target} PRIVATE common warnings)
	target_link_libraries(${target} PRIVATE cf2)
	# DOES NOT WORK: invalid order: -T ... -L ...
	#target_link_directories(cf PRIVATE "./tools/make/F405/linker")
	target_link_options(${target} PRIVATE
		"-L${CMAKE_SOURCE_DIR}/cf_fw/tools/make/F405/linker"
		"SHELL:-T ${LINKER_SCRIPT}")

	add_custom_command(
		OUTPUT "${target}.bin"
		COMMAND "${CMAKE_OBJCOPY}"
				-O binary
				--pad-to=0
				--remove-section=._usrstack --remove-section=.bss --remove-section=.nzds
				"$<TARGET_FILE:${target}>" "${target}.bin"
		DEPENDS ${target})
	add_custom_target(${target}_bin DEPENDS "${target}.bin")
	if(CLOAD)
		add_custom_target(${target}_cload
			COMMAND "${Python3_EXECUTABLE}" -m cfloader flash "${target}.bin" stm32-fw
			DEPENDS "${target}_bin")
	endif()
endfunction()

add_subdirectory("./cdr")
add_subdirectory("./rr")
