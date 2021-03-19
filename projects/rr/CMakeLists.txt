set(EXPLORATION_DEP ON)
set(EXPLORATION_METHOD 1)
set(TICKS_PER_FSM_LOOP 10)
add_subdirectory("./exploration")

#cf_executable(rr)
target_link_libraries(cf2 PRIVATE exploration)
target_include_directories(cf2 PRIVATE "./include")
target_sources(cf2 PRIVATE
	"./src/main.cpp"
	"./src/porting.cpp")
