set(EXPLORATION_DEP ON)
add_subdirectory("./exploration")

#cf_executable(rr)
target_link_libraries(cf2 PRIVATE exploration)
target_include_directories(cf2 PRIVATE "./include")
target_sources(cf2 PRIVATE
	"./src/main.cpp"
	"./src/porting.cpp")
