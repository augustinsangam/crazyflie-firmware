set(EXPLORATION_DEP ON)
set(EXPLORATION_METHOD 1)
set(EXPLORATION_DRONE_INITIALISATION_DELAY 3000)

add_subdirectory("./exploration")
target_link_libraries(exploration PRIVATE common warnings)

target_link_libraries(cf2 PRIVATE exploration)
target_include_directories(cf2 PRIVATE "./include")
target_sources(cf2 PRIVATE
	"./src/main.cpp"
	"./src/communication.cpp"
	"./src/battery.cpp"
	"./src/porting.cpp")
