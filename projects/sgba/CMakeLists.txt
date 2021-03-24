target_include_directories(cf2 PRIVATE "./include")
target_compile_definitions(cf2 PRIVATE METHOD=3)
target_sources(cf2 PRIVATE
	"./src/exploration/SGBA.cpp"
	"./src/exploration/StateMachine.cpp"
	"./src/exploration/WallFollowing.cpp"
	#"./src/exploration/WallFollowingWithAvoid.cpp"
	"./src/exploration/median_filter.cpp"
	"./src/main.cpp"
	"./src/porting.cpp"
)
