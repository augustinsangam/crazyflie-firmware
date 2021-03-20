target_include_directories(cf2 PRIVATE "./include")
target_sources(cf2 PRIVATE
	"./src/porting.cpp"
	"./src/exploration/StateMachine.cpp"
	"./src/main.cpp"
	"./src/exploration/median_filter.cpp"
	"./wallfollowing_multiranger_onboard.c"
)
