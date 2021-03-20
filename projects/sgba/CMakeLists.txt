target_include_directories(cf2 PRIVATE "./include")
target_sources(cf2 PRIVATE
	"./src/porting.cpp"
	"./src/StateMachine.cpp"
	"./src/main.cpp"
	#"./SGBA.c"
	#"./state_machine.c"
	"./wallfollowing_multiranger_onboard.c"
	#"./wallfollowing_with_avoid.c"
	)
