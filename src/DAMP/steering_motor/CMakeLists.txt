px4_add_module(
	MODULE DAMP__steering_motor
	MAIN steering_motor
	STACK_MAIN 2000
	SRCS
		steering_motor.cpp
		../pid_cont/pid_cont.cpp
		../common.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
