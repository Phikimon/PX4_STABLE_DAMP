px4_add_module(
	MODULE DAMP__main_motor
	MAIN main_motor
	STACK_MAIN 2000
	SRCS
		main_motor.cpp
		../lib/common.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
