px4_add_module(
	MODULE DAMP__brake_actuator
	MAIN brake_actuator
	STACK_MAIN 2000
	SRCS
		brake_actuator.cpp
		../lib/common.cpp
		../lib/switch.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
