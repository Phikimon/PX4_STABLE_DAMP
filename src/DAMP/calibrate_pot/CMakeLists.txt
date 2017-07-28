px4_add_module(
	MODULE DAMP__calibrate_pot
	MAIN calibrate_pot
	STACK_MAIN 2000
	SRCS
		calibrate_pot.cpp
		../lib/common.cpp
		../lib/switch.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
