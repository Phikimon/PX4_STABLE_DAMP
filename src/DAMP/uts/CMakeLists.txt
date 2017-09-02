px4_add_module(
	MODULE DAMP__uts
	MAIN uts
	STACK_MAIN 2000
	SRCS
		uts.c
	DEPENDS
		platforms__common
	)