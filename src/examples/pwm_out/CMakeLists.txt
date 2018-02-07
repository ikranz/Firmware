px4_add_module(
	MODULE examples__pwm_out
	MAIN pwm_out
	STACK_MAIN 2000
	SRCS
		pwm_out.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
