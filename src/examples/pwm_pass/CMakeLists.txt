px4_add_module(
	MODULE examples__pwm_pass
	MAIN pwm_pass
	STACK_MAIN 2000
	SRCS
		pwm_pass.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
