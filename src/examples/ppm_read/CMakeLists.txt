px4_add_module(
	MODULE examples__ppm_read
	MAIN ppm_read
	STACK_MAIN 2000
	SRCS
		ppm_read.c
	DEPENDS
		platforms__common
	)