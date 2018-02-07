px4_add_module(
	MODULE modules__serial
	MAIN serial
	STACK_MAIN 2000
	SRCS
		serial.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
