px4_add_module(
	MODULE modules__sonar_uart
	MAIN sonar_uart
	COMPILE_FLAGS
	SRCS
		sonar_uart.c
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
