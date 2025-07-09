#include <stdio.h>

#define YELLOW "\x1B[33m"
#define WHITE "\x1B[37m"
#define BLUE "\x1B[34m"
#define GREEN "\x1B[32m"
#define RED "\x1B[31m"
#define RESET "\x1B[0m"

void draw_line()
{
	printf(RED "+====================================================================+\n" RESET);
}

void draw_single_line()
{
	printf(RED "+--------------------------------------------------------------------+\n" RESET);
}

void print_main_menu()
{
	printf("\n");
	printf(RED "+====================================================================+\n");
	printf(GREEN "|                  MULTICAM command line application         	     |\n");
	printf(RED "+====================================================================+\n" RESET);
}
