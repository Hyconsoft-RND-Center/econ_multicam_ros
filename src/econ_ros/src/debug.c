#include "debug.h"

int log_level = 3;

#include <stdarg.h>
#include <stdio.h>

void DEBUG(int level, const char *fmt, ...)
{
	va_list arg;

	/* Check if the message should be logged */
	if (level > log_level)
		return;

	/* Write the error message */
	va_start(arg, fmt);
	vfprintf(stdout, fmt, arg);
	va_end(arg);
}

void ERROR(const char *fmt, ...)
{
	va_list arg;
	/* Write the error message */
	fprintf(stderr, "[ERROR] ");
	va_start(arg, fmt);
	vfprintf(stderr, fmt, arg);
	va_end(arg);
	fprintf(stderr, "\n");
}
