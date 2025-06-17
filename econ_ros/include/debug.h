#ifndef __DEBUG_H__
#define __DEBUG_H__
#include <stdio.h>
#include <stdarg.h>

void DEBUG(int level, const char *fmt, ...);
void ERROR(const char *fmt, ...);

#endif
