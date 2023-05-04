#pragma once

#include <Fmt.h>

void heap_report(Fmt &s, const char *fn = NULL, const int line = 0);
void psram_report(Fmt &s, const char *fn, const int line);
void platform_report(Fmt &s);
void build_setup_report(Fmt &s);
