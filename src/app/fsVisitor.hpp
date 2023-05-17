#pragma once

#include <FS.h>
#include <Fmt.h>

typedef enum {
  VA_PRINT = (1u << 0),
  VA_DEBUG = (1u << 1),
  VA_CACHE = (1u << 2),
} visitorAction_t;

void fsVisitor(fs::FS &fs, Fmt &fmt, const char *topdir = "/",
               uint32_t flags = 0, uint8_t levels = 10);