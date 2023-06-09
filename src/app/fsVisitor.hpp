#pragma once

#include <SdFat.h>
#include <FS.h>
#include <Fmt.h>

typedef enum {
  VA_PRINT = (1u << 0),
  VA_DEBUG = (1u << 1),
  VA_CACHE = (1u << 2),
  VA_RECURSIVE = (1u << 2),
} visitorAction_t;

void fsVisitor(SdFat &fs, Fmt &fmt, const char *topdir = "/",
               uint32_t flags = 0, uint8_t levels = 10);


void fsVisitor(fs::FS &fs, Fmt &fmt, const char *topdir = "/",
               uint32_t flags = 0, uint8_t levels = 10);

               
