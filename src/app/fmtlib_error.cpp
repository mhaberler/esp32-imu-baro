#include "defs.hpp"

extern "C" {
void fmtlib_error(const char *file, int line, const char *message) {
  log_e("%s:%d: assertion failed: %s\n", file, line, message);
}
}
