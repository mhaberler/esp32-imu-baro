#pragma once

#include "spdlog/spdlog.h"
#include "spdlog/fmt/bin_to_hex.h"

#define LOGT(format, ...)                                                      \
  do {                                                                         \
    spdlog::debug(format, ##__VA_ARGS__);                                      \
  } while (0)

#define LOGD(format, ...)                                                      \
  do {                                                                         \
    spdlog::debug(format, ##__VA_ARGS__);                                      \
  } while (0)
#define LOGI(format, ...)                                                      \
  do {                                                                         \
    spdlog::info(format, ##__VA_ARGS__);                                       \
  } while (0)
#define LOGW(format, ...)                                                      \
  do {                                                                         \
    spdlog::warn(format, ##__VA_ARGS__);                                       \
  } while (0)
#define LOGE(format, ...)                                                      \
  do {                                                                         \
    spdlog::error(format, ##__VA_ARGS__);                                      \
  } while (0)
#define LOGF(format, ...)                                                      \
  do {                                                                         \
    spdlog::critical(format, ##__VA_ARGS__);                                   \
  } while (0)

