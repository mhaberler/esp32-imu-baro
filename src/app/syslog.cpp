#include "defs.hpp"

#include "logmacros.hpp"
#include "spdlog/details/null_mutex.h"
#include "spdlog/sinks/base_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"


#include <memory>
#include <mutex>
#include <type_traits>

template <typename Mutex>
class bv_sink : public spdlog::sinks::base_sink<Mutex> {

protected:
  void sink_it_(const spdlog::details::log_msg &msg) override {

    // log_msg is a struct containing the log entry info like level, timestamp,
    // thread id etc. msg.raw contains pre formatted log

    // If needed (very likely but not mandatory), the sink formats the message
    // before sending it to its final destination:
    spdlog::memory_buf_t formatted;
    spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
    std::string buf = fmt::to_string(formatted);
    Console.print(buf.c_str());
    // write_to_console_clients(buf.c_str(), buf.size());
  }

  void flush_() override { Console.flush(); }
};

using bv_sink_st = bv_sink<spdlog::details::null_mutex>;

// set via config
void set_syslog_loglevel(const int level) {
  LOGD("setting loglevel to {}", level);
  spdlog::set_level((spdlog::level::level_enum)level);
}

void setup_syslog(void) {

  auto bv_sink = std::make_shared<bv_sink_st>();
  auto bv_logger =
      std::make_shared<spdlog::logger>("bvlog", std::move(bv_sink));

  // see
  // https://github.com/gabime/spdlog/wiki/3.-Custom-formatting#pattern-flags
  bv_logger->set_pattern("[%H:%M:%S.%e %@ %l] %v");
  bv_logger->set_level(spdlog::level::debug);
  spdlog::set_default_logger(std::move(bv_logger));
}