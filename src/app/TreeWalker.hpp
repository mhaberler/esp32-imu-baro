#pragma once

#include <FS.h>
#include <Fmt.h>
#include <Functor.h>

// return false to stop the walk
typedef Functor4wRet<fs::FS &, Fmt &, fs::File &, uint32_t, bool> Visitor;

class TreeWalker {
public:
  TreeWalker(const Visitor &visitor) : visitor_(visitor){};
  uint32_t begin(FS &fs, Fmt &out, uint32_t flags = 0,
                 const char *dirName = "/", uint8_t levels = 5) {
    visited_ = 0;
    fs_ = &fs;
    flags_ = flags;
    File current_dir = fs.open(dirName);
    if (!current_dir) {
      log_e("failed to open directory '%s'", dirName);
      return false;
    }
    if (!current_dir.isDirectory()) {
      log_e("not a directory: '%s'", dirName);
      return false;
    }
    walk(current_dir, out, levels - 1);
    return visited_;
  }

private:
  Visitor visitor_;
  uint32_t visited_;
  fs::FS *fs_;
  uint32_t flags_;

  void walk(File dir, Fmt &out, uint8_t levels) {
    File fd;
    while ((fd = dir.openNextFile())) {
      visited_++;
      if (visitor_) {
        if (!visitor_(*fs_, out, fd, flags_)) {
          return;
        }
      }
      if (levels) {
        walk(fd, out, levels - 1);
      }
    }
  }
};
