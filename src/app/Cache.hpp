#include <FS.h>
#include <RomDiskStream.hpp>
#include <fmt/chrono.h>
#include <map>

#include "esp_heap_caps.h"

// somewhere in the ESPAsyncWebServer event loop:

// String myFilePath = "/path/to/myFile.txt";
// if (MyPsramFiles[myFilePath] != NULL) {
//   server.streamFile(MyPsramFiles[myFilePath].getStream(), "text/plain");
// }

class Cache {

  struct RamFile_t {
    char *data;
    size_t size;
    RomDiskStream getStream() {
      return RomDiskStream((const uint8_t *)data, size);
    }
  };
  typedef std::map<String, RamFile_t> ramCache_t;

public:
  Cache(){};
  ~Cache() { reset(); };

  void reset(void) {
    // FIXME
    // free all resources
    // reset countes
  }

  bool hit(const String &path) { return cache_.find(path) != cache_.end(); }

  const RomDiskStream &lookup(const String &path) {
    return cache_[path].getStream();
  }

  bool insert(fs::FS &sourceFS, const char *path) {
    fs::File myFile = sourceFS.open(path);

    if (!myFile) {
      log_e("toPsram: failed to open '%s'", path);
      return false;
    }
    size_t fsize = myFile.size() + 1;
    if (ESP.getMaxAllocPsram() < fsize) {
      log_e("file larger than  getMaxAllocPsram): %u/%u\n", fsize + 1,
            ESP.getMaxAllocPsram());
      return false;
    }
    auto data_mem = ((char *)heap_caps_malloc(fsize, MALLOC_CAP_SPIRAM));
    if (data_mem == NULL) {
      log_e("failed to alloc %u\n", fsize);

      myFile.close();
      return false;
    }
    RamFile_t myPsRamFile = {.data = data_mem, .size = myFile.size()};
    size_t bytes_read = myFile.readBytes(myPsRamFile.data, myFile.size());
    bool result = (myFile.size() == bytes_read);
    if (!result) {
      // incomplete copy ?
      log_w("[File copy missed %d bytes for %s\n", myFile.size() - bytes_read,
            path);
      heap_caps_free(data_mem);
    } else {
      total_bytes_ += bytes_read;
      total_files_++;
      cache_[path] = myPsRamFile; // add file to global psram-files array
    }
    myFile.close();
    return result;
  }
  void stats(uint32_t &total_bytes, uint32_t &total_files) {

    total_bytes = total_bytes_;
    total_files = total_files_;
  }

private:
  ramCache_t cache_;
  uint32_t total_bytes_, total_files_;
};

extern Cache psramCache;