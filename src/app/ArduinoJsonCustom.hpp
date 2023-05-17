#pragma once


struct SpiRamAllocator {
  void *allocate(size_t size) {
    if (psRAMavail)
      return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    else
      return (malloc(size));
  }
  void deallocate(void *pointer) {
    if (psRAMavail)
      heap_caps_free(pointer);
    else
      free(pointer);
  }
};
using SpiRamJsonDocument = BasicJsonDocument<SpiRamAllocator>;

void combine(JsonObject &dst, const JsonDocument &src);

