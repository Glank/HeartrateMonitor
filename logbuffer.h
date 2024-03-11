#ifndef LOGBUFFER_H
#define LOGBUFFER_H

#include <memory>
#include <functional>
#include <Arduino.h>

// A circular ring buffer loging util, for safe logging within interrupts.
class LogBuffer {
  private:
    int write_head;
    int read_head;
    int buffer_len;
    std::unique_ptr<uint8_t[]> buffer;
    static std::unique_ptr<LogBuffer> global_instance;
  public:
    LogBuffer(int length) : buffer(new uint8_t[length]) {
      buffer_len = length;
      // write head must always be ahead of read_head
      // leaves 1 char on the table, but simplifies thread safety
      write_head = 1;
      read_head = 0;
    }
    ~LogBuffer() = default;
    // a count of the number of overflow errors
    int overflow_errs = 0;
    // fast, but should not be interrupted
    // returns 0 on success, -1 on buffer overflow error
    int log(char* str);
    // slow, but fine to be interrupted
    // should be called within loop or ticker to periodically dump the log lines in
    // the buffer to Serial, otherwise no logs will be printed and the buffer will overflow.
    void flush(std::function<void(const uint8_t*, size_t)> write);
    static void init_global(int length);
    static LogBuffer* global();
};

#define LOG(n, template, ...) {char _l[n];sprintf(_l, template __VA_OPT__(,) __VA_ARGS__);LogBuffer::global()->log(_l);}
#define FLUSH_LOG_TO_SERIAL() LogBuffer::global()->flush([](const uint8_t* b, size_t n){Serial.write(b,n);});
#endif