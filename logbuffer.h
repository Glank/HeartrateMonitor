#ifndef LOGBUFFER_H
#define LOGBUFFER_H

#include <memory>

// Assumes this to be the integer type for which setting
// and getting is an atomic operation. Depends on the underlying architecture,
// but since I'm only using esp8266, and it's 32 bit, int is fine.
#define ATOMIC_INT int

// A circular ring buffer loging util, for safe logging within interrupts.
class LogBuffer {
  private:
    ATOMIC_INT write_head;
    ATOMIC_INT read_head;
    ATOMIC_INT buffer_len;
    std::unique_ptr<char[]> buffer;
  public:
    LogBuffer(ATOMIC_INT length) : buffer(new char[length]) {
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
    void flush_to_serial();
};
#endif