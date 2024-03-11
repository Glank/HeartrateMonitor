#include "logbuffer.h"

#include "Arduino.h"

int LogBuffer::log(char* str) {
  int i = 0;
  int bi;
  while(str[i] != 0) {
    bi = (write_head+i)%buffer_len;
    if (bi == read_head) {
      // can't catch up to the read head
      overflow_errs++;
      return -1;
    }
    buffer[bi] = str[i];
    i++;
  }

  // append a '\n'
  bi = (write_head+i)%buffer_len;
  if (bi == read_head) {
    // can't catch up to the read head
    overflow_errs++;
    return -1;
  }
  buffer[bi] = '\n';
  i++;

  write_head = (write_head+i)%buffer_len;
  return 0;
}

void LogBuffer::flush(std::function<void(const uint8_t*, size_t)> write) {
  noInterrupts();
  int cur_write_head = write_head;
  int read_start = (read_head+1)%buffer_len;
  interrupts();
  // if the write head has wrapped around, but the read head hasn't yet
  if (cur_write_head < read_head && read_start != 0) {
    // flush to the end of the buffer
    Serial.write(&buffer[read_start], buffer_len-read_start);
    read_start = 0;
  }
  // flush to catch up with write_head
  if (cur_write_head != 0)
    write(&buffer[read_start], cur_write_head-read_start);
  read_head = (buffer_len+cur_write_head-1)%buffer_len;
}

std::unique_ptr<LogBuffer> LogBuffer::global_instance{nullptr};
void LogBuffer::init_global(int length) {
  LogBuffer::global_instance = std::make_unique<LogBuffer>(length);
}
LogBuffer* LogBuffer::global() {
  return LogBuffer::global_instance.get();
}