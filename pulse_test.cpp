#include "pulse_test.h"
#include <cstdio>
#include <vector>
#include <algorithm>

#define ASSERT(t, msg, ...) if(!(t)){char l[128];sprintf(l, msg __VA_OPT__(,) __VA_ARGS__);Serial.println(l);return false;}
#define ASSERT_CONT(ac, t, msg, ...) if(!(t)){char l[128];sprintf(l, msg __VA_OPT__(,) __VA_ARGS__);Serial.println(l);ac = false;}

bool test_ring_buffer() {
  Serial.println("Testing RingBuffer...");
  RingBuffer<int> buf(3);
  buf.push_back() = -1;
  ASSERT(buf.back() == -1, "back != -1");
  ASSERT(buf.size() == 1, "size != 1");
  ASSERT(buf.capacity() == 3, "capacity != 3");
  for(int i = 0; i < 10; i++)
    buf.push_back() = i;
  ASSERT(buf[0] == 7, "buf[0] != 7");
  ASSERT(buf[2] == 9, "buf[2] != 9");
  ASSERT(buf.full(), "buf not full");
  buf.push_back() = 10;
  buf.push_back() = 11;
  ASSERT(buf.back() == 11, "back != 11");
  return true;
}

bool test_mem_stack() {
  Serial.println("Testing MemStack...");

  MemStack<int, 3> memstack;

  auto n1 = memstack.make();
  *n1 = 1;
  {
    ASSERT(memstack.num_free() == 2, "Expected 2 free memory blocks, had %d", memstack.num_free());
    auto n2 = memstack.make();
    *n2 = 2;
    ASSERT(memstack.num_free() == 1, "Expected 1 free memory blocks, had %d", memstack.num_free());
    auto n3 = memstack.make();
    *n3 = 3;
    ASSERT(memstack.num_free() == 0, "Expected 0 free memory blocks, had %d", memstack.num_free());
  }
  ASSERT(memstack.num_free() == 2, "Expected 2 free memory blocks again, but had %d", memstack.num_free());
  memstack.set_steal([&]{
    auto cpy = n1;
    n1 = nullptr;
    return cpy;
  });
  auto n2 = memstack.make();
  auto n3 = memstack.make();
  auto n4 = memstack.make();
  ASSERT(n1 == nullptr, "Expected n1 to be stolen.");
  ASSERT(memstack.num_free() == 0, "Expected 0 free memory blocks again, had %d", memstack.num_free());
  //auto n5 = memstack.make(); Causes failure & reset, as desired

  return true;
}

class TestStream : public PeakProcessingStream {
  std::function<void(std::shared_ptr<Peak>)> on_push;
  public:
    TestStream(std::function<void(std::shared_ptr<Peak>)> f) : on_push(f) {}
    void push(std::shared_ptr<Peak> p) override { on_push(p); }
    std::shared_ptr<Peak> pop() override { return nullptr; }
};

bool test_width_calc_stream() {
  Serial.println("Testing WidthCalculatingStream...");

  MemStack<Peak, 5> memstack;
  WidthCalculatingStream stream;
  std::vector<float> widths;
  TestStream test_out([&](std::shared_ptr<Peak> p){
    widths.push_back(p->w);
  });
  stream.set_next(&test_out);

  // send a peak every second
  for(int t = 0; t < 10000; t+=1000) {
    auto peak = memstack.make();
    peak->t = t;
    stream.push(peak);
  }

  for(float w : widths) {
    ASSERT(w==2000, "Width was not 2000ms.");
  }
  ASSERT(widths.size() == 8, "Not 8 widths output, %d", widths.size());

  return true;
}

bool all_pulse_tests() {
  Serial.println("Running tests for \"pulse.h\\cpp\"...");

  ASSERT(test_ring_buffer(), "RingBuffer Failed");
  ASSERT(test_mem_stack(), "MemStack Failed");
  ASSERT(test_width_calc_stream(), "WidthCalculatingStream Failed");
  
  Serial.println("All tests pass!");
  return true;
}