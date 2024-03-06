#include "pulse_test.h"
#include <cstdio>
#include <vector>
#include <list>
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

template <typename T>
class TestStream : public PushTarget<std::shared_ptr<T>> {
  std::function<void(std::shared_ptr<T>)> on_push;
  public:
    TestStream(std::function<void(std::shared_ptr<T>)> f) : on_push(f) {}
    void push(std::shared_ptr<T> p) override { on_push(p); }
};

bool test_width_calc_stream() {
  Serial.println("Testing WidthCalcStream...");

  MemStack<Peak, 5> memstack;
  std::vector<long> widths;
  TestStream<Peak> test_out([&](std::shared_ptr<Peak> p){
    widths.push_back(p->w);
  });
  WidthCalcStream stream(&test_out);

  // send a peak every second
  for(int t = 0; t < 10000; t+=1000) {
    auto peak = memstack.make();
    peak->t = t;
    stream.push(peak);
  }

  for(long w : widths) {
    ASSERT(w==2000, "Width was not 2000ms.");
  }
  ASSERT(widths.size() == 8, "Not 8 widths output, %d", widths.size());

  return true;
}

bool test_width_stats_stream() {
  Serial.println("Testing WidthStatsStream...");

  MemStack<Peak, 2*PULSE_VALIDATION_WINDOW_MS/1000> memstack;
  std::vector<float> avgs;
  std::vector<float> stds;
  TestStream<Peak> test_out([&](std::shared_ptr<Peak> p){
    avgs.push_back(p->avg);
    stds.push_back(p->std);
  });
  WidthStatsStream stream(&test_out);

  // send a peak every second for PULSE_VALIDATION_WINDOW_MS*2 seconds
  for(long t = 0; t < PULSE_VALIDATION_WINDOW_MS*2; t+=1000) {
    auto peak = memstack.make();
    peak->t = t;
    peak->w = 2000;
    stream.push(peak);
  }

  for(float avg : avgs) {
    ASSERT(avg==2000, "Avg was %f, not 2000ms.", avg);
  }
  ASSERT(avgs.size() >= PULSE_VALIDATION_WINDOW_MS/1000, "Not enough avgs, %d", avgs.size());
  for(float std : stds) {
    ASSERT(std==0, "Std was not 0.");
  }
  ASSERT(avgs.size() == stds.size(), "Not enough stds, %d", stds.size());

  return true;
}

bool test_pulse_validation_stream() {
  Serial.println("Testing PulseValidationStream...");

  MemStack<Peak, 10> peak_memstack;
  MemStack<Pulse, 2> pulse_memstack;
  std::list<Pulse> pulses;
  TestStream<Pulse> test_out([&](std::shared_ptr<Pulse> p){
    p->next = nullptr;
    pulses.push_back(*p);
  });
  PulseValidationStream stream(&pulse_memstack, &test_out);

  // send a peak every second for 20 seconds
  for(int i = 0; i < 20; i++) {
    auto peak = peak_memstack.make();
    peak->t = i*1000L;
    peak->amp = 150;
    peak->w = 2000;
    peak->avg = 2000;
    peak->std = 0;
    stream.push(peak);
  }

  ASSERT(pulses.size() == 20, "Not enough valid pulses, %d", pulses.size());

  pulses.clear();
  char test_seq[] = "fvfqfvqfv";
  char exp_seq[] =  "fvfvfvvfv";
  for(int i = 0; i < sizeof(test_seq)-1; i++) {
    char v = test_seq[i];
    auto peak = peak_memstack.make();
    peak->t = i*1000L;
    peak->amp = v=='f' ? 50 : 150;
    peak->w = v=='v' ? 2000 : 1000;
    peak->avg = 2000;
    peak->std = 10;
    stream.push(peak);
  }
  
  ASSERT(pulses.size() == 5, "Expected 5 valid pulses, got %d", pulses.size());
  bool ac = true;
  for(int i = 0; i < sizeof(exp_seq)-1; i++) {
    if (exp_seq[i] == 'v') {
      Pulse p = pulses.front();
      pulses.pop_front();
      ASSERT_CONT(ac, p.t==i*1000L, "Expected peak %d to be marked valid.", i);
    }
  }
  return ac;
}

bool all_pulse_tests() {
  Serial.println("Running tests for \"pulse.h\\cpp\"...");

  ASSERT(test_ring_buffer(), "RingBuffer Failed");
  ASSERT(test_mem_stack(), "MemStack Failed");
  ASSERT(test_width_calc_stream(), "WidthCalcStream Failed");
  ASSERT(test_width_stats_stream(), "WidthStatsStream Failed");
  ASSERT(test_pulse_validation_stream(), "PulseValidationStream Failed");
  
  Serial.println("All tests pass!");
  return true;
}