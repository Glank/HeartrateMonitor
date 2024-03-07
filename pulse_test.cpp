#include "pulse_test.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <list>
#include <algorithm>
#include "user_interface.h"

#define ASSERT(t, msg, ...) if(!(t)){Serial.print(__LINE__);Serial.print(": ");char l[128];sprintf(l, msg __VA_OPT__(,) __VA_ARGS__);Serial.println(l);return false;}
#define ASSERT_CONT(ac, t, msg, ...) if(!(t)){Serial.print(__LINE__);Serial.print(": ");char l[128];sprintf(l, msg __VA_OPT__(,) __VA_ARGS__);Serial.println(l);ac = false;}

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

bool test_delta_calc_stream() {
  Serial.println("Testing DeltaCalcStream...");

  MemStack<Pulse, 3> memstack;
  std::list<long> deltas;
  TestStream<Pulse> test_out([&](std::shared_ptr<Pulse> p){
    deltas.push_back(p->d);
  });
  DeltaCalcStream stream(&test_out);

  // send a pulse every second for 20 seconds
  for(int i = 0; i < 20; i++) {
    auto pulse = memstack.make();
    pulse->t = i*1000L;
    stream.push(pulse);
  }

  ASSERT(deltas.size() == 19, "Expected 19 deltas, got %d", deltas.size());
  for(long d : deltas) {
    ASSERT(d==1000, "Expected delta to be 1000.");
  }

  return true;
}

bool test_hr_calc_stream() {
  Serial.println("Testing HRCalcStream...");

  MemStack<Pulse, 2*(PULSE_HR_SAMPLE_WINDOW/1000+2)> memstack;
  MemStack<HeartRate, 3> hr_memstack;
  std::vector<HeartRate> hrs;
  TestStream<HeartRate> test_out([&](std::shared_ptr<HeartRate> hr){
    hrs.push_back(*hr);
  });
  HRCalcStream stream(&hr_memstack, &test_out);

  // send a pulse every second for a little longer than the sampling window
  long cur_time = 0;
  for(int i = 0; i-10 < PULSE_HR_SAMPLE_WINDOW/1000; i++) {
    auto pulse = memstack.make();
    pulse->t = cur_time;
    pulse->d = 1000L;
    stream.push(pulse);
    cur_time += pulse->d;
  }

  ASSERT(hrs.size() != 0, "Expected heartrates.");
  long last_time = -1;
  for(auto& hr : hrs) {
    ASSERT(last_time <= hr.time, "Expected times to be strictly increasing.");
    last_time = hr.time;
    ASSERT(hr.hr == 60, "Expected hr to be 60bpm, was %f", hr.hr);
    ASSERT(hr.hr_lb == 60, "Expected hr_lb to be 60bpm, was %f", hr.hr_lb);
    ASSERT(hr.hr_ub == 60, "Expected hr_ub to be 60bpm, was %f", hr.hr_ub);
    ASSERT(hr.err[0] == 0, "Expected no err, but got %s", hr.err);
  }

  hrs.clear();
  // send really noisy HRs for a little longer than the sampling window to fill the buffer with noisy data.
  for (int i = 0; i-10 < 2*PULSE_HR_SAMPLE_WINDOW/1000; i++){
    auto pulse = memstack.make();
    pulse->t = cur_time;
    pulse->d = i%2 == 0 ? 1000L : 10L;
    stream.push(pulse);
    cur_time +=  pulse->d;
  }

  // do it again, this time with fealing.
  hrs.clear();
  for (int i = 0; i-10 < 2*PULSE_HR_SAMPLE_WINDOW/1000; i++){
    auto pulse = memstack.make();
    pulse->t = cur_time;
    pulse->d = i%2 == 0 ? 1000L : 10L;
    stream.push(pulse);
    cur_time +=  pulse->d;
  }

  ASSERT(hrs.size() != 0, "Expected heartrates.");
  for(auto& hr : hrs) {
    ASSERT(last_time <= hr.time, "Expected times to still be strictly increasing.");
    last_time = hr.time;
    ASSERT(hr.hr > 60, "Expected hr to be > 60bpm, was %f", hr.hr);
    ASSERT(hr.hr_lb < hr.hr_ub, "Expected lb < ub, but had lb=%f and ub=%f", hr.hr_lb, hr.hr_ub);
    ASSERT(hr.err[0] != 0, "Expected err, but was no err");
  }

  return true;
}

bool test_pulse_tracker() {
  Serial.println("Testing PulseTrackerInternals...");
  const long expected_lag = PULSE_HR_SAMPLE_WINDOW+PULSE_VALIDATION_WINDOW_MS+PULSE_MAX_HR_STALENESS+500;

  // it's too big to allocate on the stack without triggering "stack smashing" error
  static PulseTrackerInternals pulse_tracker;
  // send a sin signal at 1hz til t=15s
  long t = 0;
  for(; t < 15*1000L; t+=PULSE_SAMPLE_RATE) {
    double sig = 100+100*sin(PI*2*t/1000.0);
    pulse_tracker.push((int)sig, t);
  }

  HeartRate hr;
  pulse_tracker.get_heartrate(&hr);
  bool ac = true;
  ASSERT_CONT(ac, t-hr.time<=expected_lag,
    "Expected hr lag to be <= expected_lag (%d), but was %d",
    expected_lag, t-hr.time);
  ASSERT_CONT(ac, hr.hr == 60, "Expected hr to be 60bpm, was %f", hr.hr);
  ASSERT_CONT(ac, hr.hr_lb == 60, "Expected hr_lb to be 60bpm, was %f", hr.hr_lb);
  ASSERT_CONT(ac, hr.hr_ub == 60, "Expected hr_ub to be 60bpm, was %f", hr.hr_ub);
  ASSERT_CONT(ac, hr.err[0] == 0, "Expected no err, but got %s", hr.err);
  ASSERT(ac, "HR failed at 15s");

  // send a sin signal at 1.25hz till t=30s
  for(; t < 30*1000L; t+=PULSE_SAMPLE_RATE) {
    double sig = 100+100*sin(1.25*PI*2*t/1000.0);
    pulse_tracker.push((int)sig, t);
  }

  pulse_tracker.get_heartrate(&hr);
  ASSERT_CONT(ac, t-hr.time<=expected_lag,
    "Expected hr lag to be <= expected_lag (%d), but was %d",
    expected_lag, t-hr.time);
  ASSERT_CONT(ac, hr.hr == 75, "Expected hr to be 75bpm, was %f", hr.hr);
  ASSERT_CONT(ac, hr.hr_lb == 75, "Expected hr_lb to be 75bpm, was %f", hr.hr_lb);
  ASSERT_CONT(ac, hr.hr_ub == 75, "Expected hr_ub to be 75bpm, was %f", hr.hr_ub);
  ASSERT_CONT(ac, hr.err[0] == 0, "Expected no err, but got %s", hr.err);
  ASSERT(ac, "HR failed at 30s");

  // send a sin signal at 1.25hz till t=45s
  // with spikes of noise near t=32 and t=38.5
  std::srand(3141);
  for(; t < 45*1000L; t+=PULSE_SAMPLE_RATE) {
    double sig = 100+100*sin(1.25*PI*2*t/1000.0);
    if(abs(t-32000L)<250 || abs(t-38500L)<250) {
      sig += 50*std::rand()/(double)(RAND_MAX+1)-75;
      sig = sig < 0 ? 0 : sig;
    }
    pulse_tracker.push((int)sig, t);
  }

  pulse_tracker.get_heartrate(&hr);
  ASSERT_CONT(ac, t-hr.time<=expected_lag,
    "Expected hr lag to be <= expected_lag (%d), but was %d",
    expected_lag, t-hr.time);
  ASSERT_CONT(ac, abs(hr.hr-75)<5, "Expected hr to be near 75bpm, was %f", hr.hr);
  ASSERT_CONT(ac, hr.hr_lb<hr.hr_ub, "Expected hr_lb (%f) to be less than hr_ub (%f)", hr.hr_lb, hr.hr_ub);
  ASSERT_CONT(ac, hr.err[0] == 0, "Expected no err, but got %s", hr.err);
  ASSERT(ac, "HR failed at 45s");

  // send random noise till t=60s
  std::srand(3141);
  for(; t < 60*1000L; t+=PULSE_SAMPLE_RATE) {
    double sig = 50+100*std::rand()/(double)(RAND_MAX+1);
    pulse_tracker.push((int)sig, t);
  }

  pulse_tracker.get_heartrate(&hr);
  ASSERT_CONT(ac, t-hr.time<=expected_lag,
    "Expected hr lag to be <= expected_lag (%d), but was %d",
    expected_lag, t-hr.time);
  ASSERT_CONT(ac, hr.hr_lb<hr.hr_ub, "Expected hr_lb (%f) to be less than hr_ub (%f)", hr.hr_lb, hr.hr_ub);
  ASSERT_CONT(ac, hr.err[0] != 0, "Expected an error, but there was none. hr_lb=%f, hr_ub=%f", hr.hr_lb, hr.hr_ub);
  ASSERT(ac, "HR failed at 60s");

  return ac;
}

bool all_pulse_tests() {
  Serial.println("Running tests for \"pulse.h\\cpp\"...");

  ASSERT(test_ring_buffer(), "RingBuffer Failed");
  ASSERT(test_mem_stack(), "MemStack Failed");
  ASSERT(test_width_calc_stream(), "WidthCalcStream Failed");
  ASSERT(test_width_stats_stream(), "WidthStatsStream Failed");
  ASSERT(test_pulse_validation_stream(), "PulseValidationStream Failed");
  ASSERT(test_delta_calc_stream(), "DeltaCalcStream Failed");
  ASSERT(test_hr_calc_stream(), "HRCalcStream Failed");
  ASSERT(test_pulse_tracker(), "PulseTrackerInternal Failed");
  
  Serial.println("All tests pass!");
  return true;
}