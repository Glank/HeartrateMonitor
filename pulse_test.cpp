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

bool test_stream() {
  Serial.println("Testing stream...");
  RingBuffer<int> buf(5);
  RBStream<int>* stream = buf.new_stream(2);
  stream->head(0).set_before_increment([=]{
    if(stream->head(0) == stream->head(1))
      stream->inc(1);
  });
  buf.push_back() = -3;
  ASSERT(stream->at(0) == -3, "value at head 0 is %d, not -3", stream->at(0));
  ASSERT(stream->at(1) == -3, "value at head 1 is %d, not -3", stream->at(1));
  stream->inc(1);
  int d = stream->head(1) - stream->head(0);
  ASSERT(d == 1, "Difference between heads is %d, not 1", d);
  for(int i = 0; i < 5; i++)
    buf.push_back() = i;
  ASSERT(stream->at(0) == buf[0], "value at head 0 is %d, not %d", stream->at(0), buf[0]);
  d = stream->head(1) - stream->head(0);
  ASSERT(d == 0, "Difference between heads is %d, not 0", d);
  ASSERT(stream->at(1) == buf[0], "value at head 1 is %d, not %d", stream->at(1), buf[0]);
  return true;
}

bool test_peak_buffer() {
  Serial.println("Testing PeakBuffer...");
  PeakBuffer buf(5, []{
    return 5; // chosen by a fair dice throw, guaranteed to be random
  });
  int indirect;
  buf.add_smart_index(&indirect);
  buf.push_back().t = -2;
  buf.push_back().t = -3;
  indirect = buf.size()-1; // point to the last element
  ASSERT(buf[indirect].t == -3, "Logic error in peak_buffer test.");
  for(int i = 0; i < 4; i++)
    buf.push_back().t = i;
  ASSERT(buf[indirect].t == -3, "Indirect smart index (%d) not tracking value", indirect);

  int time_sum = buf.register_smart_sum([](Peak& p){return p.t;});
  for(int i = 0; i < buf.capacity(); i++)
    buf.push_back().t = i;

  float sum = buf.calc_smart_sum(time_sum, 0, 5);
  ASSERT(sum == 10, "Smart sum of {0,1,2,3,4} != 10");
  // run out the lifetime of the average to force a recalc
  for(int i = 0; i < 10; i++) {
    int end = 1+i%4;
    float exp_sum = end*(end-1)/2;
    sum = buf.calc_smart_sum(time_sum, 0, end);
    ASSERT(sum == exp_sum, "Smart avg from [0,%d) = %f and not %f", end, sum, exp_sum);
  }

  // check that the avg window stay's updated with buffer overflows
  sum = buf.calc_smart_sum(time_sum, 0, 5);
  ASSERT(sum == 10, "Smart sum of {0,1,2,3,4} != 10");
  buf.push_back().t = 5;
  buf.push_back().t = 6;
  sum = buf.calc_smart_sum(time_sum, 0, 5);
  ASSERT(sum == 20, "Smart sum of {2,3,4,5,6} != 20");

  return true;
}

bool test_peak_detection() {
  Serial.println("Testing peak detection...");
  PulseTrackerInternals tracker;

  // just fill up the buffer so we can set it manually
  while(!tracker.pulse_signals.full())
    tracker.pulse_signals.push_back() = 0;

  // init with a peak at PULSE_SLOPE_WINDOW/2-1
  tracker.last_slope = 1;
  int expected_max_i = PULSE_SLOPE_WINDOW/2-2;
  int expected_max = 104;
  // simple saw tooth peak
  for(int i = 0; i<=expected_max_i; i++)
    tracker.pulse_signals[expected_max_i-i] = expected_max-i;
  for(int i = 0; i+expected_max_i < tracker.pulse_signals.size(); i++)
    tracker.pulse_signals[i+expected_max_i] = expected_max-2*i;
  double frame_duration = 1000.0/PULSE_SAMPLE_RATE;
  double current_time = frame_duration*(tracker.pulse_signals.size()-1);
  double expected_max_time = frame_duration*expected_max_i;

  ASSERT(tracker.detect_peak(current_time), "Peak not detected.");

  ASSERT(tracker.peaks.size()==1, "peaks.size() != 1");
  double time_err = abs(tracker.peaks[0].t-expected_max_time);
  ASSERT(time_err < 5, "peak.t is not within 5ms of expected_max_time, time_err:%f", time_err);
  ASSERT(tracker.peaks[0].amp == expected_max, "peaks.amp=%d and not expected_max", tracker.peaks[0].amp);
  return true;
}

bool test_update_peak_stats() {
  Serial.println("Testing peak stats updater...");

  PulseTrackerInternals tracker;

  // add a peak every second until the tracker overflows twice
  int past_full = 0;
  for(long t = 0; past_full < 2; t+=1000) {
    if (tracker.peaks.full())
      past_full++;
    auto& p = tracker.peaks.push_back();
    p.t = t;
    p.w = -1;
    p.amp = -1;
    p.avg = -1;
    p.std = -1;
    p.val = 0;
    p.d = -1;
    tracker.update_widths();
    while(tracker.update_stats());
  }

  for (int i = 0; i < tracker.peaks.size()-1; i++) {
    ASSERT(tracker.peaks[i].w == 2000, "peak[%d].w != 2000", i);
  }

  // expect that there will be a block of no averages, then a block of averages, then another block of no averages
  int num_changeover = 0;
  bool last_had_avg = false;
  int num_avgs = 0;
  for (int i = 0; i < tracker.peaks.size(); i++) {
    bool has_avg = tracker.peaks[i].avg != -1;
    if (has_avg != last_had_avg)
      num_changeover++;
    last_had_avg = has_avg;
    if (!has_avg)
      continue;
    num_avgs++;
    if (num_avgs <= 2) {
      // The first two stats are a little fuzzy cus the first peaks have 0 widths
      ASSERT(abs(tracker.peaks[i].avg-2000)<250, "peak[%d].avg = %f, and not near-ish 2000", i, tracker.peaks[i].avg);
      ASSERT(tracker.peaks[i].std<1000, "peak[%d].std = %f, and not < 1000", i, tracker.peaks[i].std);
    } else {
      ASSERT(tracker.peaks[i].avg == 2000, "peak[%d].avg = %f, and not 2000", i, tracker.peaks[i].avg);
      ASSERT(tracker.peaks[i].std == 0, "peak[%d].std = %f, and not 0", i, tracker.peaks[i].std);
    }
  }

  ASSERT(num_changeover == 2 && !last_had_avg,
    "Unexpected pattern of recorded averages: %d changeovers%s", 
    num_changeover, last_had_avg?" and had an average at the end":"");

  return true;
}

bool test_inspect_pulses() {
  Serial.println("Testing inspect pulses...");
  PulseTrackerInternals tracker;

  // plan out pulses, evenly spaced at 1000ms intervals, except for those at fp_times
  std::vector<long> times;
  for(int i = 0; i < tracker.peaks.capacity()+2; i++) {
    times.push_back(i*1000);
  }
  std::vector<long> fp_times {17100, 34700};
  for (long t : fp_times)
    times.push_back(t);
  std::sort(times.begin(), times.end());

  // add the pulses & do the processing stpes to allow for inspection
  for(long t: times) {
    auto& p = tracker.peaks.push_back();
    p.t = t;
    p.w = -1;
    p.amp = -1;
    p.avg = -1;
    p.std = -1;
    p.val = 0;
    tracker.update_widths();
    while(tracker.update_stats());
    while(tracker.inspect_pulse());
  }

  int questionable_count = 0;
  for(int i = 0; i < tracker.peaks.size(); i++) {
    auto& p = tracker.peaks[i];
    bool is_fp_time = std::find(fp_times.begin(), fp_times.end(), p.t) != fp_times.end();
    if (is_fp_time) {
      ASSERT(p.val == '?', "Peak at %d is not marked questionable.", p.t);
    }
    if(p.val == '?')
      questionable_count++;
  }
  ASSERT(questionable_count <= 3*fp_times.size(),
    "More questionable peaks than expected: %d", questionable_count);
  ASSERT(questionable_count >= fp_times.size(),
    "Fewer questionable peaks than expected: %d", questionable_count);

  return true;
}

bool test_resolve_questionable() {
  Serial.println("Testing resovle_questionable...");
  
  PulseTrackerInternals tracker;

                          // 0123456789012345678901
  char input_validation[] = "___vvvv?vv???vv??v____";
  char exp_validation[] =   "___vvvvfvvfvfvvvfv____";
  int n = sizeof(input_validation)-1;
  for(int i = 0; i < n; i++) {
    auto& p = tracker.peaks.push_back();
    p.val = input_validation[i];
    p.amp = i%2==0 ? 2 : 11; // the evens will be the actual false pulses
    if(i+1 < sizeof(input_validation) && (input_validation[i] != '_' || i<3))
      tracker.inspection_head = i+1;
    while(tracker.resolve_questionable());
  }
  
  bool ac = true;
  for(int i = 0; i < n; i++) {
    ASSERT_CONT(ac, tracker.peaks[i].val == exp_validation[i],
      "Peak at %d with amp %d was marked '%c' and not '%c'.",
      i, tracker.peaks[i].amp, tracker.peaks[i].val, exp_validation[i]);
  }
  return ac;
}

bool all_pulse_tests() {
  Serial.println("Running tests for \"pulse.h\\cpp\"...");

  ASSERT(test_ring_buffer(), "RingBuffer Failed");
  ASSERT(test_stream(), "Test Stream Failed");
  ASSERT(test_peak_buffer(), "PeakBuffer Failed");
  ASSERT(test_peak_detection(), "Peak Detection Failed");
  ASSERT(test_update_peak_stats(), "Peak Stats Update Failed");
  ASSERT(test_inspect_pulses(), "Inspecting Pulses Failed");
  ASSERT(test_resolve_questionable(), "Resolving Questionable Pulses Failed");
  
  Serial.println("All tests pass!");
  return true;
}