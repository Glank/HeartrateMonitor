#ifndef PULSE_H
#define PULSE_H

#include <functional>
#include <memory>
#include <stdlib.h>

#define PULSE_DEBUG
#ifdef PULSE_DEBUG
#include <Arduino.h>
#endif

#define PULSE_SAMPLE_RATE 40  // samples per second
#define PULSE_SLOPE_WINDOW_MS 225
#define PULSE_SLOPE_WINDOW (PULSE_SLOPE_WINDOW_MS*PULSE_SAMPLE_RATE/1000) // in num samples
#define PULSE_PEAKS_LEN (15*250*3/(2*60)) // enough to cover about 15s of pulses at 250bpm, with an additiopnal 50% false pulses
#define PULSE_VALIDATION_WINDOW_MS (10000) // 10s
#define PULSE_MAX_SMART_SUM_AGE 10000

struct HeartRate {
  long time; // time of measure, relative to system clock. millisecs
  float hr; // heart rate
  float hr_lb; // lower bound
  float hr_ub; // upper bound
  char err[40]; // error message, empty string if no error.
};

struct Peak {
  long t; //time
  int amp; //amplitude
  // width (next peak's time - previous peak's time) and
  // average and standard deviation of the width in relation to nearby (within ~PULSE_VALIDATION_WINDOW_MS/2) peaks.
  // if havent yet been calculated, they will be equal to -1
  float w, avg, std;
  char val; //validation state:
    // '_' = unvalidated
    // '?' = potentially a false pulse
    // 'f' = definitely a false pulse
    // 'v' = valid pulse
  float d; // delta (time till the next valid pulse)
};

template <typename T> class RingBuffer;
template <typename T> class RBStream;

template <typename T>
class SyncedIndex {
  private:
    int i;
    const int buf_cap;
    std::function<void()> before_increment = nullptr;
    SyncedIndex& operator=(SyncedIndex& other) {
      #ifdef PULSE_DEBUG
      if (other.buf_cap != buf_cap)
        Serial.println("Error: Trying to copy-assign mismatched SyncedIndex buf_caps!");
      #endif
      i = other.i;
    }
  public:
    SyncedIndex(int i, int buf_cap) : buf_cap(buf_cap) {
      i = i;
    }
    void set_before_increment(std::function<void()> f) {
      before_increment = f;
    }
    SyncedIndex& operator++() {
      if (before_increment != nullptr)
        before_increment();
      i = (i+1)%buf_cap;
      return *this;
    }
    bool operator==(const SyncedIndex& other) {
      #ifdef PULSE_DEBUG
      if (other.buf_cap != buf_cap)
        Serial.println("Error: Trying to compare mismatched SyncedIndex buf_caps!");
      #endif
      return i == other.i;
    }
    int operator-(const SyncedIndex& other) const {
      #ifdef PULSE_DEBUG
      if (other.buf_cap != buf_cap)
        Serial.println("Error: Trying to subtract mismatched SyncedIndex buf_caps!");
      #endif
      return (buf_cap+i-other.i)%buf_cap;
    }
  friend RingBuffer<T>;
  friend RBStream<T>;
};

template <typename T>
class RBStream {
  protected:
    std::vector<std::unique_ptr<SyncedIndex<T>>> heads;
    RingBuffer<T>* const parent;
  public:
    RBStream(RingBuffer<T>* parent, int n_heads) : parent(parent) {
      heads.reserve(n_heads);
      for(int i = 0; i < n_heads; i++)
        heads.push_back(std::make_unique<SyncedIndex<T>>(parent->h, parent->cap));
    }
    void inc(int h) {
      ++(*(heads[h]));
    }
    SyncedIndex<T>& head(int h) {
      return *(heads[h]);
    }
    T& at(int h) {
      return (*parent)[head(h)];
    }
    T& at(SyncedIndex<T>& i) {
      return (*parent)[i];
    }
};

// A 3 headed stream that tries to position 3 heads (tail, mid, and front)
// to satisfy the constraints specified in front_is_balanced(), mid_is_balanced(),
// and back_is_balanced().
// Implementation Note:
// Once balanced, the mid should *stay* balanced, because skipping the mid head forward
// is treated as a missed result, skipping the output stream forward to compensate.
// Implementations should never manually increment the heads, that's all handled by the
// automatic balancing logic and the write() function.
template <typename T>
class BalancedStream : public RBStream<T> {
  private:
    BalancedStream<T>* output = nullptr;
  public:
    virtual bool front_is_blanced() { return true; }
    virtual bool mid_is_balanced() { return true; }
    virtual bool back_is_balanced() { return true; }
    virtual void after_push_when_balanced() = 0;
    virtual void before_back_increment() {};
    SyncedIndex<T>& back() { return this->head(0); }
    SyncedIndex<T>& mid() { return this->head(1); }
    SyncedIndex<T>& front() { return this->head(2); }
    BalancedStream(RingBuffer<T>* parent) : RBStream<T>(parent, 3) {
      // ensure that the mid head stays at or ahead of the back head
      // and the front head stays ahead of the mid head
      back().set_before_increment([&]{
        this->before_back_increment();
        if(mid() == back())
          ++mid();
      });
      mid().set_before_increment([&]{
        if(front() == mid())
          ++front();
      });
    }
    void set_output(BalancedStream<T>* out) {
      output = out;
    }
  protected:
    // moves the mid forward and pushes to output.
    void write() {
      ++mid();
      output->push();
    }
  public:
    void skip() {
      ++front();
      ++mid();
      ++back();
    }
    void push() {
      ++front();
      while(back() != mid() && !this->back_is_balanced()) {
        ++back();
      }
      while(mid() != front() && !this->mid_is_balanced()) {
        ++mid();
        output->skip();
      }
      if (!this->back_is_balanced() || !this->mid_is_balanced() || !this->front_is_balanced())
        return;
      this->after_push_when_balanced();
    }
};

template <typename T>
class RingBuffer {
  private:
    int h; //head
    int len; //length
    int cap; // capacity
    std::unique_ptr<T[]> buffer;
    std::vector<std::unique_ptr<RBStream<T>>> streams;
  protected:
    virtual void on_advance(T& drop) {
      for(auto& s : streams) {
        if((s->head(0).i+1)%cap == h)
          s->inc(0);
      }
    }
  public:
    RingBuffer(int capacity) : buffer(new T[capacity]) {
      h = 0;
      len =  0;
      cap = capacity;
    }
    ~RingBuffer() = default;
    T& operator[]( const int i ) const {
      return buffer[(h+i)%cap];
    }
    T& operator[]( const SyncedIndex<T>& idx ) const {
      return buffer[idx.i];
    }
    T& push_back() {
      T& ret = buffer[(h+len)%cap];
      if (len == cap) {
        h++;
        on_advance(ret);
      }
      else
        len++;
      return ret;
    }
    T& back() {
      return buffer[(h+len-1)%cap];
    }
    int size() const { return len; }
    int capacity() const { return cap; }
    bool full() const { return len==cap; }
    RBStream<T>* new_stream(int heads) {
      streams.push_back(std::make_unique<RBStream<T>>(this, heads));
      return streams.back().get();
    }
    int relative(const SyncedIndex<T>& idx) const {
      // h+r = i (mod cap)
      // r = i-h (mod cap)
      return (cap+idx.i-h)%cap;
    }
  friend RBStream<T>;
};

template <typename T, typename S>
class SumStream : BalancedStream<T> {
  S sum;
  int age;
  public:
    virtual S value(T& elem) = 0;
    SumStream(RingBuffer<T>* parent) : BalancedStream<T>(parent), sum(0) {
      this->front().set_before_increment([&]{
        sum = sum + this->value(this->at(this->front()));
        age++;
      });
    }
    void before_back_increment() override {
      sum = sum - this->value(this->at(this->back()));
      age++;
    }
};

class PeakBuffer : public RingBuffer<Peak> {
  private:
    struct SumWindow {
      std::function<float(Peak&)> value;
      // inclusive
      int head, tail;
      int lifetime;
      float sum;
    };
    std::vector<int*> smart_indexes;
    std::vector<std::unique_ptr<SumWindow>> sum_windows;
    const std::function<int()> rand_int;
  protected:
    void on_advance(Peak& drop) {
      for (int* si : smart_indexes) {
        (*si)--;
      }
      for(auto& swp : sum_windows) {
        auto& sw = *swp;
        if (sw.tail < 0 && sw.head >= 0) {
          sw.sum -= sw.value(drop);
          sw.lifetime--;
        }
      }
    }
  public:
    PeakBuffer(int capacity=PULSE_PEAKS_LEN, std::function<int()> rand_generator=rand)
      : RingBuffer<Peak>(capacity), rand_int(rand_generator) {}
    // TODO: move these function defs to pulse.cpp
    void add_smart_index(int* index) {
      smart_indexes.push_back(index);
    }
    int register_smart_sum(std::function<float(Peak&)> value) {
      std::unique_ptr<SumWindow> sw(new SumWindow());
      sw->head = -1;
      sw->tail = -1;
      sw->lifetime = 0;
      sw->sum = 0;
      sw->value = value;
      add_smart_index(&sw->head);
      add_smart_index(&sw->tail);
      sum_windows.push_back(std::move(sw));
      return sum_windows.size()-1;
    }
    // key is the return from register_smart_average
    // start is inclusive
    // end is exclusive
    float calc_smart_sum(int key, int start, int end) {
      if (start == end)
        return 0;
      SumWindow& sw = *sum_windows[key];
      // recalculate when the sum gets too old to account for accumulated floating point errors.
      if (sw.lifetime <= 0) {
        // use a pseudo-random lifetime to prevent having to
        // recalculate all of these at the same time.
        sw.lifetime = rand_int()%PULSE_MAX_SMART_SUM_AGE;
        sw.head = end-1;
        sw.tail = start;
        sw.sum = 0;
        for (int i = start; i < end; i++) {
          sw.sum += sw.value((*this)[i]);
        }
        return sw.sum;
      }
      // match the tail
      while(sw.tail < start) {
        if(sw.tail>=0) {
          sw.sum -= sw.value((*this)[sw.tail]);
          sw.lifetime--;
        }
        sw.tail++;
      }
      while(sw.tail > start) {
        sw.tail--;
        sw.sum += sw.value((*this)[sw.tail]);
        sw.lifetime--;
      }
      // match the head
      int new_head = end-1;
      while(sw.head > new_head) {
        sw.sum -= sw.value((*this)[sw.head]);
        sw.lifetime--;
        sw.head--;
      }
      while(sw.head < new_head) {
        sw.head++;
        sw.sum += sw.value((*this)[sw.head]);
        sw.lifetime--;
      }
      return sw.sum;
    }
};

class PulseTrackerInternals {
  public:
    // record samples for long enough to calculate the slope accurately
    RingBuffer<int> pulse_signals;
    // calculates the slope and max of the current pulse_signals
    // should not be interrupted
    void slope_and_max(float* slope, int* max_index, int* max_amp);
    float last_slope = -1;
    // check to see if the latest pulse signal caused the slope to switch from
    // increasing to decreasing, and if so, push a peak on the stack
    bool detect_peak(long now);

    PeakBuffer peaks;
    RingBuffer<HeartRate> hr_swap_buf;
    // pointers to various bits of work that need to be done on Peaks
    int widths_head = 0; // updated in update_widths
    int stats_head = 0; // updated in peaks.on_advance & update_stats
    int stats_tail = 0; // updated in peaks.on_advance & update_stats
    int inspection_head = 0; // updated in peaks.on_advance & inspection_pulse
    int resolution_head = 0; // updated in peaks.on_advance & resolve_questionable
    int resolution_tail = 0; // updated in peaks.on_advance & resolve_questionable
    int deltas_head = 0; // updated in peaks.on_advance & update_deltas
    // keys for smart sum in peaks
    int widths_sum, widths2_sum, delta_count, delta_sum, delta2_sum;
    // four resonably complex clean-up steps that are split up because
    // they operate at different points on the peak buffer, and should be
    // separately tested
    // The ones with boolean returns should be repeated until they return false;
    void update_widths();
    bool update_stats();
    bool inspect_pulse();
    bool resolve_questionable();
    bool update_deltas();
    void update_hr();

    // Fast func to push a signal onto the buffer. Not safe to be interrupted.
    // Also calls all of the above update functions so that get_heartrate has
    // as little work to do as possible.
    void push(int pulse_signal, long time);
    // Safe to be interrupted
    void get_heartrate(HeartRate* out) const;

    PulseTrackerInternals() : pulse_signals(PULSE_SLOPE_WINDOW), hr_swap_buf(2) {
      peaks.add_smart_index(&stats_head);
      peaks.add_smart_index(&stats_tail);
      peaks.add_smart_index(&inspection_head);
      peaks.add_smart_index(&resolution_head);
      peaks.add_smart_index(&resolution_tail);
      peaks.add_smart_index(&deltas_head);
      widths_sum = peaks.register_smart_sum([](Peak& p){return p.w;});
      widths2_sum = peaks.register_smart_sum([](Peak& p){return p.w*p.w;});
      delta_count = peaks.register_smart_sum([](Peak& p){return p.d<0?0:1;});
      delta_sum = peaks.register_smart_sum([](Peak& p){return p.d<0?0:p.d;});
      delta2_sum = peaks.register_smart_sum([](Peak& p){return p.d<0?0:p.d*p.d;});
    }
};

// public wrapper of PulseTrackerInternals
class PulseTracker {
  private:
    PulseTrackerInternals internals;
  public:
    // Fast func to push a signal onto the buffer. Not safe to be interrupted.
    void push(int pulse_signal, long time) { internals.push(pulse_signal, time); };
    // Safe to be interrupted
    void get_heartrate(HeartRate* out) const { internals.get_heartrate(out); };
};

#endif