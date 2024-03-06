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
#define PULSE_MAX_SUM_AGE 10000

struct HeartRate {
  long time; // time of measure, relative to system clock. millisecs
  float hr; // heart rate
  float hr_lb; // lower bound
  float hr_ub; // upper bound
  char err[40]; // error message, empty string if no error.
};

struct Peak {
  long t = -1; //time
  int amp = -1; //amplitude
  // width (next peak's time - previous peak's time) and
  // average and standard deviation of the width in relation to nearby (within ~PULSE_VALIDATION_WINDOW_MS/2) peaks.
  // if havent yet been calculated, they will be equal to -1
  long w = -1;
  float avg = -1;
  float std = -1;

  std::shared_ptr<Peak> next = nullptr;

  ~Peak() {
    next = nullptr;
  }
};

struct Pulse {
  long t = -1; // time
  float d = -1; // delta (time till the next valid pulse)

  std::shared_ptr<Pulse> next = nullptr;

  ~Pulse() {
    next = nullptr;
  }
};

template <typename T>
class RingBuffer {
  private:
    int h; //head
    int len; //length
    int cap; // capacity
    std::unique_ptr<T[]> buffer;
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
    T& push_back() {
      T& ret = buffer[(h+len)%cap];
      if (len == cap)
        h++;
      else
        len++;
      return ret;
    }
    T& back() {
      return buffer[(h+len-1)%cap];
    }
    T pop_front() {
      int temp = h;
      h = (h+1)%cap;
      len--;
      return buffer[temp];
    }
    int size() const { return len; }
    int capacity() const { return cap; }
    bool full() const { return len==cap; }
    bool empty() const { return len==0; }
};

template <typename T>
class Allocator {
  public:
    virtual std::shared_ptr<T> make() = 0;
};

template <typename T, unsigned int N>
class MemStack : public Allocator<T> {
  private:
    T mem[N];
    T *free[N];
    int n_free;
    struct MSDeleter {
      MemStack* const p;
      MSDeleter(MemStack* parent) : p(parent) {}
      void operator()(T* t) {
        t->~T();
        p->free[p->n_free] = t;
        p->n_free++;
      }
    };
  std::function<std::shared_ptr<T>()> steal = nullptr;
  public:
    MemStack() {
      for(int i = 0; i < N; i++)
        free[i] = &(mem[i]);
      n_free = N;
    }
    void set_steal(std::function<std::shared_ptr<T>()> f) {
      steal = f;
    }
    std::shared_ptr<T> make() override {
      if (n_free == 0) {
        #ifdef PULSE_DEBUG
        if(steal == nullptr)
          Serial.println("Error! No thief allocator in MemStack.");
        #endif
        std::shared_ptr<T> stolen = steal();
        #ifdef PULSE_DEBUG
        if(stolen.use_count() != 1)
          Serial.println("Error! Memory could not be free'd to make new in MemStack.");
        #endif
      }
      if (n_free == 0)
        ESP.restart();
      return std::shared_ptr<T>(free[--n_free], MSDeleter(this));
    }
    int num_free() {
      return n_free;
    }
};

template <typename T>
class PushTarget {
  public:
    virtual void push(T t) = 0;
};

template <typename T>
class LinkedProcessingStream : public PushTarget<std::shared_ptr<T>> {
  protected:
    const int num_heads;
    std::unique_ptr<std::shared_ptr<T>[]> heads;
    std::unique_ptr<int[]> sizes;
    void advance(int h) {
      heads[h] = heads[h]->next;
      if(h>0 && heads[h-1] != nullptr) // nullptr is possible when advancing with only 1 element
        sizes[h-1]++;
      if(h<num_heads-1) {
        sizes[h]--;
        // push forward the next head on collision
        if (sizes[h] <= 0)
          advance(h+1);
      }
    }
    virtual void after_push() = 0;
    virtual void before_pop() {};
  public:
    LinkedProcessingStream(int num_heads):
      num_heads(num_heads),
      heads(new std::shared_ptr<T>[num_heads]),
      sizes(new int[num_heads-1]) {
      #ifdef PULSE_DEBUG
      if (heads == nullptr)
        Serial.println("Error: linked processing stream 'heads' could not be allocated.");
      if (sizes == nullptr)
        Serial.println("Error: linked processing stream 'sizes' could not be allocated.");
      #endif
      for(int i = 0; i < num_heads; i++)
        heads[i] = nullptr;
      for(int i = 0; i < num_heads-1; i++)
        sizes[i] = 0;
    }
    void push(std::shared_ptr<T> p) override {
      #ifdef PULSE_DEBUG
      if (p==nullptr)
        Serial.println("Error: Recieved null push!");
      #endif
      if (heads[num_heads-1] == nullptr) {
        heads[num_heads-1] = p;
        for(int i = num_heads-2; i>=0; i--) {
          heads[i] = p;
          sizes[i] = 1;
        }
      } else {
        heads[num_heads-1]->next = p;
        heads[num_heads-1] = p;
        if(num_heads>=2)
          sizes[num_heads-2]++;
      }
      #ifdef PULSE_DEBUG
      for(int i = 0; i < num_heads; i++)
        if (heads[i] == nullptr)
          Serial.println("Error: Head null somehow!");
      #endif
      this->after_push();
    }
    std::shared_ptr<T> pop() {
      this->before_pop();
      std::shared_ptr<T> tail = heads[0];
      advance(0);
      return tail;
    }
};

class WidthCalcStream : public LinkedProcessingStream<Peak> {
  private:
    static constexpr int BACK = 0;
    static constexpr int FRONT = 1;
    PushTarget<std::shared_ptr<Peak>>* next_stream = nullptr;
    void after_push() override;
  public:
    WidthCalcStream(PushTarget<std::shared_ptr<Peak>>* next) : LinkedProcessingStream<Peak>(2), next_stream(next) {}
};

class WidthStatsStream : public LinkedProcessingStream<Peak> {
  private:
    static constexpr int BACK = 0;
    static constexpr int WRITE = 1;
    static constexpr int FRONT = 2;
    static constexpr int AVAILABLE = 3;
    long w_sum = 0, w2_sum = 0;
    PushTarget<std::shared_ptr<Peak>>* next_stream = nullptr;
    void after_push() override;
    void before_pop() override;
  public:
    WidthStatsStream(PushTarget<std::shared_ptr<Peak>>* next) : LinkedProcessingStream<Peak>(4), next_stream(next) {}
};

class PulseValidationStream : public LinkedProcessingStream<Peak> {
  private:
    static constexpr int BACK = 0;
    static constexpr int FRONT = 1;
    Allocator<Pulse>* pulse_allocator;
    PushTarget<std::shared_ptr<Pulse>>* next_stream = nullptr;
    void after_push() override;
  public:
    PulseValidationStream(Allocator<Pulse>* pulse_allocator, PushTarget<std::shared_ptr<Pulse>>* next):
      LinkedProcessingStream<Peak>(2),
      pulse_allocator(pulse_allocator),
      next_stream(next){}
};

class DeltaCalcStream : public LinkedProcessingStream<Pulse> {
  private:
    static constexpr int BACK = 0;
    static constexpr int FRONT = 1;
    PushTarget<std::shared_ptr<Pulse>>* next_stream = nullptr;
    void after_push() override;
  public:
    DeltaCalcStream(PushTarget<std::shared_ptr<Pulse>>* next) : LinkedProcessingStream<Pulse>(2), next_stream(next) {}
};

class PulseTrackerInternals {
  public:
    MemStack<Peak, PULSE_PEAKS_LEN> peak_mem;
    // record samples for long enough to calculate the slope accurately
    RingBuffer<int> pulse_signals;
    // calculates the slope and max of the current pulse_signals
    // should not be interrupted
    void slope_and_max(float* slope, int* max_index, int* max_amp);
    float last_slope = -1;
    // check to see if the latest pulse signal caused the slope to switch from
    // increasing to decreasing, and if so, push a peak on the stack
    bool detect_peak(long now);

    // Fast func to push a signal onto the buffer. Not safe to be interrupted.
    // Also calls all of the above update functions so that get_heartrate has
    // as little work to do as possible.
    void push(int pulse_signal, long time);
    // Safe to be interrupted
    void get_heartrate(HeartRate* out) const;

    PulseTrackerInternals() : pulse_signals(PULSE_SLOPE_WINDOW) {}
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