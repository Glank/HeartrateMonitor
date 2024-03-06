#include "pulse.h"

#include <cstring>
#include <math.h>

void PulseTrackerInternals::slope_and_max(float* slope, int* max_index, int* max_amp) {
  // OPT: could make this O(1) except with occasional fp-err fixes,
  // but PULSE_SLOPE_WINDOW is small, so that's unnecessary for now.
  float avgp = 0;
  for(int i = 0; i < pulse_signals.size(); i++)
    avgp += pulse_signals[i];
  avgp /= pulse_signals.size();
  const float avgi = (pulse_signals.size()-1)/2.0;
  float sip = 0;
  int max = pulse_signals[0];
  int max_i = 0;
  for(int i = 0; i < pulse_signals.size(); i++) {
    int p = pulse_signals[i];
    sip += (i-avgi)*(p-avgp);
    if (max < p) {
      max = p;
      max_i = i;
    }
  }
  (*slope) = sip; // didnt divide by Sii because we don't care about the scale factor of the slope, just the sign
  (*max_index) = max_i;
  (*max_amp) = max;
}
bool PulseTrackerInternals::detect_peak(long now) {
  if (!pulse_signals.full())
    return false;
  float slope;
  int max_i;
  int max_amp;
  slope_and_max(&slope, &max_i, &max_amp);
 
  bool maximum = last_slope > 0 && slope <= 0;
  last_slope = slope;
  if (!maximum)
    return false;
  return true;
}

void WidthCalcStream::after_push() {
  if(sizes[BACK] > 3)
    advance(BACK);
  #ifdef PULSE_DEBUG
  if (sizes[BACK] > 3)
    Serial.println("Error: invalid WidthCalcStream state, more than 3 pulses");
  #endif
  if(sizes[BACK] != 3)
    return;
  auto mid = heads[BACK]->next;
  mid->w = heads[FRONT]->t - heads[BACK]->t;

  if(next_stream==nullptr)
    return;
  next_stream->push(mid);
}

void WidthStatsStream::after_push() {
  if(heads[AVAILABLE]==heads[BACK]) {
    // first push, need to init w_sum and w2_sum
    w_sum += heads[FRONT]->w;
    w2_sum += heads[FRONT]->w*heads[FRONT]->w;
  }
  // while there is enough lead time to calculate the next avg and std
  while(true) {
    // snug up the back half of the window if necessary
    while (sizes[BACK]>1 && heads[WRITE]->t - heads[BACK]->next->t > PULSE_VALIDATION_WINDOW_MS/2) {
      w_sum -= heads[BACK]->w;
      w2_sum -= heads[BACK]->w*heads[BACK]->w;
      advance(BACK);
    }
    // expand the front half of the window as necessary
    while (sizes[FRONT] > 1 && heads[FRONT]->t - heads[WRITE]->t < PULSE_VALIDATION_WINDOW_MS/2) {
      advance(FRONT);
      w_sum += heads[FRONT]->w;
      w2_sum += heads[FRONT]->w*heads[FRONT]->w;
    }
    if (heads[FRONT]->t - heads[WRITE]->t < PULSE_VALIDATION_WINDOW_MS/2)
      return; // not enough lead time to calculate the avg and std

    // write the stats
    int n = sizes[BACK]+sizes[WRITE]-1; // -1 is because the write head gets counted twice
    float avg = w_sum/n;
    float avg2 = w2_sum/n;
    heads[WRITE]->avg = avg;
    heads[WRITE]->std = sqrt(avg2-avg*avg);
    next_stream->push(heads[WRITE]);
    advance(WRITE);
  }
}
void WidthStatsStream::before_pop() {
  if (sizes[BACK] > 0) {
    w_sum -= heads[BACK]->w;
    w2_sum -= heads[BACK]->w*heads[BACK]->w;
  }
}

void PulseValidationStream::after_push() {
  auto f = heads[FRONT];
  bool assumed_valid = (f->w-f->avg)/f->std > -1 || f->w/f->avg >= 0.7;
  if (!assumed_valid)
    return; // wait for valid to resolve questionable

  if (sizes[BACK] <= 2) {
    std::shared_ptr<Pulse> pulse = pulse_allocator->make();
    pulse->t = heads[FRONT]->t;
    next_stream->push(pulse);
    while(heads[FRONT] != nullptr)
      advance(BACK);
    return;
  }

  float e_avg = 0, o_avg = 0;
  
  int i = 0;
  for (auto cur = heads[BACK]; cur != f; cur = cur->next) {
    if (i%2 == 0)
      e_avg += cur->amp;
    else
      o_avg += cur->amp;
    i++;
  }
  e_avg /= sizes[BACK]/2;
  o_avg /= (sizes[BACK]-1)/2;
  int valid_parity = e_avg < o_avg;
  for (i = 0; heads[BACK] != f; i++) {
    if (i%2 == valid_parity) {
      std::shared_ptr<Pulse> pulse = pulse_allocator->make();
      pulse->t = heads[BACK]->t;
      next_stream->push(pulse);
    }
    advance(BACK);
  }
  std::shared_ptr<Pulse> pulse = pulse_allocator->make();
  pulse->t = heads[FRONT]->t;
  next_stream->push(pulse);
  advance(BACK);
}

void DeltaCalcStream::after_push() {
  // TODO
}

void PulseTrackerInternals::push(int pulse_signal, long time) {
  pulse_signals.push_back() = pulse_signal;
  if(!detect_peak(time))
    return;
  //TODO
}
void PulseTrackerInternals::get_heartrate(HeartRate* out) const {
  out->time = -1;
  out->hr = -1;
  out->hr_lb = -1;
  out->hr_ub = -1;
  strcpy(out->err,"");
  // TODO
}