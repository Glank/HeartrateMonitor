#include "pulse.h"

#include <cstring>
#include <math.h>

void WidthCalcStream::after_push() {
  if(sizes[BACK] > 3) {
    advance(BACK);
  }
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
    if (sizes[FRONT] <= 1 || heads[FRONT]->t - heads[WRITE]->t < PULSE_VALIDATION_WINDOW_MS/2) {
      return; // not enough lead time to calculate the avg and std
    }

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
  auto& f = heads[FRONT];
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
  if(sizes[BACK] < 2)
    return;
  #ifdef PULSE_DEBUG
  if (sizes[BACK] != 2)
    Serial.println("Error: Invalid DeltaCalcStream state, size > 2");
  #endif
  heads[BACK]->d = heads[FRONT]->t - heads[BACK]->t;
  next_stream->push(heads[BACK]);
  advance(BACK);
}

void HRCalcStream::after_push() {
  if (sizes[BACK] < 2)
    return;
  // snug up the calculation window
  while (sizes[BACK] > 0 && heads[FRONT]->t - heads[BACK]->next->t > PULSE_HR_SAMPLE_WINDOW)
    advance(BACK);
  // this operation is a little expensive, so only do it as frequently as necessary
  if(heads[FRONT]->t-last_calc_time < PULSE_MAX_HR_STALENESS) {
    return;
  }
  last_calc_time = heads[FRONT]->t;

  long full_delta = heads[FRONT]->t - heads[BACK]->t;

  // not enough pulse data to calculate HR
  if (full_delta < PULSE_HR_SAMPLE_WINDOW) {
    return;
  }
  
  // OPT: this could be done in O(1) with occasional O(n) floating-point error fixes
  float d_avg = 0, d2_avg = 0;
  for(auto cur = heads[BACK]; cur != heads[FRONT]->next; cur = cur->next) {
    d_avg += cur->d;
    d2_avg += cur->d*cur->d;
  }
  d_avg /= sizes[BACK];
  d2_avg /= sizes[BACK];
  float std = sqrt(d2_avg - d_avg*d_avg)/sizes[BACK];
  std::shared_ptr<HeartRate> hr = hr_allocator->make();
  hr->time = heads[BACK]->t+full_delta/2;
  hr->hr = 60000.0/d_avg;
  hr->hr_lb = 60000.0/(d_avg+2*std);
  hr->hr_ub = 60000.0/(d_avg-2*std);
  float range = hr->hr_ub - hr->hr_lb;
  if (range > PULSE_MAX_ABSOLUTE_HR_VARIANCE || range/hr->hr > PULSE_MAX_PERCENT_HR_VARIANCE) {
    strcpy(hr->err, "Variance Too High");
  } else {
    hr->err[0] = 0;
  }
  next_stream->push(hr);
}
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
void PulseTrackerInternals::detect_peak(long now) {
  if (!pulse_signals.full())
    return;
  float slope = 0;
  int max_i = 0;
  int max_amp = 0;
  slope_and_max(&slope, &max_i, &max_amp);
 
  bool maximum = last_slope > 0 && slope <= 0;
  last_slope = slope;
  if (!maximum)
    return;

  std::shared_ptr<Peak> peak = peak_mem.make();
  peak->t = now+(max_i*PULSE_SLOPE_WINDOW_MS/PULSE_SLOPE_WINDOW);
  peak->amp = max_amp;
  width_calc_stream.push(peak);
}
void PulseTrackerInternals::push(std::shared_ptr<HeartRate> hr) {
  cur_hr = hr;
}
void PulseTrackerInternals::push(int pulse_signal, long time) {
  pulse_signals.push_back() = pulse_signal;
  detect_peak(time);
}
void PulseTrackerInternals::get_heartrate(HeartRate* out) const {
  noInterrupts();
  if (cur_hr != nullptr) {
    *out = *cur_hr;
    return;
  }
  interrupts();
  out->time = -1;
  out->hr = -1;
  out->hr_lb = -1;
  out->hr_ub = -1;
  strcpy(out->err, "No HR yet.");
}