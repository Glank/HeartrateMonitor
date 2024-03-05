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

  /*
  Peak& peak = peaks.push_back();
  peak.t = now-(PULSE_SLOPE_WINDOW-max_i-1)*1000/PULSE_SAMPLE_RATE;
  peak.amp = max_amp;
  peak.w = -1;
  peak.avg = -1;
  peak.std = -1;
  peak.val = '_';
  peak.d = -1;
  */
  return true;
}
void WidthCalculatingStream::push(std::shared_ptr<Peak> p) {
  if (size==0) {
    tail = p;
    head = p;
    size = 1;
  } else {
    head->next = p;
    head = p;
    size++;
  }
  if (size>3) {
    tail = tail->next;
    size--;
  }

  #ifdef PULSE_DEBUG
  if (size > 3)
    Serial.println("Error: unexpected stream state, size>3");
  #endif

  if(size != 3)
    return;
  auto mid = tail->next;
  mid->w = head->t - tail->t;

  if(next_stream==nullptr)
    return;
  next_stream->push(mid);
}
std::shared_ptr<Peak> WidthCalculatingStream::pop() {
  if (size==0)
    return nullptr;
  size--;
  auto tmp = tail;
  tail = tail->next;
  if(size<=1)
    head = tail;
  return tmp;
}
void WidthCalculatingStream::set_next(PeakProcessingStream* n) {
  next_stream = n;
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