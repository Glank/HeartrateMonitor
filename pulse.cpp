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

  Peak& peak = peaks.push_back();
  peak.t = now-(PULSE_SLOPE_WINDOW-max_i-1)*1000/PULSE_SAMPLE_RATE;
  peak.amp = max_amp;
  peak.w = -1;
  peak.avg = -1;
  peak.std = -1;
  peak.val = '_';
  peak.d = -1;
  return true;
}
void PulseTrackerInternals::update_widths() {
  widths_head = peaks.size()-2;
  if(peaks.size() < 3) {
    // we need at least 3 peaks to calculate the width
    peaks.back().w = 0;
    return;
  }
  peaks[widths_head].w = peaks[widths_head+1].t-peaks[widths_head-1].t;
}
bool PulseTrackerInternals::update_stats() {
  // "validation window" = the time > PULSE_VALIDATION_WINDOW_MS around the peak at stats_head
  // In practice, the validation window should go from [stats_tail to widths_head-1] with
  // stats_head approximately in the middle.
  // This function works as follows:
  //   First, we check to see if we have enouogh lead time in the buffer, or if we need to wait for more pulses to come in.
  //   Then, we snug up the tail so that the validation window is as small as possible while still longer than specified
  //   Then we calculate the average and standard deviation within the window and save it to the stats_head peak.
  // Warning: stats_head and stats_tail are smart indices that are automatically decremented every time peaks' head moves.
  if (stats_head<0)
    stats_head = 0;
  if (stats_tail<0)
    stats_tail = 0;
  // if we can't fit the validation window around the stats head and have only peaks with measured widths,
  // we can't update
  if(stats_head >= peaks.size()
    || widths_head-1 < 0
    || (peaks[widths_head-1].t-peaks[stats_head].t)<PULSE_VALIDATION_WINDOW_MS/2) {
    return false;
  }
  
  // move the stats_tail forward in time if there is slack in the validation window
  while(stats_tail < stats_head && (peaks[stats_head].t-peaks[stats_tail+1].t)>PULSE_VALIDATION_WINDOW_MS/2) {
    stats_tail++;
  }

  // If the stats tail is too close, we can't calculate the stats.
  // This should only happen while we don't have enough peaks, or stats_head has fallen behind
  // because of a flood of high frequency peaks.
  if (peaks[stats_head].t-peaks[stats_tail].t < PULSE_VALIDATION_WINDOW_MS/2) {
    #ifdef PULSE_DEBUG
    if (peaks.full() && stats_tail > 0)
      Serial.println("Error: Stats tail moved too close!");
    #endif
    stats_head++;
    return false;
  }

  float avg = peaks.calc_smart_sum(widths_sum, stats_tail, widths_head)/(widths_head-stats_tail);
  float avg2 = peaks.calc_smart_sum(widths2_sum, stats_tail, widths_head)/(widths_head-stats_tail);
  peaks[stats_head].avg = avg;
  peaks[stats_head].std = sqrt(avg2-avg*avg);
  stats_head++;
  return true;
}
bool PulseTrackerInternals::inspect_pulse() {
  if (inspection_head < 0)
    inspection_head = 0;
  Peak& p = peaks[inspection_head];
  if (p.avg == -1) {
    if (inspection_head >= stats_head-1) {
      // caught up to stat's head
      return false;
    }
    inspection_head++;
    return true;
  }
  if(p.std != 0 && (p.w-p.avg)/p.std < -1 && p.w/p.avg < 0.7)
    p.val = '?';
  else
    p.val = 'v';

  inspection_head++;
  return true;
}
bool PulseTrackerInternals::resolve_questionable() {
  if (resolution_head < 0)
    resolution_head = 0;
  if (resolution_tail < 0)
    resolution_tail = 0;
  if (resolution_head >= peaks.size()) {
    // edge case in the very begining of processing
    return false;
  }
  if (peaks[resolution_head].val == '_') {
    if (resolution_head >= inspection_head-1) {
      // caught up to inspeciton head
      return false;
    }
    resolution_head++;
    resolution_tail=resolution_head;
    return true;
  }
  if (peaks[resolution_head].val == 'v' && resolution_tail == resolution_head) {
    // nothing to resolve, continue
    resolution_head++;
    resolution_tail++;
    return true;
  }
  if (peaks[resolution_head].val == '?') {
    // leave the tail at it's current position to capture this questionable group
    resolution_head++;
    return true;
  }
  // at this point we can assume we're at the end of a questionable group
  #ifdef PULSE_DEBUG
  if (peaks[resolution_head].val != 'v' || resolution_head <= resolution_tail) {
    char l[128];
    sprintf(l, "Error: Invalid stream state in resolve_questionable! head: %d, tail: %d, val_at_head: %c",
      resolution_head, resolution_tail, peaks[resolution_head].val);
    Serial.println(l);
  }
  #endif

  int num_questionable = resolution_head-resolution_tail;

  // if we just have an isolated questionable pulse, just mark it as false and move on
  if (num_questionable == 1) {
    peaks[resolution_tail].val = 'f';
    resolution_head++;
    resolution_tail = resolution_head;
    return true;
  }
  
  // otherwise, calculate the average even and odd amplitudes
  // and mark the set with the smaller average as false
  float avg_e = 0;
  float avg_o = 0;
  for (int i = 0; i < num_questionable; i++){
    if(i%2==0)
      avg_e += peaks[resolution_tail+i].amp;
    else
      avg_o += peaks[resolution_tail+i].amp;
  }
  avg_e /= (int)((num_questionable+1)/2);
  avg_o /= (int)(num_questionable/2);
  int valid_parity = avg_e < avg_o;
  for (int i = 0; i < num_questionable; i++) {
    peaks[resolution_tail+i].val = (i%2 == valid_parity) ? 'v' : 'f';
  }
  
  resolution_head++;
  resolution_tail = resolution_head;
  return true;
}
bool PulseTrackerInternals::update_deltas() {
  if (deltas_head < 0)
    deltas_head = 0;
  // waiting for more.
  if (deltas_head <= peaks.size() || deltas_head+1 >= resolution_tail)
    return false;
  // TODO
  return false;
}
void PulseTrackerInternals::update_hr() {
  // TODO
}
void PulseTrackerInternals::push(int pulse_signal, long time) {
  pulse_signals.push_back() = pulse_signal;
  if(!detect_peak(time))
    return;
  update_widths();
  while(update_stats());
  while(inspect_pulse());
  while(resolve_questionable());
}
void PulseTrackerInternals::get_heartrate(HeartRate* out) const {
  out->time = -1;
  out->hr = -1;
  out->hr_lb = -1;
  out->hr_ub = -1;
  strcpy(out->err,"");
  // TODO
}