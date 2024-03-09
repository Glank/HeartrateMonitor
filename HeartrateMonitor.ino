#define TIMER_INTERRUPT_DEBUG 1
#define _TIMERINTERRUPT_LOGLEVEL_ 1

// Mode select macros:
//#define TEST
#define LOG_PULSE_DATA
#define LOG_HR_DATA
//#define HR_HUMAN_READABLE

#include "pulse.h"
#ifdef TEST
#include "pulse_test.h"

void setup() {
  LogBuffer::init_global(512);
  Serial.begin(460800);
  Serial.println("\n\n");
 
  all_pulse_tests();
}

void loop() {
  delay(100);
}

#else

#include "ESP8266TimerInterrupt.h"
#include "logbuffer.h"

// use the least accurate timer interrupt for pulse
// tracking (doesn't need super high resolution)
#define USING_TIM_DIV256 true

#define PULSE_SENSOR_INPUT_PIN A0
#define LED_PIN 2

#define PULSE_TIMER_INTERVAL_MICROSECS (1000000L/PULSE_SAMPLE_RATE)
ESP8266Timer pulse_timer;
PulseTracker pulse_tracker;

void sample_pulse() {
  int pulse_signal = analogRead(PULSE_SENSOR_INPUT_PIN);
  long now = millis();
  pulse_tracker.push(pulse_signal, now);

  #ifdef LOG_PULSE_DATA
    LOG(30, "p,%d,%d", now, pulse_signal);
  #endif
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  LogBuffer::init_global(512);
  Serial.begin(460800);
  Serial.println("\n\n");

  if(!pulse_timer.attachInterruptInterval(PULSE_TIMER_INTERVAL_MICROSECS, sample_pulse)) {
    Serial.print("Failed to attach timer.");
    while(true);
  }
}

long last_hr_time = 0;
void loop() {
  #ifdef LOG_HR_DATA
    long now = millis();
    if (now-last_hr_time>=1000) {
      last_hr_time = now;
      HeartRate hr;
      pulse_tracker.get_heartrate(&hr);
      // disable logging durring interrupts
      #ifdef HR_HUMAN_READABLE
        Serial.println();
        if (hr.err[0]==0) {
          LOG(30, "hr: %f [%f - %f]", hr.hr, hr.hr_lb, hr.hr_ub);
        } else {
          LOG(120, "Error: %s", hr.err);
        }
      #else
        LOG(128, "hr,%d,%f,%f,%f,%s", hr.time, hr.hr, hr.hr_lb, hr.hr_ub, hr.err);
      #endif
    }
  #endif
  FLUSH_LOG_TO_SERIAL();
}

#endif