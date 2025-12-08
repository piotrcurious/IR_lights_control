#include "slow_pwm.h"

// Non-blocking software PWM implementation for Arduino.
// This implementation uses micros() for timing to avoid blocking the main loop.

#define MAX_PWM_CHANNELS 8

struct PwmChannel {
  uint8_t pin;
  uint8_t value;    // 0-255 duty cycle
  bool active;
  bool is_on;      // Tracks if the pin is currently HIGH
};

PwmChannel channels[MAX_PWM_CHANNELS];
int pwm_frequency = 100;
unsigned long pwm_period_micros;
unsigned long last_cycle_start_micros = 0;

void slow_pwm_init(int frequency) {
  pwm_frequency = frequency > 0 ? frequency : 100;
  pwm_period_micros = 1000000UL / pwm_frequency;
  for(int i=0; i < MAX_PWM_CHANNELS; ++i) {
    channels[i].active = false;
  }
}

void slow_pwm_write(uint8_t pin, uint8_t value) {
  int free_channel = -1;
  for(int i=0; i < MAX_PWM_CHANNELS; ++i) {
    if(channels[i].pin == pin && channels[i].active) {
      channels[i].value = value;
      if (value == 0 && channels[i].is_on) {
        digitalWrite(pin, LOW);
        channels[i].is_on = false;
      }
      return;
    }
    if(!channels[i].active && free_channel == -1) {
      free_channel = i;
    }
  }

  if(free_channel != -1) {
    pinMode(pin, OUTPUT);
    channels[free_channel].pin = pin;
    channels[free_channel].value = value;
    channels[free_channel].active = true;
    channels[free_channel].is_on = false;
    if (value == 0) {
       digitalWrite(pin, LOW);
    }
  }
}

void slow_pwm_update() {
  unsigned long now = micros();
  unsigned long elapsed_micros = now - last_cycle_start_micros;

  if (elapsed_micros >= pwm_period_micros) {
    last_cycle_start_micros = now;
    elapsed_micros = 0;

    for(int i=0; i < MAX_PWM_CHANNELS; ++i) {
      if(channels[i].active) {
        if (channels[i].value > 0) {
          digitalWrite(channels[i].pin, HIGH);
          channels[i].is_on = true;
        } else if (channels[i].is_on) {
          digitalWrite(channels[i].pin, LOW);
          channels[i].is_on = false;
        }
      }
    }
  }

  for(int i=0; i < MAX_PWM_CHANNELS; ++i) {
    if(channels[i].active && channels[i].is_on) {
      unsigned long turn_off_micros = (pwm_period_micros * channels[i].value) / 255;
      if(elapsed_micros >= turn_off_micros) {
        digitalWrite(channels[i].pin, LOW);
        channels[i].is_on = false;
      }
    }
  }
}
