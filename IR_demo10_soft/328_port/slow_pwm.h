#ifndef SLOW_PWM_H
#define SLOW_PWM_H

#include <Arduino.h>

// Initializes the software PWM system with a given frequency.
void slow_pwm_init(int frequency);

// Sets the PWM duty cycle for a specific pin.
void slow_pwm_write(uint8_t pin, uint8_t value);

// Updates all software PWM channels. This should be called in the main loop.
void slow_pwm_update();

#endif // SLOW_PWM_H
