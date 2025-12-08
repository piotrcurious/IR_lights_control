// Arduino Uno port of IR_demo10_soft/13.ino
// Implements a non-blocking software PWM library for LED control.
// IR receiver on pin 2; Samsung protocol address 0x7 only.

#include <Arduino.h>
#include <IRremote.h>

// This is a placeholder for the user-specified software PWM library.
// The actual implementation will depend on the library's API.
// For this port, we assume the following functions exist:
// void slow_pwm_write(uint8_t pin, uint8_t value);
// void slow_pwm_update();
#include "slow_pwm.h" // Assumed header for the library

// -------------------- Configuration --------------------

namespace Config {
  constexpr uint8_t IR_PIN = 2;
  // Arduino Uno has PWM on pins 3, 5, 6, 9, 10, 11
  constexpr uint8_t PWM_PINS[6] = {3, 5, 6, 9, 10, 11};
  constexpr uint8_t CHANNEL_COUNT = 6; // Reduced from 8 due to pin limitations

  constexpr float MIN_EFFECT_FREQ = 0.1f;
  constexpr float MAX_EFFECT_FREQ = 50.0f;
  constexpr float FREQ_STEP_RATIO = 1.1f;
  constexpr float DEFAULT_DUTY_CYCLE = 50.0f;
  constexpr float DUTY_CYCLE_STEP = 5.0f;

  constexpr uint32_t FADE_STEP_MS = 20;
  constexpr uint32_t FLASH_INTERVAL_MS = 80;
  constexpr uint8_t FLASH_CYCLES = 3;
  constexpr uint8_t BRIGHTNESS_STEPS = 16;
}

// -------------------- Types & Enums --------------------

enum class EffectState : uint8_t {
  IDLE, PULSE_ON, PULSE_OFF, FADE_UP, FADE_DOWN,
  BREATHING, STROBE_ON, STROBE_OFF, RUNNING
};

enum class EffectType : uint8_t {
  NONE, PULSE, FADE_IN_OUT, FADE_IN, FADE_OUT, BREATHING,
  STROBE, HEARTBEAT, RANDOM_FLASH, SAWTOOTH_UP, SAWTOOTH_DOWN,
  TRIANGLE, CANDLE_FLICKER, SMOOTH_RANDOM
};

enum class EffectMode : uint8_t {
  LOOPED, ONE_SHOT
};

struct Effect {
  EffectType type = EffectType::NONE;
  EffectState state = EffectState::IDLE;
  EffectMode mode = EffectMode::LOOPED;

  float frequency = 1.0f;
  float dutyCycle = Config::DEFAULT_DUTY_CYCLE;
  uint32_t duration = 500;
  uint8_t savedBrightness = 0; // Changed to uint8_t
  uint32_t onTime = 0;
  uint32_t offTime = 0;
  uint32_t lastStepTime = 0;
  uint32_t nextUpdateMs = 0;

  uint8_t linkedChannel = 0xFF;
  EffectType nextEffectType = EffectType::PULSE;
  uint32_t cycleCount = 0;
  uint32_t maxCycles = 0;

  uint8_t randomTarget = 0; // Changed to uint8_t
  uint32_t randomDuration = 0;
  uint32_t phaseOffset = 0;
  uint8_t flickerIntensity = 50;

  void validate() {
    if (frequency <= 0.0f) frequency = 1.0f;
    if (dutyCycle <= 0.0f || dutyCycle > 100.0f) dutyCycle = Config::DEFAULT_DUTY_CYCLE;
    if (duration == 0) duration = 500;
    frequency = constrain(frequency, Config::MIN_EFFECT_FREQ, Config::MAX_EFFECT_FREQ);
    dutyCycle = constrain(dutyCycle, 1.0f, 99.0f);
    flickerIntensity = constrain(flickerIntensity, 0, 100);
  }

  void reset() {
    state = EffectState::IDLE;
    lastStepTime = 0;
    nextUpdateMs = 0;
    cycleCount = 0;
    randomTarget = 0;
    randomDuration = 0;
    phaseOffset = 0;
  }

  bool isActive() const {
    return type != EffectType::NONE && state != EffectState::IDLE;
  }

  bool shouldLoop() const {
    return mode == EffectMode::LOOPED || (maxCycles > 0 && cycleCount < maxCycles);
  }
};

struct FlashState {
  bool active = false;
  bool isBright = false;
  bool prevWasFading = false;
  uint8_t savedValue = 0; // Changed to uint8_t
  uint32_t nextToggleMs = 0;
  uint8_t channel = 0xFF;
  uint8_t blinksRemaining = 0;
};

namespace RemoteKeys {
  enum KeyCode : uint8_t {
    POWER = 0xE6,
    KEY_0 = 0x11, KEY_1 = 0x04, KEY_2 = 0x05, KEY_3 = 0x06,
    KEY_4 = 0x08, KEY_5 = 0x09, KEY_6 = 0x0A, KEY_7 = 0x0C,
    KEY_8 = 0x0D, KEY_9 = 0x0E,
    UP = 0x60, DOWN = 0x61, LEFT = 0x65, RIGHT = 0x62,
    OK = 0x68, MENU = 0x79, INFO = 0x1F,
    RED = 0x6C, GREEN = 0x14, YELLOW = 0x15, BLUE = 0x16,
    VOL_UP = 0x07, VOL_DOWN = 0x0B,
    CH_UP = 0x12, CH_DOWN = 0x10, CH_LIST = 0x6B,
    PLAY = 0x47, PAUSE = 0x4A, STOP = 0x46,
    REWIND = 0x45, FORWARD = 0x48,
    MUTE = 0x0F, SETTINGS = 0x1A, SUBTITLES = 0x25,
    SOURCE = 0x01, GUIDE = 0x4F,
    NETFLIX = 0xF3, PRIME_VIDEO = 0xF4
  };
}

// -------------------- Global State --------------------

class LEDController {
private:
  Effect effects[Config::CHANNEL_COUNT];
  uint8_t brightness[Config::CHANNEL_COUNT] = {0}; // Changed to uint8_t

  int selectedIndex = 0;
  bool isChaining = false;
  int sourceChainChannel = -1;

  FlashState flashState;

  char logBuffer[128]; // Buffer for formatted strings

  // -------------------- Utility Methods --------------------

  void softPwmWrite(uint8_t channel, uint8_t duty) {
    if (channel >= Config::CHANNEL_COUNT) return;
    // Assumes slow_pwm library takes the PIN number.
    slow_pwm_write(Config::PWM_PINS[channel], duty);
  }

  static float easeInOutQuad(float t) {
    return t < 0.5f ? 2.0f * t * t : 1.0f - 2.0f * (1.0f - t) * (1.0f - t);
  }

  static float easeInOutCubic(float t) {
    return t < 0.5f ? 4.0f * t * t * t : 1.0f - 4.0f * (1.0f - t) * (1.0f - t) * (1.0f - t);
  }

  static const char* effectTypeToString(EffectType type) {
    switch (type) {
      case EffectType::NONE: return "None";
      case EffectType::PULSE: return "Pulse";
      case EffectType::FADE_IN_OUT: return "Fade In/Out";
      case EffectType::FADE_IN: return "Fade In";
      case EffectType::FADE_OUT: return "Fade Out";
      case EffectType::BREATHING: return "Breathing";
      case EffectType::STROBE: return "Strobe";
      case EffectType::HEARTBEAT: return "Heartbeat";
      case EffectType::RANDOM_FLASH: return "Random Flash";
      case EffectType::SAWTOOTH_UP: return "Sawtooth Up";
      case EffectType::SAWTOOTH_DOWN: return "Sawtooth Down";
      case EffectType::TRIANGLE: return "Triangle";
      case EffectType::CANDLE_FLICKER: return "Candle Flicker";
      case EffectType::SMOOTH_RANDOM: return "Smooth Random";
      default: return "Unknown";
    }
  }

  static const char* effectModeToString(EffectMode mode) {
    return (mode == EffectMode::LOOPED) ? "Looped" : "One-Shot";
  }

  // -------------------- Effect Management --------------------

  void calculateEffectTiming(Effect &e) {
    e.validate();
    const float period = 1000.0f / e.frequency;

    switch (e.type) {
      case EffectType::PULSE:
      case EffectType::HEARTBEAT: {
        const float duty = e.dutyCycle / 100.0f;
        e.onTime = max(1U, (uint32_t)(period * duty + 0.5f));
        e.offTime = max(1U, (uint32_t)(period * (1.0f - duty) + 0.5f));
        break;
      }
      case EffectType::STROBE:
        e.onTime = 50;
        e.offTime = max(50U, (uint32_t)(period - 50));
        break;
      case EffectType::FADE_IN_OUT:
      case EffectType::FADE_IN:
      case EffectType::FADE_OUT:
      case EffectType::BREATHING:
      case EffectType::SAWTOOTH_UP:
      case EffectType::SAWTOOTH_DOWN:
      case EffectType::TRIANGLE:
        e.duration = max(1U, (uint32_t)(period / 2.0f + 0.5f));
        break;
      case EffectType::RANDOM_FLASH:
      case EffectType::CANDLE_FLICKER:
        e.duration = max(50U, (uint32_t)(period + 0.5f));
        break;
      case EffectType::SMOOTH_RANDOM:
        e.duration = max(100U, (uint32_t)(period + 0.5f));
        break;
      default:
        break;
    }
  }

  void startEffect(uint8_t channel, EffectType type, EffectMode mode) {
    if (channel >= Config::CHANNEL_COUNT) return;

    Effect &e = effects[channel];
    if (e.isActive()) stopEffect(channel, false);

    e.type = type;
    e.mode = mode;
    e.savedBrightness = brightness[channel];
    e.cycleCount = 0;

    calculateEffectTiming(e);
    e.lastStepTime = millis();
    e.nextUpdateMs = millis();

    switch (type) {
      case EffectType::PULSE:
        e.state = EffectState::PULSE_ON;
        softPwmWrite(channel, e.savedBrightness);
        break;
      case EffectType::FADE_IN_OUT:
      case EffectType::FADE_IN:
      case EffectType::TRIANGLE:
        e.state = EffectState::FADE_UP;
        softPwmWrite(channel, 0);
        break;
      case EffectType::FADE_OUT:
      case EffectType::SAWTOOTH_DOWN:
        e.state = EffectState::FADE_DOWN;
        softPwmWrite(channel, e.savedBrightness);
        break;
      case EffectType::BREATHING:
        e.state = EffectState::BREATHING;
        softPwmWrite(channel, 0);
        break;
      case EffectType::STROBE:
        e.state = EffectState::STROBE_ON;
        softPwmWrite(channel, e.savedBrightness);
        break;
      case EffectType::HEARTBEAT:
        e.state = EffectState::PULSE_ON;
        e.phaseOffset = 0;
        softPwmWrite(channel, e.savedBrightness);
        break;
      case EffectType::RANDOM_FLASH:
      case EffectType::CANDLE_FLICKER:
      case EffectType::SMOOTH_RANDOM:
        e.state = EffectState::RUNNING;
        e.randomTarget = random(0, e.savedBrightness + 1);
        softPwmWrite(channel, e.randomTarget);
        break;
      case EffectType::SAWTOOTH_UP:
        e.state = EffectState::FADE_UP;
        softPwmWrite(channel, 0);
        break;
      default:
        e.state = EffectState::IDLE;
        break;
    }

    snprintf(logBuffer, sizeof(logBuffer), "Ch%u: %s [%s] %.2fHz", channel,
             effectTypeToString(type), effectModeToString(mode), e.frequency);
    Serial.println(logBuffer);
  }

  void stopEffect(uint8_t channel, bool restoreBrightness = true) {
    if (channel >= Config::CHANNEL_COUNT) return;

    Effect &e = effects[channel];

    if (restoreBrightness) {
      brightness[channel] = e.savedBrightness;
      softPwmWrite(channel, brightness[channel]);
    }

    e.state = EffectState::IDLE;
    e.type = EffectType::NONE;
    e.cycleCount = 0;
  }

  void toggleEffectMode(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;

    Effect &e = effects[channel];
    e.mode = (e.mode == EffectMode::LOOPED) ? EffectMode::ONE_SHOT : EffectMode::LOOPED;

    snprintf(logBuffer, sizeof(logBuffer), "Ch%u: Mode -> %s", channel, effectModeToString(e.mode));
    Serial.println(logBuffer);

    if (e.isActive()) {
      EffectType currentType = e.type;
      startEffect(channel, currentType, e.mode);
    }
  }

  void cycleEffect(uint8_t channel, int direction) {
    if (channel >= Config::CHANNEL_COUNT) return;

    int effectNum = (int)effects[channel].type;
    effectNum += direction;

    if (effectNum > (int)EffectType::SMOOTH_RANDOM) effectNum = (int)EffectType::NONE;
    if (effectNum < (int)EffectType::NONE) effectNum = (int)EffectType::SMOOTH_RANDOM;

    EffectType newType = (EffectType)effectNum;

    if (newType == EffectType::NONE) {
      stopEffect(channel);
    } else {
      startEffect(channel, newType, effects[channel].mode);
    }
  }

  void handleEffectCompletion(uint8_t channel, Effect &e) {
    const uint8_t linked = e.linkedChannel;
    const EffectType next = e.nextEffectType;

    snprintf(logBuffer, sizeof(logBuffer), "Ch%u: Completed (cycles=%lu)", channel, (unsigned long)e.cycleCount);
    Serial.println(logBuffer);

    stopEffect(channel, true);

    if (linked < Config::CHANNEL_COUNT) {
      snprintf(logBuffer, sizeof(logBuffer), "Ch%u -> Ch%u: %s", channel, linked, effectTypeToString(next));
      Serial.println(logBuffer);
      startEffect(linked, next, EffectMode::ONE_SHOT);
    }
  }

  // -------------------- Effect Update Functions --------------------

  void updatePulseEffect(uint8_t channel, Effect &e, uint32_t now) {
    if (e.state == EffectState::PULSE_ON) {
      if (now - e.lastStepTime >= e.onTime) {
        e.lastStepTime = now;
        e.state = EffectState::PULSE_OFF;
        softPwmWrite(channel, 0);
      }
    } else if (e.state == EffectState::PULSE_OFF) {
      if (now - e.lastStepTime >= e.offTime) {
        e.lastStepTime = now;
        e.cycleCount++;

        if (e.mode == EffectMode::ONE_SHOT ||
            (e.maxCycles > 0 && e.cycleCount >= e.maxCycles)) {
          handleEffectCompletion(channel, e);
          return;
        }

        e.state = EffectState::PULSE_ON;
        softPwmWrite(channel, e.savedBrightness);
      }
    }
  }

  void updateStrobeEffect(uint8_t channel, Effect &e, uint32_t now) {
    if (e.state == EffectState::STROBE_ON) {
      if (now - e.lastStepTime >= e.onTime) {
        e.lastStepTime = now;
        e.state = EffectState::STROBE_OFF;
        softPwmWrite(channel, 0);
      }
    } else if (e.state == EffectState::STROBE_OFF) {
      if (now - e.lastStepTime >= e.offTime) {
        e.lastStepTime = now;
        e.cycleCount++;

        if (e.mode == EffectMode::ONE_SHOT) {
          handleEffectCompletion(channel, e);
          return;
        }

        e.state = EffectState::STROBE_ON;
        softPwmWrite(channel, e.savedBrightness);
      }
    }
  }

  void updateHeartbeatEffect(uint8_t channel, Effect &e, uint32_t now) {
    const uint32_t beatDuration = e.onTime;
    const uint32_t pauseDuration = e.offTime;

    if (e.phaseOffset == 0) {
      if (e.state == EffectState::PULSE_ON && now - e.lastStepTime >= beatDuration) {
        e.lastStepTime = now;
        e.state = EffectState::PULSE_OFF;
        e.phaseOffset = 1;
        softPwmWrite(channel, 0);
      } else if (e.state == EffectState::PULSE_OFF && now - e.lastStepTime >= beatDuration / 2) {
        e.lastStepTime = now;
        e.state = EffectState::PULSE_ON;
        e.phaseOffset = 2;
        softPwmWrite(channel, e.savedBrightness);
      }
    } else if (e.phaseOffset == 2) {
      if (now - e.lastStepTime >= beatDuration) {
        e.lastStepTime = now;
        e.phaseOffset = 3;
        softPwmWrite(channel, 0);
      }
    } else if (e.phaseOffset == 3) {
      if (now - e.lastStepTime >= pauseDuration) {
        e.lastStepTime = now;
        e.phaseOffset = 0;
        e.state = EffectState::PULSE_ON;
        e.cycleCount++;

        if (e.mode == EffectMode::ONE_SHOT) {
          handleEffectCompletion(channel, e);
          return;
        }

        softPwmWrite(channel, e.savedBrightness);
      }
    }
  }

  void updateFadeEffect(uint8_t channel, Effect &e, uint32_t now) {
    if ((int32_t)(now - e.nextUpdateMs) < 0) return;
    e.nextUpdateMs = now + Config::FADE_STEP_MS;

    const uint32_t elapsed = now - e.lastStepTime;
    const uint32_t dur = e.duration;
    const float progress = min((float)elapsed / dur, 1.0f);

    if (progress >= 1.0f) {
      bool effectComplete = false;

      if (e.type == EffectType::FADE_IN) {
        softPwmWrite(channel, e.savedBrightness);
        effectComplete = true;
      } else if (e.type == EffectType::FADE_OUT) {
        softPwmWrite(channel, 0);
        effectComplete = true;
      } else if (e.type == EffectType::FADE_IN_OUT || e.type == EffectType::TRIANGLE) {
        e.lastStepTime = now;
        if (e.state == EffectState::FADE_UP) {
          softPwmWrite(channel, e.savedBrightness);
          e.state = EffectState::FADE_DOWN;
        } else {
          softPwmWrite(channel, 0);
          e.state = EffectState::FADE_UP;
          e.cycleCount++;
          if (e.mode == EffectMode::ONE_SHOT) effectComplete = true;
        }
      } else if (e.type == EffectType::SAWTOOTH_UP) {
        softPwmWrite(channel, 0);
        e.lastStepTime = now;
        e.cycleCount++;
        if (e.mode == EffectMode::ONE_SHOT) effectComplete = true;
      } else if (e.type == EffectType::SAWTOOTH_DOWN) {
        softPwmWrite(channel, e.savedBrightness);
        e.lastStepTime = now;
        e.cycleCount++;
        if (e.mode == EffectMode::ONE_SHOT) effectComplete = true;
      }

      if (effectComplete) {
        handleEffectCompletion(channel, e);
      }
    } else {
      const uint8_t target = e.savedBrightness;
      uint8_t duty = 0;

      if (e.state == EffectState::FADE_UP) {
        duty = (uint8_t)(target * progress + 0.5f);
      } else {
        duty = (uint8_t)(target * (1.0f - progress) + 0.5f);
      }

      softPwmWrite(channel, duty);
    }
  }

  void updateBreathingEffect(uint8_t channel, Effect &e, uint32_t now) {
    if ((int32_t)(now - e.nextUpdateMs) < 0) return;
    e.nextUpdateMs = now + Config::FADE_STEP_MS;

    const uint32_t elapsed = now - e.lastStepTime;
    const uint32_t totalDuration = e.duration * 2;
    const float progress = min((float)elapsed / totalDuration, 1.0f);

    if (progress >= 1.0f) {
      e.lastStepTime = now;
      e.cycleCount++;

      if (e.mode == EffectMode::ONE_SHOT) {
        handleEffectCompletion(channel, e);
        return;
      }
    }

    const float easedProgress = easeInOutCubic(progress < 0.5f ? progress * 2.0f : (1.0f - progress) * 2.0f);
    const uint8_t duty = (uint8_t)(e.savedBrightness * easedProgress + 0.5f);
    softPwmWrite(channel, duty);
  }

  void updateRandomFlashEffect(uint8_t channel, Effect &e, uint32_t now) {
    if ((int32_t)(now - e.nextUpdateMs) < 0) return;

    const uint32_t nextInterval = random(50, e.duration);
    e.nextUpdateMs = now + nextInterval;

    e.randomTarget = random(0, e.savedBrightness + 1);
    softPwmWrite(channel, e.randomTarget);

    e.cycleCount++;
    if (e.mode == EffectMode::ONE_SHOT && e.cycleCount >= 10) {
      handleEffectCompletion(channel, e);
    }
  }

  void updateCandleFlickerEffect(uint8_t channel, Effect &e, uint32_t now) {
    if ((int32_t)(now - e.nextUpdateMs) < 0) return;

    e.nextUpdateMs = now + random(30, 100);

    const float flickerAmount = e.flickerIntensity / 100.0f;
    const uint8_t baseLevel = (uint8_t)(e.savedBrightness * (1.0f - flickerAmount * 0.5f));
    const uint8_t flickerRange = (uint8_t)(e.savedBrightness * flickerAmount);

    if (random(100) < 5) {
      e.randomTarget = baseLevel - random(flickerRange);
    } else {
      e.randomTarget = baseLevel + random(flickerRange / 2);
    }

    e.randomTarget = constrain(e.randomTarget, 0, e.savedBrightness);
    softPwmWrite(channel, e.randomTarget);
  }

  void updateSmoothRandomEffect(uint8_t channel, Effect &e, uint32_t now) {
    // This effect requires reading the current duty cycle, which is not
    // available with a simple software PWM library. This effect is disabled.
    if(e.state != EffectState::IDLE) stopEffect(channel);
  }

  void updateEffects() {
    const uint32_t now = millis();

    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      Effect &e = effects[i];

      if (!e.isActive()) continue;

      switch (e.type) {
        case EffectType::PULSE:
          updatePulseEffect(i, e, now);
          break;
        case EffectType::STROBE:
          updateStrobeEffect(i, e, now);
          break;
        case EffectType::HEARTBEAT:
          updateHeartbeatEffect(i, e, now);
          break;
        case EffectType::FADE_IN_OUT:
        case EffectType::FADE_IN:
        case EffectType::FADE_OUT:
        case EffectType::TRIANGLE:
        case EffectType::SAWTOOTH_UP:
        case EffectType::SAWTOOTH_DOWN:
          updateFadeEffect(i, e, now);
          break;
        case EffectType::BREATHING:
          updateBreathingEffect(i, e, now);
          break;
        case EffectType::RANDOM_FLASH:
          updateRandomFlashEffect(i, e, now);
          break;
        case EffectType::CANDLE_FLICKER:
          updateCandleFlickerEffect(i, e, now);
          break;
        case EffectType::SMOOTH_RANDOM:
          // Disabled for this port.
          break;
        default:
          break;
      }
    }
  }

  // -------------------- Visual Feedback --------------------

  void startFlash(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;

    if (flashState.active && flashState.channel < Config::CHANNEL_COUNT) {
      softPwmWrite(flashState.channel, flashState.savedValue);
      if (flashState.prevWasFading) {
        startEffect(flashState.channel, EffectType::FADE_IN_OUT, EffectMode::LOOPED);
      }
    }

    flashState.prevWasFading = effects[channel].isActive();
    if (flashState.prevWasFading) {
      effects[channel].state = EffectState::IDLE;
    }

    flashState.channel = channel;
    flashState.savedValue = brightness[channel];
    flashState.active = true;
    flashState.blinksRemaining = Config::FLASH_CYCLES * 2;
    flashState.isBright = true;
    flashState.nextToggleMs = millis() + Config::FLASH_INTERVAL_MS;
    softPwmWrite(channel, 255);
  }

  void updateFlash() {
    if (!flashState.active) return;
    if ((int32_t)(millis() - flashState.nextToggleMs) < 0) return;

    const uint8_t ch = flashState.channel;

    if (flashState.blinksRemaining > 0) {
      flashState.isBright = !flashState.isBright;
      flashState.blinksRemaining--;
      softPwmWrite(ch, flashState.isBright ? 255 : 0);
      flashState.nextToggleMs = millis() + Config::FLASH_INTERVAL_MS;
    } else {
      if (ch < Config::CHANNEL_COUNT) {
        softPwmWrite(ch, flashState.savedValue);
        if (flashState.prevWasFading) {
          startEffect(ch, EffectType::FADE_IN_OUT, EffectMode::LOOPED);
        }
      }
      flashState.active = false;
    }
  }

  // -------------------- Command Handlers --------------------

  void handleDigitSelection(int digit) {
    if (digit < 0 || digit >= Config::CHANNEL_COUNT) {
      snprintf(logBuffer, sizeof(logBuffer), "Digit %d out of range", digit);
      Serial.println(logBuffer);
      return;
    }

    if (isChaining) {
      if (sourceChainChannel >= 0 && sourceChainChannel < Config::CHANNEL_COUNT) {
        effects[sourceChainChannel].linkedChannel = digit;
        snprintf(logBuffer, sizeof(logBuffer), "Chained ch%d -> ch%d", sourceChainChannel, digit);
        Serial.println(logBuffer);
      }
      isChaining = false;
      sourceChainChannel = -1;
    } else {
      selectedIndex = digit;
      snprintf(logBuffer, sizeof(logBuffer), "Selected ch%d (Pin %u)", selectedIndex, Config::PWM_PINS[selectedIndex]);
      Serial.println(logBuffer);
      startFlash(selectedIndex);
    }
  }

  void adjustBrightnessByDelta(int32_t deltaSteps) {
    const uint8_t maxV = 255;
    const uint8_t current = brightness[selectedIndex];

    if ((deltaSteps > 0 && current >= maxV) || (deltaSteps < 0 && current == 0)) {
      startFlash(selectedIndex);
      return;
    }

    const uint16_t step = max(255 / Config::BRIGHTNESS_STEPS, 1);
    const int16_t newVal = (int16_t)current + (int16_t)deltaSteps * (int16_t)step;
    brightness[selectedIndex] = constrain(newVal, 0, 255);

    softPwmWrite(selectedIndex, brightness[selectedIndex]);

    dtostrf(100.0f * brightness[selectedIndex] / maxV, 4, 1, logBuffer + 30);
    snprintf(logBuffer, sizeof(logBuffer), "Ch%d brightness: %u/255 (%s%%)", selectedIndex,
             brightness[selectedIndex], logBuffer + 30);
    Serial.println(logBuffer);
  }

  void adjustDutyCycle(int32_t direction) {
    Effect &e = effects[selectedIndex];
    e.dutyCycle += direction * Config::DUTY_CYCLE_STEP;
    e.dutyCycle = constrain(e.dutyCycle, 1.0f, 99.0f);

    if (e.isActive() && (e.type == EffectType::PULSE || e.type == EffectType::HEARTBEAT)) {
      calculateEffectTiming(e);
    }

    dtostrf(e.dutyCycle, 4, 1, logBuffer + 20);
    snprintf(logBuffer, sizeof(logBuffer), "Ch%d duty: %s%%", selectedIndex, logBuffer + 20);
    Serial.println(logBuffer);
  }

  void adjustFlickerIntensity(int32_t direction) {
    Effect &e = effects[selectedIndex];
    e.flickerIntensity = constrain((int)e.flickerIntensity + direction * 10, 0, 100);
    snprintf(logBuffer, sizeof(logBuffer), "Ch%d flicker: %u%%", selectedIndex, e.flickerIntensity);
    Serial.println(logBuffer);
  }

  void applyFrequencyChangeToChannel(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;

    Effect &e = effects[channel];
    calculateEffectTiming(e);

    if (!e.isActive()) {
      dtostrf(e.frequency, 6, 3, logBuffer + 30);
      snprintf(logBuffer, sizeof(logBuffer), "Ch%u: freq=%sHz (stored)", channel, logBuffer+30);
      Serial.println(logBuffer);
      return;
    }

    e.lastStepTime = millis();
    e.nextUpdateMs = millis();
    dtostrf(e.frequency, 6, 3, logBuffer + 30);
    snprintf(logBuffer, sizeof(logBuffer), "Ch%u: freq=%sHz updated", channel, logBuffer+30);
    Serial.println(logBuffer);
  }

  void processSamsungCommand(uint32_t rawCmd) {
    const uint8_t cmd = rawCmd & 0xFF;

    switch (cmd) {
      case RemoteKeys::KEY_0 ... RemoteKeys::KEY_7: {
        int digit = (cmd == RemoteKeys::KEY_0) ? 0 : (cmd - RemoteKeys::KEY_1 + 1);
        handleDigitSelection(digit);
        break;
      }

      case RemoteKeys::VOL_UP:
        adjustBrightnessByDelta(+1);
        break;

      case RemoteKeys::VOL_DOWN:
        adjustBrightnessByDelta(-1);
        break;

      case RemoteKeys::MUTE: {
        brightness[selectedIndex] = (brightness[selectedIndex] == 0) ? 127 : 0;
        softPwmWrite(selectedIndex, brightness[selectedIndex]);
        snprintf(logBuffer, sizeof(logBuffer), "Mute toggle ch%d", selectedIndex);
        Serial.println(logBuffer);
        break;
      }

      case RemoteKeys::UP:
      case RemoteKeys::DOWN:
        if (isChaining && sourceChainChannel >= 0) {
          Effect &e = effects[sourceChainChannel];
          int nextType = (int)e.nextEffectType;
          nextType += (cmd == RemoteKeys::UP) ? 1 : -1;

          if (nextType > (int)EffectType::SMOOTH_RANDOM) nextType = (int)EffectType::PULSE;
          if (nextType < (int)EffectType::PULSE) nextType = (int)EffectType::SMOOTH_RANDOM;

          e.nextEffectType = (EffectType)nextType;
          snprintf(logBuffer, sizeof(logBuffer), "Next effect ch%d: %s", sourceChainChannel,
                       effectTypeToString(e.nextEffectType));
          Serial.println(logBuffer);
        } else {
          Serial.println("Resolution change not supported.");
        }
        break;

      case RemoteKeys::LEFT:
      case RemoteKeys::RIGHT: {
        const int target = (isChaining && sourceChainChannel >= 0) ?
                          sourceChainChannel : selectedIndex;
        if (target >= 0 && target < Config::CHANNEL_COUNT) {
          Effect &e = effects[target];
          e.frequency *= (cmd == RemoteKeys::LEFT) ?
                        (1.0f / Config::FREQ_STEP_RATIO) : Config::FREQ_STEP_RATIO;
          e.frequency = constrain(e.frequency, Config::MIN_EFFECT_FREQ, Config::MAX_EFFECT_FREQ);
          applyFrequencyChangeToChannel(target);
        }
        break;
      }

      case RemoteKeys::CH_UP:
      case RemoteKeys::CH_DOWN:
        if (effects[selectedIndex].type == EffectType::CANDLE_FLICKER) {
          adjustFlickerIntensity((cmd == RemoteKeys::CH_UP) ? +1 : -1);
        } else {
          adjustDutyCycle((cmd == RemoteKeys::CH_UP) ? +1 : -1);
        }
        break;

      case RemoteKeys::REWIND:
      case RemoteKeys::FORWARD:
        cycleEffect(selectedIndex, (cmd == RemoteKeys::FORWARD) ? +1 : -1);
        break;

      case RemoteKeys::PLAY:
        startEffect(selectedIndex, EffectType::FADE_OUT, EffectMode::ONE_SHOT);
        break;

      case RemoteKeys::STOP:
        if (effects[selectedIndex].isActive()) {
          stopEffect(selectedIndex);
        }
        break;

      case RemoteKeys::PAUSE: {
        Effect &e = effects[selectedIndex];
        if (e.isActive()) {
          e.state = EffectState::IDLE;
          snprintf(logBuffer, sizeof(logBuffer), "Ch%d: Paused", selectedIndex);
          Serial.println(logBuffer);
        } else if (e.type != EffectType::NONE) {
          e.lastStepTime = millis();
          e.nextUpdateMs = millis();
          e.state = EffectState::RUNNING; // A generic running state
          snprintf(logBuffer, sizeof(logBuffer), "Ch%d: Resumed", selectedIndex);
          Serial.println(logBuffer);
        }
        break;
      }

      case RemoteKeys::CH_LIST:
        isChaining = true;
        sourceChainChannel = selectedIndex;
        snprintf(logBuffer, sizeof(logBuffer), "Chaining mode: ch%d", selectedIndex);
        Serial.println(logBuffer);
        startFlash(selectedIndex);
        break;

      case RemoteKeys::INFO:
        printChannelStatus(selectedIndex);
        break;

      case RemoteKeys::MENU:
        printAllChannelsStatus();
        break;

      case RemoteKeys::GUIDE:
        printEffectsList();
        break;

      default:
        // Presets and save keys are ignored
        snprintf(logBuffer, sizeof(logBuffer), "Unhandled or unsupported key: 0x%02X", cmd);
        Serial.println(logBuffer);
        break;
    }
  }

  // -------------------- Status Display --------------------

  void printEffectsList() {
    Serial.println("\n========== Available Effects ==========");
    Serial.println("Use REWIND/FORWARD to cycle:");
    Serial.println("  0. None");
    Serial.println("  1. Pulse");
    Serial.println("  2. Fade In/Out");
    Serial.println("  3. Fade In");
    Serial.println("  4. Fade Out");
    Serial.println("  5. Breathing");
    Serial.println("  6. Strobe");
    Serial.println("  7. Heartbeat");
    Serial.println("  8. Random Flash");
    Serial.println("  9. Sawtooth Up");
    Serial.println(" 10. Sawtooth Down");
    Serial.println(" 11. Triangle");
    Serial.println(" 12. Candle Flicker");
    Serial.println(" 13. Smooth Random (Disabled)");
    Serial.println("=======================================\n");
  }

  void printChannelStatus(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;

    const Effect &e = effects[channel];

    Serial.println("\n--- Channel Status ---");
    snprintf(logBuffer, sizeof(logBuffer), "Channel: %d (Pin %u)", channel, Config::PWM_PINS[channel]);
    Serial.println(logBuffer);

    dtostrf(100.0f * brightness[channel] / 255.0f, 4, 1, logBuffer + 30);
    snprintf(logBuffer, sizeof(logBuffer), "Brightness: %u/255 (%s%%)", brightness[channel], logBuffer + 30);
    Serial.println(logBuffer);

    snprintf(logBuffer, sizeof(logBuffer), "Effect: %s [%s]", effectTypeToString(e.type), effectModeToString(e.mode));
    Serial.println(logBuffer);

    if (e.type != EffectType::NONE) {
      dtostrf(e.frequency, 6, 3, logBuffer + 20);
      snprintf(logBuffer, sizeof(logBuffer), "Frequency: %s Hz", logBuffer + 20);
      Serial.println(logBuffer);

      if (e.type == EffectType::PULSE || e.type == EffectType::HEARTBEAT) {
        dtostrf(e.dutyCycle, 4, 1, logBuffer + 40);
        snprintf(logBuffer, sizeof(logBuffer), "Duty: %s%% (on=%ums off=%ums)",
                 logBuffer + 40, (unsigned int)e.onTime, (unsigned int)e.offTime);
        Serial.println(logBuffer);
      } else if (e.type == EffectType::CANDLE_FLICKER) {
        snprintf(logBuffer, sizeof(logBuffer), "Flicker: %u%%", e.flickerIntensity);
        Serial.println(logBuffer);
      }

      snprintf(logBuffer, sizeof(logBuffer), "State: %s", e.isActive() ? "Active" : "Idle");
      Serial.println(logBuffer);

      snprintf(logBuffer, sizeof(logBuffer), "Cycles: %lu", (unsigned long)e.cycleCount);
      Serial.println(logBuffer);

      if (e.linkedChannel < Config::CHANNEL_COUNT) {
        snprintf(logBuffer, sizeof(logBuffer), "Linked: Ch%u -> %s",
                 e.linkedChannel, effectTypeToString(e.nextEffectType));
        Serial.println(logBuffer);
      }
    }
    Serial.println("---------------------\n");
  }

  void printAllChannelsStatus() {
    Serial.println("\n========== All Channels ==========");
    snprintf(logBuffer, sizeof(logBuffer), "Selected: Ch%d", selectedIndex);
    Serial.println(logBuffer);
    Serial.println();

    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      const Effect &e = effects[i];
      const float pct = 100.0f * brightness[i] / 255.0f;

      char pct_str[10];
      dtostrf(pct, 3, 0, pct_str);

      snprintf(logBuffer, sizeof(logBuffer), "Ch%d: %s%% | %s", i, pct_str, effectTypeToString(e.type));
      Serial.print(logBuffer);

      if (e.type != EffectType::NONE) {
        char freq_str[10];
        dtostrf(e.frequency, 4, 1, freq_str);
        snprintf(logBuffer, sizeof(logBuffer), " [%s] %sHz", effectModeToString(e.mode), freq_str);
        Serial.print(logBuffer);
        if (e.isActive()) Serial.print(" *");
      }
      Serial.println();
    }
    Serial.println("==================================\n");
  }

public:
  LEDController() {
    // Constructor
  }

  void begin() {
    Serial.begin(115200);
    while (!Serial) {;}
    delay(500);

    printWelcomeMessage();

    // Initialize software PWM
    slow_pwm_init(100); // 100 Hz frequency

    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      softPwmWrite(i, brightness[i]);
    }

    delay(200);
    IrReceiver.begin(Config::IR_PIN, DISABLE_LED_FEEDBACK);
    snprintf(logBuffer, sizeof(logBuffer), "IR: GPIO %u", Config::IR_PIN);
    Serial.println(logBuffer);

    startFlash(selectedIndex);
    Serial.println("Ready\n");
  }

  void update() {
    updateFlash();
    updateEffects();

    // The software PWM library might require an update call in the main loop
    slow_pwm_update();

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.protocol == SAMSUNG &&
          IrReceiver.decodedIRData.address == 0x7) {
        const uint8_t cmd = IrReceiver.decodedIRData.command & 0xFF;
        snprintf(logBuffer, sizeof(logBuffer), "CMD: 0x%02X", cmd);
        Serial.println(logBuffer);
        processSamsungCommand(IrReceiver.decodedIRData.command);
      }
      IrReceiver.resume();
    }
  }

private:
  static void printWelcomeMessage() {
    Serial.println("\n======================================================");
    Serial.println("     Arduino Uno LED Controller - Ported");
    Serial.println("======================================================\n");
    Serial.println("Basic Controls:");
    Serial.println("  0-5            Select channel");
    Serial.println("  VOL+/VOL-      Brightness");
    Serial.println("  CH+/CH-        Duty cycle / Flicker intensity");
    Serial.println("  UP/DOWN        (No function)");
    Serial.println("  LEFT/RIGHT     Effect frequency");
    Serial.println("  MUTE           Toggle 0%/50%");
    Serial.println();
    Serial.println("Effects:");
    Serial.println("  REWIND/FWD     Cycle through all effects");
    Serial.println("  PLAY           Fade Out (one-shot)");
    Serial.println("  PAUSE          Pause/Resume effect");
    Serial.println("  STOP           Stop effect");
    Serial.println();
    Serial.println("Advanced:");
    Serial.println("  CH LIST        Chain effects");
    Serial.println("  INFO           Channel status");
    Serial.println("  MENU           All channels");
    Serial.println("  GUIDE          List all effects");
    Serial.println("======================================================\n");
  }
};

// -------------------- Global Instance --------------------

LEDController controller;

// -------------------- Arduino Entry Points --------------------

void setup() {
  controller.begin();
}

void loop() {
  controller.update();
}
