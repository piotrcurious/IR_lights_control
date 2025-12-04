// ESP32 LEDC PWM + IR Remote Controller (Optimized with Enhanced Effects)
// IR receiver on GPIO15; Samsung protocol address 0x7 only.

#include <Arduino.h>
#include <IRremote.h>
#include <Preferences.h>

#if defined(ARDUINO_ARCH_ESP32)
extern "C" {
#include "driver/ledc.h"
}
#endif

// -------------------- Configuration --------------------

namespace Config {
  constexpr uint8_t IR_PIN = 15;
  constexpr uint8_t PWM_PINS[8] = {2, 4, 16, 17, 18, 19, 21, 22};
  constexpr uint8_t CHANNEL_COUNT = 8;
  constexpr uint32_t LEDC_FREQ = 5000;
  constexpr uint8_t MIN_RESOLUTION = 1;
  constexpr uint8_t MAX_RESOLUTION = 13;
  constexpr uint8_t DEFAULT_RESOLUTION = 8;
  
  constexpr float MIN_EFFECT_FREQ = 0.1f;
  constexpr float MAX_EFFECT_FREQ = 50.0f;
  constexpr float FREQ_STEP_RATIO = 1.1f;
  constexpr float DEFAULT_DUTY_CYCLE = 50.0f;
  constexpr float DUTY_CYCLE_STEP = 5.0f;
  
  constexpr uint32_t FADE_STEP_MS = 20;  // ~50Hz update rate
  constexpr uint32_t FLASH_INTERVAL_MS = 80;
  constexpr uint8_t FLASH_CYCLES = 3;
  constexpr uint8_t SAVE_INDICATOR_BLINKS = 4;
  constexpr uint8_t BRIGHTNESS_STEPS = 16;
  
  constexpr ledc_mode_t LEDC_MODE = LEDC_HIGH_SPEED_MODE;
  constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
}

// -------------------- Types & Enums --------------------

enum class EffectState : uint8_t {
  IDLE,
  PULSE_ON,
  PULSE_OFF,
  FADE_UP,
  FADE_DOWN
};

enum class EffectType : uint8_t {
  NONE,
  PULSE,
  FADE_IN_OUT,
  FADE_IN,      // One-shot fade in
  FADE_OUT      // One-shot fade out
};

enum class EffectMode : uint8_t {
  LOOPED,
  ONE_SHOT
};

struct Effect {
  EffectType type = EffectType::NONE;
  EffectState state = EffectState::IDLE;
  EffectMode mode = EffectMode::LOOPED;
  
  float frequency = 1.0f;
  float dutyCycle = Config::DEFAULT_DUTY_CYCLE;
  uint32_t duration = 500;
  uint32_t savedBrightness = 0;
  uint32_t onTime = 0;
  uint32_t offTime = 0;
  uint32_t lastStepTime = 0;
  uint32_t nextUpdateMs = 0;
  
  uint8_t linkedChannel = 0xFF;
  EffectType nextEffectType = EffectType::PULSE;
  uint32_t cycleCount = 0;  // Track number of cycles completed
  uint32_t maxCycles = 0;   // 0 = infinite, >0 = stop after N cycles
  
  // Validation helpers
  void validate() {
    if (frequency <= 0.0f) frequency = 1.0f;
    if (dutyCycle <= 0.0f || dutyCycle > 100.0f) dutyCycle = Config::DEFAULT_DUTY_CYCLE;
    if (duration == 0) duration = 500;
    frequency = constrain(frequency, Config::MIN_EFFECT_FREQ, Config::MAX_EFFECT_FREQ);
    dutyCycle = constrain(dutyCycle, 1.0f, 99.0f);
  }
  
  void reset() {
    state = EffectState::IDLE;
    lastStepTime = 0;
    nextUpdateMs = 0;
    cycleCount = 0;
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
  uint32_t savedValue = 0;
  uint32_t nextToggleMs = 0;
  uint8_t channel = 0xFF;
  uint8_t blinksRemaining = 0;
};

struct SaveIndicatorState {
  bool active = false;
  bool isBright = false;
  uint32_t nextToggleMs = 0;
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
  uint32_t brightness[Config::CHANNEL_COUNT] = {0};
  uint32_t lastDuty[Config::CHANNEL_COUNT];
  bool wasFading[Config::CHANNEL_COUNT] = {false};
  
  uint8_t currentResolution = Config::DEFAULT_RESOLUTION;
  int selectedIndex = 0;
  bool isChaining = false;
  int sourceChainChannel = -1;
  
  FlashState flashState;
  SaveIndicatorState saveIndicatorState;

  // -------------------- Utility Methods --------------------
  
  static constexpr uint32_t maxForRes(uint8_t res) {
    return (res >= 32) ? UINT32_MAX : ((1UL << res) - 1);
  }
  
  static uint32_t scaleValue(uint32_t value, uint8_t oldRes, uint8_t newRes) {
    const uint32_t oldMax = maxForRes(oldRes);
    const uint32_t newMax = maxForRes(newRes);
    if (oldMax == 0) return 0;
    
    const uint64_t scaled = ((uint64_t)value * newMax + oldMax / 2) / oldMax;
    return (scaled > newMax) ? newMax : (uint32_t)scaled;
  }
  
  void ledcWriteDuty(uint8_t channel, uint32_t duty) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    const uint32_t maxV = maxForRes(currentResolution);
    duty = min(duty, maxV);
    
    if (lastDuty[channel] == duty) return;
    
    lastDuty[channel] = duty;
    ledc_set_duty(Config::LEDC_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(Config::LEDC_MODE, (ledc_channel_t)channel);
  }
  
  static const char* effectTypeToString(EffectType type) {
    switch (type) {
      case EffectType::NONE: return "None";
      case EffectType::PULSE: return "Pulse";
      case EffectType::FADE_IN_OUT: return "Fade In/Out";
      case EffectType::FADE_IN: return "Fade In";
      case EffectType::FADE_OUT: return "Fade Out";
      default: return "Unknown";
    }
  }
  
  static const char* effectModeToString(EffectMode mode) {
    return (mode == EffectMode::LOOPED) ? "Looped" : "One-Shot";
  }

  // -------------------- LEDC Configuration --------------------
  
  void setupLEDCDriver() {
    ledc_timer_config_t timer_config = {
      .speed_mode = Config::LEDC_MODE,
      .duty_resolution = (ledc_timer_bit_t)currentResolution,
      .timer_num = Config::LEDC_TIMER,
      .freq_hz = Config::LEDC_FREQ,
      .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);
    
    for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
      ledc_channel_config_t channel_config = {
        .gpio_num = Config::PWM_PINS[ch],
        .speed_mode = Config::LEDC_MODE,
        .channel = (ledc_channel_t)ch,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = Config::LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
      };
      ledc_channel_config(&channel_config);
      yield();
    }
  }
  
  void reconfigureLEDCTimer(uint8_t newRes) {
    newRes = constrain(newRes, Config::MIN_RESOLUTION, Config::MAX_RESOLUTION);
    if (newRes == currentResolution) return;
    
    const uint8_t oldRes = currentResolution;
    Serial.printf("Reconfiguring resolution %u -> %u\n", oldRes, newRes);
    
    // Scale all values
    for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
      brightness[ch] = scaleValue(brightness[ch], oldRes, newRes);
      if (lastDuty[ch] != UINT32_MAX) {
        lastDuty[ch] = scaleValue(lastDuty[ch], oldRes, newRes);
      }
      yield();
    }
    
    if (flashState.active && flashState.channel < Config::CHANNEL_COUNT) {
      flashState.savedValue = scaleValue(flashState.savedValue, oldRes, newRes);
    }
    
    // Reconfigure timer
    ledc_timer_config_t timer_config = {
      .speed_mode = Config::LEDC_MODE,
      .duty_resolution = (ledc_timer_bit_t)newRes,
      .timer_num = Config::LEDC_TIMER,
      .freq_hz = Config::LEDC_FREQ,
      .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);
    
    // Update all channels
    for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
      ledc_channel_config_t channel_config = {
        .gpio_num = Config::PWM_PINS[ch],
        .speed_mode = Config::LEDC_MODE,
        .channel = (ledc_channel_t)ch,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = Config::LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
      };
      ledc_channel_config(&channel_config);
      
      if (flashState.active && flashState.channel == ch) {
        ledcWriteDuty(ch, flashState.isBright ? maxForRes(newRes) : 0);
      } else {
        ledcWriteDuty(ch, brightness[ch]);
      }
      yield();
    }
    
    currentResolution = newRes;
  }

  // -------------------- Effect Management --------------------
  
  void calculateEffectTiming(Effect &e) {
    e.validate();
    
    const float period = 1000.0f / e.frequency;
    
    if (e.type == EffectType::PULSE) {
      const float duty = e.dutyCycle / 100.0f;
      e.onTime = (uint32_t)(period * duty + 0.5f);
      e.offTime = (uint32_t)(period * (1.0f - duty) + 0.5f);
      if (e.onTime == 0) e.onTime = 1;
      if (e.offTime == 0) e.offTime = 1;
    } else if (e.type == EffectType::FADE_IN_OUT || 
               e.type == EffectType::FADE_IN || 
               e.type == EffectType::FADE_OUT) {
      e.duration = (uint32_t)(period / 2.0f + 0.5f);
      if (e.duration == 0) e.duration = 1;
    }
  }
  
  void applyFrequencyChangeToChannel(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    calculateEffectTiming(e);
    
    if (!e.isActive()) {
      Serial.printf("Ch%u: freq=%.3fHz (stored)\n", channel, e.frequency);
      return;
    }
    
    // Reset timing for active effects
    e.lastStepTime = millis();
    e.nextUpdateMs = millis();
    
    if (e.type == EffectType::PULSE) {
      ledcWriteDuty(channel, (e.state == EffectState::PULSE_ON) ? e.savedBrightness : 0);
      Serial.printf("Ch%u PULSE: %.3fHz, on=%ums, off=%ums\n", 
                    channel, e.frequency, e.onTime, e.offTime);
    } else {
      Serial.printf("Ch%u FADE: %.3fHz, duration=%ums/direction\n",
                    channel, e.frequency, e.duration);
    }
  }
  
  void startEffect(uint8_t channel, EffectType type, EffectMode mode) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    
    // Stop current effect if any
    if (e.isActive()) {
      stopEffect(channel, false);
    }
    
    e.type = type;
    e.mode = mode;
    e.savedBrightness = brightness[channel];
    e.cycleCount = 0;
    
    calculateEffectTiming(e);
    
    e.lastStepTime = millis();
    e.nextUpdateMs = millis();
    
    // Set initial state based on effect type
    switch (type) {
      case EffectType::PULSE:
        e.state = EffectState::PULSE_ON;
        ledcWriteDuty(channel, e.savedBrightness);
        Serial.printf("Ch%u: Started PULSE [%s] %.3fHz (on=%ums off=%ums)\n",
                     channel, effectModeToString(mode), e.frequency, e.onTime, e.offTime);
        break;
        
      case EffectType::FADE_IN_OUT:
        e.state = EffectState::FADE_UP;
        ledcWriteDuty(channel, 0);
        Serial.printf("Ch%u: Started FADE_IN_OUT [%s] %.3fHz (%ums)\n",
                     channel, effectModeToString(mode), e.frequency, e.duration);
        break;
        
      case EffectType::FADE_IN:
        e.state = EffectState::FADE_UP;
        ledcWriteDuty(channel, 0);
        Serial.printf("Ch%u: Started FADE_IN [%s] %.3fHz (%ums)\n",
                     channel, effectModeToString(mode), e.frequency, e.duration);
        break;
        
      case EffectType::FADE_OUT:
        e.state = EffectState::FADE_DOWN;
        ledcWriteDuty(channel, e.savedBrightness);
        Serial.printf("Ch%u: Started FADE_OUT [%s] %.3fHz (%ums)\n",
                     channel, effectModeToString(mode), e.frequency, e.duration);
        break;
        
      default:
        e.state = EffectState::IDLE;
        break;
    }
  }
  
  void stopEffect(uint8_t channel, bool restoreBrightness = true) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    
    if (restoreBrightness) {
      brightness[channel] = e.savedBrightness;
      ledcWriteDuty(channel, brightness[channel]);
      Serial.printf("Ch%u: Stopped effect, brightness=%lu\n", 
                    channel, (unsigned long)brightness[channel]);
    }
    
    e.state = EffectState::IDLE;
    e.type = EffectType::NONE;
    e.cycleCount = 0;
  }
  
  void toggleEffectMode(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    e.mode = (e.mode == EffectMode::LOOPED) ? EffectMode::ONE_SHOT : EffectMode::LOOPED;
    
    Serial.printf("Ch%u: Effect mode -> %s\n", channel, effectModeToString(e.mode));
    
    // If effect is currently running, restart with new mode
    if (e.isActive()) {
      EffectType currentType = e.type;
      startEffect(channel, currentType, e.mode);
    }
  }
  
  void updatePulseEffect(uint8_t channel, Effect &e, uint32_t now) {
    if (e.state == EffectState::PULSE_ON) {
      if (now - e.lastStepTime >= e.onTime) {
        e.lastStepTime = now;
        e.state = EffectState::PULSE_OFF;
        ledcWriteDuty(channel, 0);
      }
    } else if (e.state == EffectState::PULSE_OFF) {
      if (now - e.lastStepTime >= e.offTime) {
        e.lastStepTime = now;
        e.cycleCount++;
        
        // Check if we should continue or stop
        if (e.mode == EffectMode::ONE_SHOT || 
            (e.maxCycles > 0 && e.cycleCount >= e.maxCycles)) {
          handleEffectCompletion(channel, e);
          return;
        }
        
        e.state = EffectState::PULSE_ON;
        ledcWriteDuty(channel, e.savedBrightness);
      }
    }
  }
  
  void updateFadeEffect(uint8_t channel, Effect &e, uint32_t now) {
    // Throttle updates
    if ((int32_t)(now - e.nextUpdateMs) < 0) return;
    e.nextUpdateMs = now + Config::FADE_STEP_MS;
    
    const uint32_t elapsed = now - e.lastStepTime;
    const uint32_t dur = e.duration;
    const float progress = min((float)elapsed / dur, 1.0f);
    
    if (progress >= 1.0f) {
      // Completed one direction
      bool effectComplete = false;
      
      if (e.type == EffectType::FADE_IN) {
        // Fade in completes when we reach full brightness
        ledcWriteDuty(channel, e.savedBrightness);
        effectComplete = true;
      } else if (e.type == EffectType::FADE_OUT) {
        // Fade out completes when we reach zero
        ledcWriteDuty(channel, 0);
        effectComplete = true;
      } else if (e.type == EffectType::FADE_IN_OUT) {
        // Toggle direction for fade in/out
        e.lastStepTime = now;
        
        if (e.state == EffectState::FADE_UP) {
          ledcWriteDuty(channel, e.savedBrightness);
          e.state = EffectState::FADE_DOWN;
        } else {
          ledcWriteDuty(channel, 0);
          e.state = EffectState::FADE_UP;
          e.cycleCount++;
          
          // Check if cycle limit reached
          if (e.mode == EffectMode::ONE_SHOT || 
              (e.maxCycles > 0 && e.cycleCount >= e.maxCycles)) {
            effectComplete = true;
          }
        }
      }
      
      if (effectComplete) {
        handleEffectCompletion(channel, e);
      }
    } else {
      // Calculate intermediate value
      const uint32_t target = e.savedBrightness;
      uint32_t duty;
      
      if (e.state == EffectState::FADE_UP) {
        duty = (uint32_t)(target * progress + 0.5f);
      } else {
        duty = (uint32_t)(target * (1.0f - progress) + 0.5f);
      }
      
      ledcWriteDuty(channel, duty);
    }
  }
  
  void handleEffectCompletion(uint8_t channel, Effect &e) {
    const uint8_t linked = e.linkedChannel;
    const EffectType next = e.nextEffectType;
    
    Serial.printf("Ch%u: Effect completed (cycles=%lu)\n", channel, (unsigned long)e.cycleCount);
    
    stopEffect(channel, true);
    
    // Trigger linked channel if configured
    if (linked < Config::CHANNEL_COUNT) {
      Serial.printf("Ch%u: Triggering linked ch%u with %s\n", 
                   channel, linked, effectTypeToString(next));
      startEffect(linked, next, EffectMode::ONE_SHOT);
    }
  }
  
  void updateEffects() {
    const uint32_t now = millis();
    
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      Effect &e = effects[i];
      
      if (!e.isActive()) continue;
      
      if (e.type == EffectType::PULSE) {
        updatePulseEffect(i, e, now);
      } else if (e.type == EffectType::FADE_IN_OUT || 
                 e.type == EffectType::FADE_IN || 
                 e.type == EffectType::FADE_OUT) {
        updateFadeEffect(i, e, now);
      }
      
      yield();
    }
  }

  // -------------------- Visual Feedback --------------------
  
  void startFlash(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    // Restore previous flash if active
    if (flashState.active && flashState.channel < Config::CHANNEL_COUNT) {
      ledcWriteDuty(flashState.channel, flashState.savedValue);
      if (flashState.prevWasFading) {
        startEffect(flashState.channel, EffectType::FADE_IN_OUT, EffectMode::LOOPED);
      }
    }
    
    // Setup new flash
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
    ledcWriteDuty(channel, maxForRes(currentResolution));
  }
  
  void updateFlash() {
    if (!flashState.active) return;
    if ((int32_t)(millis() - flashState.nextToggleMs) < 0) return;
    
    const uint8_t ch = flashState.channel;
    
    if (flashState.blinksRemaining > 0) {
      flashState.isBright = !flashState.isBright;
      flashState.blinksRemaining--;
      ledcWriteDuty(ch, flashState.isBright ? maxForRes(currentResolution) : 0);
      flashState.nextToggleMs = millis() + Config::FLASH_INTERVAL_MS;
    } else {
      if (ch < Config::CHANNEL_COUNT) {
        ledcWriteDuty(ch, flashState.savedValue);
        if (flashState.prevWasFading) {
          startEffect(ch, EffectType::FADE_IN_OUT, EffectMode::LOOPED);
        }
      }
      flashState.active = false;
    }
  }
  
  void indicateSave() {
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      wasFading[i] = effects[i].isActive();
      if (wasFading[i]) {
        effects[i].state = EffectState::IDLE;
      }
      yield();
    }
    
    saveIndicatorState.active = true;
    saveIndicatorState.blinksRemaining = Config::SAVE_INDICATOR_BLINKS;
    saveIndicatorState.isBright = false;
    saveIndicatorState.nextToggleMs = millis();
  }
  
  void updateSaveIndicator() {
    if (!saveIndicatorState.active) return;
    if ((int32_t)(millis() - saveIndicatorState.nextToggleMs) < 0) return;
    
    if (saveIndicatorState.blinksRemaining > 0) {
      saveIndicatorState.isBright = !saveIndicatorState.isBright;
      saveIndicatorState.blinksRemaining--;
      const uint32_t duty = saveIndicatorState.isBright ? maxForRes(currentResolution) : 0;
      for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
        ledcWriteDuty(ch, duty);
      }
      saveIndicatorState.nextToggleMs = millis() + Config::FLASH_INTERVAL_MS;
    } else {
      for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
        ledcWriteDuty(i, brightness[i]);
        if (wasFading[i]) {
          startEffect(i, EffectType::FADE_IN_OUT, EffectMode::LOOPED);
        }
        yield();
      }
      saveIndicatorState.active = false;
    }
  }

  // -------------------- Command Handlers --------------------
  
  void handleDigitSelection(int digit) {
    if (digit < 0 || digit >= Config::CHANNEL_COUNT) {
      Serial.printf("Digit %d out of range (0..%d)\n", digit, Config::CHANNEL_COUNT - 1);
      return;
    }
    
    if (isChaining) {
      if (sourceChainChannel >= 0 && sourceChainChannel < Config::CHANNEL_COUNT) {
        effects[sourceChainChannel].linkedChannel = digit;
        Serial.printf("Chained ch%d -> ch%d\n", sourceChainChannel, digit);
      }
      isChaining = false;
      sourceChainChannel = -1;
    } else {
      selectedIndex = digit;
      Serial.printf("Selected ch%d (GPIO %u)\n", selectedIndex, Config::PWM_PINS[selectedIndex]);
      startFlash(selectedIndex);
    }
  }
  
  void adjustBrightnessByDelta(int32_t deltaSteps) {
    const uint32_t maxV = maxForRes(currentResolution);
    const uint32_t current = brightness[selectedIndex];
    
    if ((deltaSteps > 0 && current >= maxV) || (deltaSteps < 0 && current == 0)) {
      startFlash(selectedIndex);
      return;
    }
    
    const uint32_t step = max(maxV / Config::BRIGHTNESS_STEPS, 1U);
    const int64_t newVal = (int64_t)current + (int64_t)deltaSteps * (int64_t)step;
    brightness[selectedIndex] = constrain(newVal, 0LL, (int64_t)maxV);
    
    ledcWriteDuty(selectedIndex, brightness[selectedIndex]);
    Serial.printf("Ch%d brightness: %lu/%lu (%.1f%%)\n", selectedIndex, 
                  (unsigned long)brightness[selectedIndex], (unsigned long)maxV,
                  100.0f * brightness[selectedIndex] / maxV);
  }
  
  void adjustDutyCycle(int32_t direction) {
    Effect &e = effects[selectedIndex];
    e.dutyCycle += direction * Config::DUTY_CYCLE_STEP;
    e.dutyCycle = constrain(e.dutyCycle, 1.0f, 99.0f);
    
    if (e.isActive() && e.type == EffectType::PULSE) {
      applyFrequencyChangeToChannel(selectedIndex);
    }
    
    Serial.printf("Ch%d duty cycle: %.1f%%\n", selectedIndex, e.dutyCycle);
  }
  
  void processSamsungCommand(uint32_t rawCmd) {
    const uint8_t cmd = rawCmd & 0xFF;
    
    switch (cmd) {
      case RemoteKeys::KEY_0 ... RemoteKeys::KEY_7: {
        int digit = (cmd == RemoteKeys::KEY_0) ? 0 : (cmd - RemoteKeys::KEY_1 + 1);
        handleDigitSelection(digit);
        break;
      }
