// ESP32 LEDC PWM + IR Remote Controller (Enhanced with Multiple Effects & Presets)
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
  
  constexpr uint32_t FADE_STEP_MS = 20;
  constexpr uint32_t FLASH_INTERVAL_MS = 80;
  constexpr uint8_t FLASH_CYCLES = 3;
  constexpr uint8_t SAVE_INDICATOR_BLINKS = 4;
  constexpr uint8_t BRIGHTNESS_STEPS = 16;
  constexpr uint8_t PRESET_SLOTS = 4;  // RED, GREEN, YELLOW, BLUE
  
  constexpr ledc_mode_t LEDC_MODE = LEDC_HIGH_SPEED_MODE;
  constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
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
  uint32_t savedBrightness = 0;
  uint32_t onTime = 0;
  uint32_t offTime = 0;
  uint32_t lastStepTime = 0;
  uint32_t nextUpdateMs = 0;
  
  uint8_t linkedChannel = 0xFF;
  EffectType nextEffectType = EffectType::PULSE;
  uint32_t cycleCount = 0;
  uint32_t maxCycles = 0;
  
  uint32_t randomTarget = 0;
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

struct Preset {
  uint32_t brightness[Config::CHANNEL_COUNT];
  Effect effects[Config::CHANNEL_COUNT];
  uint8_t resolution;
  char name[16];
  bool valid;
  
  Preset() : resolution(Config::DEFAULT_RESOLUTION), valid(false) {
    memset(brightness, 0, sizeof(brightness));
    memset(name, 0, sizeof(name));
    strcpy(name, "Empty");
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
  
  Preset presets[Config::PRESET_SLOTS];
  bool savePresetMode = false;
  
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
    Serial.printf("Resolution %u -> %u\n", oldRes, newRes);
    
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
    
    ledc_timer_config_t timer_config = {
      .speed_mode = Config::LEDC_MODE,
      .duty_resolution = (ledc_timer_bit_t)newRes,
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
        ledcWriteDuty(channel, e.savedBrightness);
        break;
      case EffectType::FADE_IN_OUT:
      case EffectType::FADE_IN:
      case EffectType::TRIANGLE:
        e.state = EffectState::FADE_UP;
        ledcWriteDuty(channel, 0);
        break;
      case EffectType::FADE_OUT:
      case EffectType::SAWTOOTH_DOWN:
        e.state = EffectState::FADE_DOWN;
        ledcWriteDuty(channel, e.savedBrightness);
        break;
      case EffectType::BREATHING:
        e.state = EffectState::BREATHING;
        ledcWriteDuty(channel, 0);
        break;
      case EffectType::STROBE:
        e.state = EffectState::STROBE_ON;
        ledcWriteDuty(channel, e.savedBrightness);
        break;
      case EffectType::HEARTBEAT:
        e.state = EffectState::PULSE_ON;
        e.phaseOffset = 0;
        ledcWriteDuty(channel, e.savedBrightness);
        break;
      case EffectType::RANDOM_FLASH:
      case EffectType::CANDLE_FLICKER:
      case EffectType::SMOOTH_RANDOM:
        e.state = EffectState::RUNNING;
        e.randomTarget = random(0, e.savedBrightness + 1);
        ledcWriteDuty(channel, e.randomTarget);
        break;
      case EffectType::SAWTOOTH_UP:
        e.state = EffectState::FADE_UP;
        ledcWriteDuty(channel, 0);
        break;
      default:
        e.state = EffectState::IDLE;
        break;
    }
    
    Serial.printf("Ch%u: %s [%s] %.2fHz\n", channel, 
                 effectTypeToString(type), effectModeToString(mode), e.frequency);
  }
  
  void stopEffect(uint8_t channel, bool restoreBrightness = true) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    
    if (restoreBrightness) {
      brightness[channel] = e.savedBrightness;
      ledcWriteDuty(channel, brightness[channel]);
    }
    
    e.state = EffectState::IDLE;
    e.type = EffectType::NONE;
    e.cycleCount = 0;
  }
  
  void toggleEffectMode(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    e.mode = (e.mode == EffectMode::LOOPED) ? EffectMode::ONE_SHOT : EffectMode::LOOPED;
    
    Serial.printf("Ch%u: Mode -> %s\n", channel, effectModeToString(e.mode));
    
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
    
    Serial.printf("Ch%u: Completed (cycles=%lu)\n", channel, (unsigned long)e.cycleCount);
    
    stopEffect(channel, true);
    
    if (linked < Config::CHANNEL_COUNT) {
      Serial.printf("Ch%u -> Ch%u: %s\n", channel, linked, effectTypeToString(next));
      startEffect(linked, next, EffectMode::ONE_SHOT);
    }
  }
  
  // -------------------- Effect Update Functions --------------------
  
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
  
  void updateStrobeEffect(uint8_t channel, Effect &e, uint32_t now) {
    if (e.state == EffectState::STROBE_ON) {
      if (now - e.lastStepTime >= e.onTime) {
        e.lastStepTime = now;
        e.state = EffectState::STROBE_OFF;
        ledcWriteDuty(channel, 0);
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
        ledcWriteDuty(channel, e.savedBrightness);
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
        ledcWriteDuty(channel, 0);
      } else if (e.state == EffectState::PULSE_OFF && now - e.lastStepTime >= beatDuration / 2) {
        e.lastStepTime = now;
        e.state = EffectState::PULSE_ON;
        e.phaseOffset = 2;
        ledcWriteDuty(channel, e.savedBrightness);
      }
    } else if (e.phaseOffset == 2) {
      if (now - e.lastStepTime >= beatDuration) {
        e.lastStepTime = now;
        e.phaseOffset = 3;
        ledcWriteDuty(channel, 0);
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
        
        ledcWriteDuty(channel, e.savedBrightness);
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
        ledcWriteDuty(channel, e.savedBrightness);
        effectComplete = true;
      } else if (e.type == EffectType::FADE_OUT) {
        ledcWriteDuty(channel, 0);
        effectComplete = true;
      } else if (e.type == EffectType::FADE_IN_OUT || e.type == EffectType::TRIANGLE) {
        e.lastStepTime = now;
        if (e.state == EffectState::FADE_UP) {
          ledcWriteDuty(channel, e.savedBrightness);
          e.state = EffectState::FADE_DOWN;
        } else {
          ledcWriteDuty(channel, 0);
          e.state = EffectState::FADE_UP;
          e.cycleCount++;
          if (e.mode == EffectMode::ONE_SHOT) effectComplete = true;
        }
      } else if (e.type == EffectType::SAWTOOTH_UP) {
        ledcWriteDuty(channel, 0);
        e.lastStepTime = now;
        e.cycleCount++;
        if (e.mode == EffectMode::ONE_SHOT) effectComplete = true;
      } else if (e.type == EffectType::SAWTOOTH_DOWN) {
        ledcWriteDuty(channel, e.savedBrightness);
        e.lastStepTime = now;
        e.cycleCount++;
        if (e.mode == EffectMode::ONE_SHOT) effectComplete = true;
      }
      
      if (effectComplete) {
        handleEffectCompletion(channel, e);
      }
    } else {
      const uint32_t target = e.savedBrightness;
      uint32_t duty = 0;
      
      if (e.state == EffectState::FADE_UP) {
        duty = (uint32_t)(target * progress + 0.5f);
      } else {
        duty = (uint32_t)(target * (1.0f - progress) + 0.5f);
      }
      
      ledcWriteDuty(channel, duty);
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
    const uint32_t duty = (uint32_t)(e.savedBrightness * easedProgress + 0.5f);
    ledcWriteDuty(channel, duty);
  }
  
  void updateRandomFlashEffect(uint8_t channel, Effect &e, uint32_t now) {
    if ((int32_t)(now - e.nextUpdateMs) < 0) return;
    
    const uint32_t nextInterval = random(50, e.duration);
    e.nextUpdateMs = now + nextInterval;
    
    e.randomTarget = random(0, e.savedBrightness + 1);
    ledcWriteDuty(channel, e.randomTarget);
    
    e.cycleCount++;
    if (e.mode == EffectMode::ONE_SHOT && e.cycleCount >= 10) {
      handleEffectCompletion(channel, e);
    }
  }
  
  void updateCandleFlickerEffect(uint8_t channel, Effect &e, uint32_t now) {
    if ((int32_t)(now - e.nextUpdateMs) < 0) return;
    
    e.nextUpdateMs = now + random(30, 100);
    
    const float flickerAmount = e.flickerIntensity / 100.0f;
    const uint32_t baseLevel = (uint32_t)(e.savedBrightness * (1.0f - flickerAmount * 0.5f));
    const uint32_t flickerRange = (uint32_t)(e.savedBrightness * flickerAmount);
    
    if (random(100) < 5) {
      e.randomTarget = baseLevel - random(flickerRange);
    } else {
      e.randomTarget = baseLevel + random(flickerRange / 2);
    }
    
    e.randomTarget = constrain(e.randomTarget, 0, e.savedBrightness);
    ledcWriteDuty(channel, e.randomTarget);
  }
  
  void updateSmoothRandomEffect(uint8_t channel, Effect &e, uint32_t now) {
    if ((int32_t)(now - e.nextUpdateMs) < 0) return;
    e.nextUpdateMs = now + Config::FADE_STEP_MS;
    
    if (e.randomDuration == 0 || now - e.lastStepTime >= e.randomDuration) {
      e.lastStepTime = now;
      e.randomTarget = random(0, e.savedBrightness + 1);
      e.randomDuration = random(500, e.duration);
    }
    
    const uint32_t elapsed = now - e.lastStepTime;
    const float progress = min((float)elapsed / e.randomDuration, 1.0f);
    const float easedProgress = easeInOutQuad(progress);
    
    const uint32_t currentDuty = lastDuty[channel];
    const int32_t delta = e.randomTarget - currentDuty;
    const uint32_t newDuty = currentDuty + (int32_t)(delta * easedProgress);
    
    ledcWriteDuty(channel, newDuty);
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
          updateSmoothRandomEffect(i, e, now);
          break;
        default:
          break;
      }
      
      yield();
    }
  }

  // -------------------- Preset Management --------------------
  
  void savePreset(uint8_t slot) {
    if (slot >= Config::PRESET_SLOTS) return;
    
    Preset &preset = presets[slot];
    memcpy(preset.brightness, brightness, sizeof(brightness));
    memcpy(preset.effects, effects, sizeof(effects));
    preset.resolution = currentResolution;
    preset.valid = true;
    
    const char* slotNames[] = {"RED", "GREEN", "YELLOW", "BLUE"};
    snprintf(preset.name, sizeof(preset.name), "%s Preset", slotNames[slot]);
    
    Preferences prefs;
    char keyName[32];
    snprintf(keyName, sizeof(keyName), "preset_%u", slot);
    prefs.begin("led_presets", false);
    prefs.putBytes(keyName, &preset, sizeof(Preset));
    prefs.end();
    
    indicateSave();
    Serial.printf("Saved preset %u: %s\n", slot, preset.name);
  }
  
  void loadPreset(uint8_t slot) {
    if (slot >= Config::PRESET_SLOTS) return;
    
    if (!presets[slot].valid) {
      Serial.printf("Preset %u is empty\n", slot);
      startFlash(selectedIndex);
      return;
    }
    
    // Stop all current effects
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      if (effects[i].isActive()) {
        stopEffect(i, false);
      }
    }
    
    Preset &preset = presets[slot];
    
    // Scale brightness if resolution changed
    if (preset.resolution != currentResolution) {
      for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
        brightness[i] = scaleValue(preset.brightness[i], preset.resolution, currentResolution);
      }
    } else {
      memcpy(brightness, preset.brightness, sizeof(brightness));
    }
    
    memcpy(effects, preset.effects, sizeof(effects));
    
    // Validate and reset effect states
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      effects[i].validate();
      effects[i].reset();
    }
    
    // Apply brightness
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      ledcWriteDuty(i, brightness[i]);
      yield();
    }
    
    // Start effects
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      if (effects[i].type != EffectType::NONE) {
        startEffect(i, effects[i].type, effects[i].mode);
      }
    }
    
    Serial.printf("Loaded preset %u: %s\n", slot, preset.name);
    
    // Flash all channels briefly
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      ledcWriteDuty(i, maxForRes(currentResolution));
    }
    delay(100);
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      ledcWriteDuty(i, brightness[i]);
    }
  }
  
  void loadPresetsFromFlash() {
    Preferences prefs;
    prefs.begin("led_presets", true);
    
    for (uint8_t slot = 0; slot < Config::PRESET_SLOTS; ++slot) {
      char keyName[32];
      snprintf(keyName, sizeof(keyName), "preset_%u", slot);
      
      size_t size = prefs.getBytes(keyName, &presets[slot], sizeof(Preset));
      if (size == sizeof(Preset) && presets[slot].valid) {
        Serial.printf("Loaded preset %u: %s\n", slot, presets[slot].name);
      }
    }
    
    prefs.end();
  }
  
  void clearPreset(uint8_t slot) {
    if (slot >= Config::PRESET_SLOTS) return;
    
    presets[slot] = Preset();
    
    Preferences prefs;
    char keyName[32];
    snprintf(keyName, sizeof(keyName), "preset_%u", slot);
    prefs.begin("led_presets", false);
    prefs.remove(keyName);
    prefs.end();
    
    Serial.printf("Cleared preset %u\n", slot);
  }
  
  void printPresets() {
    Serial.println("\n========== Preset Slots ==========");
    const char* slotNames[] = {"RED", "GREEN", "YELLOW", "BLUE"};
    
    for (uint8_t i = 0; i < Config::PRESET_SLOTS; ++i) {
      Serial.printf("%s: ", slotNames[i]);
      if (presets[i].valid) {
        Serial.printf("%s (%u-bit)\n", presets[i].name, presets[i].resolution);
      } else {
        Serial.println("Empty");
      }
    }
    
    Serial.println("\nUsage:");
    Serial.println("  Press SOURCE, then color key to save");
    Serial.println("  Press color key to load preset");
    Serial.println("==================================\n");
  }

  // -------------------- Visual Feedback --------------------
  
  void startFlash(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    if (flashState.active && flashState.channel < Config::CHANNEL_COUNT) {
      ledcWriteDuty(flashState.channel, flashState.savedValue);
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
      Serial.printf("Digit %d out of range\n", digit);
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
    
    if (e.isActive() && (e.type == EffectType::PULSE || e.type == EffectType::HEARTBEAT)) {
      calculateEffectTiming(e);
    }
    
    Serial.printf("Ch%d duty: %.1f%%\n", selectedIndex, e.dutyCycle);
  }
  
  void adjustFlickerIntensity(int32_t direction) {
    Effect &e = effects[selectedIndex];
    e.flickerIntensity = constrain((int)e.flickerIntensity + direction * 10, 0, 100);
    Serial.printf("Ch%d flicker: %u%%\n", selectedIndex, e.flickerIntensity);
  }
  
  void applyFrequencyChangeToChannel(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    calculateEffectTiming(e);
    
    if (!e.isActive()) {
      Serial.printf("Ch%u: freq=%.3fHz (stored)\n", channel, e.frequency);
      return;
    }
    
    e.lastStepTime = millis();
    e.nextUpdateMs = millis();
    Serial.printf("Ch%u: freq=%.3fHz updated\n", channel, e.frequency);
  }
  
  void handleColorKey(uint8_t slot) {
    if (savePresetMode) {
      // Save mode: save to selected slot
      savePreset(slot);
      savePresetMode = false;
      Serial.println("Save mode deactivated.");
    } else {
      // Normal mode: load preset
      loadPreset(slot);
    }
  }
  
  void processSamsungCommand(uint32_t rawCmd) {
    const uint8_t cmd = rawCmd & 0xFF;
    
    // Handle color keys for presets
    if (cmd == RemoteKeys::RED || cmd == RemoteKeys::GREEN || 
        cmd == RemoteKeys::YELLOW || cmd == RemoteKeys::BLUE) {
      uint8_t slot = 0xFF;
      switch (cmd) {
        case RemoteKeys::RED: slot = 0; break;
        case RemoteKeys::GREEN: slot = 1; break;
        case RemoteKeys::YELLOW: slot = 2; break;
        case RemoteKeys::BLUE: slot = 3; break;
      }
      handleColorKey(slot);
      return;
    }
    
    switch (cmd) {
      case RemoteKeys::KEY_0 ... RemoteKeys::KEY_7: {
        int digit = (cmd == RemoteKeys::KEY_0) ? 0 : (cmd - RemoteKeys::KEY_1 + 1);
        handleDigitSelection(digit);
        break;
      }
        
      case RemoteKeys::KEY_8:
      case RemoteKeys::KEY_9:
        Serial.printf("Digits 8/9 not supported\n");
        startFlash(selectedIndex);
        break;
        
      case RemoteKeys::VOL_UP:
        adjustBrightnessByDelta(+1);
        break;
        
      case RemoteKeys::VOL_DOWN:
        adjustBrightnessByDelta(-1);
        break;
        
      case RemoteKeys::MUTE: {
        const uint32_t maxV = maxForRes(currentResolution);
        brightness[selectedIndex] = (brightness[selectedIndex] == 0) ? (maxV / 2) : 0;
        ledcWriteDuty(selectedIndex, brightness[selectedIndex]);
        Serial.printf("Mute toggle ch%d\n", selectedIndex);
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
          Serial.printf("Next effect ch%d: %s\n", sourceChainChannel,
                       effectTypeToString(e.nextEffectType));
        } else {
          reconfigureLEDCTimer(currentResolution + ((cmd == RemoteKeys::UP) ? 1 : -1));
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
          Serial.printf("Ch%d: Paused\n", selectedIndex);
        } else if (e.type != EffectType::NONE) {
          e.lastStepTime = millis();
          e.nextUpdateMs = millis();
          e.state = EffectState::RUNNING;
          Serial.printf("Ch%d: Resumed\n", selectedIndex);
        }
        break;
      }
        
      case RemoteKeys::SOURCE:
        if (savePresetMode) {
          // Cancel save mode
          savePresetMode = false;
          Serial.println("Save mode cancelled.");
        } else {
          // Enter save mode
          savePresetMode = true;
          Serial.println("\n*** PRESET SAVE MODE ***");
          Serial.println("Press color key to save:");
          Serial.println("  RED    - Slot 1");
          Serial.println("  GREEN  - Slot 2");
          Serial.println("  YELLOW - Slot 3");
          Serial.println("  BLUE   - Slot 4");
          Serial.println("Press SOURCE again to cancel");
          Serial.println("************************\n");
          
          // Visual indicator - flash all channels briefly
          for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
            ledcWriteDuty(i, maxForRes(currentResolution));
          }
          delay(100);
          for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
            ledcWriteDuty(i, brightness[i]);
          }
        }
        break;
        
      case RemoteKeys::CH_LIST:
        isChaining = true;
        sourceChainChannel = selectedIndex;
        Serial.printf("Chaining mode: ch%d\n", selectedIndex);
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
        
      case RemoteKeys::SETTINGS:
        printPresets();
        break;
        
      default:
        Serial.printf("Unhandled: 0x%02X\n", cmd);
        break;
    }
  }

  // -------------------- Status Display --------------------
  
  void printEffectsList() {
    Serial.println("\n========== Available Effects ==========");
    Serial.println("Use REWIND/FORWARD to cycle:");
    Serial.println("  0. None");
    Serial.println("  1. Pulse - Square wave");
    Serial.println("  2. Fade In/Out - Smooth fading");
    Serial.println("  3. Fade In - One-shot up");
    Serial.println("  4. Fade Out - One-shot down");
    Serial.println("  5. Breathing - Ease in/out");
    Serial.println("  6. Strobe - Fast flash");
    Serial.println("  7. Heartbeat - Double pulse");
    Serial.println("  8. Random Flash - Chaotic");
    Serial.println("  9. Sawtooth Up - Ramp/drop");
    Serial.println(" 10. Sawtooth Down - Rise/ramp");
    Serial.println(" 11. Triangle - Linear wave");
    Serial.println(" 12. Candle Flicker - Realistic");
    Serial.println(" 13. Smooth Random - Soft transitions");
    Serial.println("=======================================\n");
  }
  
  void printChannelStatus(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    const Effect &e = effects[channel];
    const uint32_t maxV = maxForRes(currentResolution);
    
    Serial.println("\n--- Channel Status ---");
    Serial.printf("Channel: %d (GPIO %u)\n", channel, Config::PWM_PINS[channel]);
    Serial.printf("Brightness: %lu/%lu (%.1f%%)\n", 
                 (unsigned long)brightness[channel], (unsigned long)maxV,
                 100.0f * brightness[channel] / maxV);
    Serial.printf("Effect: %s [%s]\n", 
                 effectTypeToString(e.type), effectModeToString(e.mode));
    
    if (e.type != EffectType::NONE) {
      Serial.printf("Frequency: %.3f Hz\n", e.frequency);
      
      if (e.type == EffectType::PULSE || e.type == EffectType::HEARTBEAT) {
        Serial.printf("Duty: %.1f%% (on=%ums off=%ums)\n", 
                     e.dutyCycle, e.onTime, e.offTime);
      } else if (e.type == EffectType::CANDLE_FLICKER) {
        Serial.printf("Flicker: %u%%\n", e.flickerIntensity);
      }
      
      Serial.printf("State: %s\n", e.isActive() ? "Active" : "Idle");
      Serial.printf("Cycles: %lu\n", (unsigned long)e.cycleCount);
      
      if (e.linkedChannel < Config::CHANNEL_COUNT) {
        Serial.printf("Linked: Ch%u -> %s\n", 
                     e.linkedChannel, effectTypeToString(e.nextEffectType));
      }
    }
    Serial.println("---------------------\n");
  }
  
  void printAllChannelsStatus() {
    Serial.println("\n========== All Channels ==========");
    Serial.printf("Resolution: %u-bit | Selected: Ch%d\n\n", 
                 currentResolution, selectedIndex);
    
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      const Effect &e = effects[i];
      const uint32_t maxV = maxForRes(currentResolution);
      const float pct = 100.0f * brightness[i] / maxV;
      
      Serial.printf("Ch%d: %3.0f%% | %s", i, pct, effectTypeToString(e.type));
      
      if (e.type != EffectType::NONE) {
        Serial.printf(" [%s] %.1fHz", effectModeToString(e.mode), e.frequency);
        if (e.isActive()) Serial.print(" *");
      }
      Serial.println();
    }
    Serial.println("==================================\n");
  }

  // -------------------- Persistence --------------------
  
  void saveSettings() {
    Preferences prefs;
    prefs.begin("led_settings", false);
    prefs.putBytes("brightness", brightness, sizeof(brightness));
    prefs.putBytes("effects", effects, sizeof(effects));
    prefs.putUChar("resolution", currentResolution);
    prefs.end();
    
    indicateSave();
    Serial.println("Settings saved.");
  }
  
  void loadSettings() {
    Preferences prefs;
    prefs.begin("led_settings", true);
    
    size_t brightSize = prefs.getBytes("brightness", brightness, sizeof(brightness));
    size_t effectSize = prefs.getBytes("effects", effects, sizeof(effects));
    currentResolution = prefs.getUChar("resolution", Config::DEFAULT_RESOLUTION);
    
    prefs.end();
    
    currentResolution = constrain(currentResolution, Config::MIN_RESOLUTION, Config::MAX_RESOLUTION);
    
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      effects[i].validate();
      effects[i].reset();
    }
    
    if (brightSize > 0 || effectSize > 0) {
      Serial.println("Settings loaded.");
    } else {
      Serial.println("Using defaults.");
    }
  }

public:
  LEDController() {
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      lastDuty[i] = UINT32_MAX;
    }
  }
  
  void begin() {
    Serial.begin(115200);
    delay(500);
    
    printWelcomeMessage();
    
    loadSettings();
    loadPresetsFromFlash();
    setupLEDCDriver();
    
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      ledcWriteDuty(i, brightness[i]);
      yield();
    }
    
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      if (effects[i].type != EffectType::NONE) {
        Serial.printf("Autostart ch%d: %s\n", i, effectTypeToString(effects[i].type));
        startEffect(i, effects[i].type, effects[i].mode);
      }
    }
    
    delay(200);
    IrReceiver.begin(Config::IR_PIN, DISABLE_LED_FEEDBACK);
    Serial.printf("IR: GPIO %u\n", Config::IR_PIN);
    
    startFlash(selectedIndex);
    Serial.printf("Ready: %u-bit\n\n", currentResolution);
  }
  
  void update() {
    updateFlash();
    updateSaveIndicator();
    updateEffects();
    
    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.protocol == SAMSUNG &&
          IrReceiver.decodedIRData.address == 0x7) {
        const uint8_t cmd = IrReceiver.decodedIRData.command & 0xFF;
        Serial.printf("CMD: 0x%02X\n", cmd);
        processSamsungCommand(IrReceiver.decodedIRData.command);
      }
      IrReceiver.resume();
    }
  }
  
private:
  static void printWelcomeMessage() {
    Serial.println("\n======================================================");
    Serial.println("   ESP32 LED Controller - 14 Effects + 4 Presets");
    Serial.println("======================================================\n");
    Serial.println("Basic Controls:");
    Serial.println("  0-7            Select channel");
    Serial.println("  VOL+/VOL-      Brightness");
    Serial.println("  CH+/CH-        Duty cycle / Flicker intensity");
    Serial.println("  UP/DOWN        Resolution (or next effect type)");
    Serial.println("  LEFT/RIGHT     Effect frequency");
    Serial.println("  MUTE           Toggle 0%/50%");
    Serial.println();
    Serial.println("Effects:");
    Serial.println("  REWIND/FWD     Cycle through all effects");
    Serial.println("  PLAY           Fade Out (one-shot)");
    Serial.println("  PAUSE          Pause/Resume effect");
    Serial.println("  STOP           Stop effect");
    Serial.println();
    Serial.println("Presets (Press SOURCE first, then color to save):");
    Serial.println("  RED            Preset slot 1");
    Serial.println("  GREEN          Preset slot 2");
    Serial.println("  YELLOW         Preset slot 3");
    Serial.println("  BLUE           Preset slot 4");
    Serial.println();
    Serial.println("Advanced:");
    Serial.println("  CH LIST        Chain effects");
    Serial.println("  INFO           Channel status");
    Serial.println("  MENU           All channels");
    Serial.println("  GUIDE          List all effects");
    Serial.println("  SETTINGS       Show presets");
    Serial.println("  SOURCE         Enter save mode (then press color)");
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
  yield();
}
