// ESP32 LEDC PWM + IR Remote Controller (Optimized)
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
  FADE_IN_OUT
};

struct Effect {
  EffectType type = EffectType::NONE;
  EffectState state = EffectState::IDLE;
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
  bool oneShot = false;
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
      default: return "Unknown";
    }
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
  
  void applyFrequencyChangeToChannel(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    e.frequency = constrain(e.frequency, Config::MIN_EFFECT_FREQ, Config::MAX_EFFECT_FREQ);
    
    if (e.type == EffectType::NONE) {
      Serial.printf("Ch%u: freq set to %.3f Hz (stored)\n", channel, e.frequency);
      return;
    }
    
    const float period = 1000.0f / e.frequency;
    
    if (e.type == EffectType::PULSE) {
      const float duty = e.dutyCycle / 100.0f;
      e.onTime = (uint32_t)(period * duty + 0.5f);
      e.offTime = (uint32_t)(period * (1.0f - duty) + 0.5f);
      e.lastStepTime = millis();
      e.nextUpdateMs = millis();
      
      ledcWriteDuty(channel, (e.state == EffectState::PULSE_ON) ? e.savedBrightness : 0);
      Serial.printf("Ch%u PULSE: %.3fHz, on=%ums, off=%ums\n", 
                    channel, e.frequency, e.onTime, e.offTime);
    } else if (e.type == EffectType::FADE_IN_OUT) {
      e.duration = (uint32_t)(period / 2.0f + 0.5f);
      e.lastStepTime = millis();
      e.nextUpdateMs = millis();
      Serial.printf("Ch%u FADE: %.3fHz, duration=%ums/direction\n",
                    channel, e.frequency, e.duration);
    }
  }
  
  void startEffect(uint8_t channel, EffectType type, bool oneShot) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    if (e.frequency <= 0.0f) e.frequency = 1.0f;
    
    e.type = type;
    e.oneShot = oneShot;
    e.savedBrightness = brightness[channel];
    e.lastStepTime = millis();
    e.nextUpdateMs = millis();
    
    if (type == EffectType::FADE_IN_OUT) {
      e.state = EffectState::FADE_UP;
      const float period = 1000.0f / e.frequency;
      e.duration = (uint32_t)(period / 2.0f + 0.5f);
      ledcWriteDuty(channel, 0);
      Serial.printf("Started FADE on ch%u: %.3fHz, %ums\n", 
                    channel, e.frequency, e.duration);
    } else if (type == EffectType::PULSE) {
      e.state = EffectState::PULSE_ON;
      const float period = 1000.0f / e.frequency;
      const float duty = e.dutyCycle / 100.0f;
      e.onTime = (uint32_t)(period * duty + 0.5f);
      e.offTime = (uint32_t)(period * (1.0f - duty) + 0.5f);
      ledcWriteDuty(channel, e.savedBrightness);
      Serial.printf("Started PULSE on ch%u: %.3fHz, on=%ums, off=%ums\n",
                    channel, e.frequency, e.onTime, e.offTime);
    }
  }
  
  void stopEffect(uint8_t channel) {
    if (channel >= Config::CHANNEL_COUNT) return;
    
    Effect &e = effects[channel];
    e.state = EffectState::IDLE;
    e.type = EffectType::NONE;
    brightness[channel] = e.savedBrightness;
    ledcWriteDuty(channel, brightness[channel]);
    
    Serial.printf("Stopped effect on ch%u, brightness=%lu\n", 
                  channel, (unsigned long)brightness[channel]);
  }
  
  void updateEffects() {
    const uint32_t now = millis();
    
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      Effect &e = effects[i];
      if (e.state == EffectState::IDLE) continue;
      
      bool effectFinished = false;
      
      if (e.type == EffectType::FADE_IN_OUT) {
        if ((int32_t)(now - e.nextUpdateMs) < 0) continue;
        e.nextUpdateMs = now + Config::FADE_STEP_MS;
        
        const uint32_t elapsed = now - e.lastStepTime;
        const uint32_t dur = max(e.duration, 1U);
        const float t = min((float)elapsed / dur, 1.0f);
        
        if (t >= 1.0f) {
          if (e.oneShot) {
            effectFinished = true;
          } else {
            e.lastStepTime = now;
            if (e.state == EffectState::FADE_UP) {
              e.state = EffectState::FADE_DOWN;
              ledcWriteDuty(i, e.savedBrightness);
            } else {
              e.state = EffectState::FADE_UP;
              ledcWriteDuty(i, 0);
            }
          }
        } else {
          const uint32_t target = e.savedBrightness;
          const uint32_t duty = (e.state == EffectState::FADE_UP) ?
            (uint32_t)(target * t + 0.5f) :
            (uint32_t)(target * (1.0f - t) + 0.5f);
          ledcWriteDuty(i, duty);
        }
      } else if (e.type == EffectType::PULSE) {
        if (e.state == EffectState::PULSE_ON && now - e.lastStepTime > e.onTime) {
          if (e.oneShot) {
            effectFinished = true;
          } else {
            e.lastStepTime = now;
            e.state = EffectState::PULSE_OFF;
            ledcWriteDuty(i, 0);
          }
        } else if (e.state == EffectState::PULSE_OFF && now - e.lastStepTime > e.offTime) {
          e.lastStepTime = now;
          e.state = EffectState::PULSE_ON;
          ledcWriteDuty(i, e.savedBrightness);
        }
      }
      
      if (effectFinished) {
        const uint8_t linked = e.linkedChannel;
        const EffectType next = e.nextEffectType;
        stopEffect(i);
        if (linked < Config::CHANNEL_COUNT) {
          startEffect(linked, next, true);
        }
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
        startEffect(flashState.channel, EffectType::FADE_IN_OUT, false);
      }
    }
    
    // Setup new flash
    flashState.prevWasFading = (effects[channel].type == EffectType::FADE_IN_OUT);
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
          startEffect(ch, EffectType::FADE_IN_OUT, false);
        }
      }
      flashState.active = false;
    }
  }
  
  void indicateSave() {
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      wasFading[i] = (effects[i].type == EffectType::FADE_IN_OUT);
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
          startEffect(i, EffectType::FADE_IN_OUT, false);
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
    Serial.printf("Ch%d brightness: %lu/%lu\n", selectedIndex, 
                  (unsigned long)brightness[selectedIndex], (unsigned long)maxV);
  }
  
  void processSamsungCommand(uint32_t rawCmd) {
    const uint8_t cmd = rawCmd & 0xFF;
    
    switch (cmd) {
      case RemoteKeys::KEY_0 ... RemoteKeys::KEY_7:
        handleDigitSelection(cmd == RemoteKeys::KEY_0 ? 0 : 
                           (cmd - RemoteKeys::KEY_1 + 1));
        break;
        
      case RemoteKeys::KEY_8:
      case RemoteKeys::KEY_9:
        Serial.printf("Digits 8/9 not supported (max %d channels)\n", Config::CHANNEL_COUNT);
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
          if (nextType > (int)EffectType::FADE_IN_OUT) nextType = (int)EffectType::PULSE;
          if (nextType < (int)EffectType::PULSE) nextType = (int)EffectType::FADE_IN_OUT;
          e.nextEffectType = (EffectType)nextType;
          Serial.printf("Next effect for ch%d: %s\n", sourceChainChannel,
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
          Serial.printf("Ch%d freq: %.3f Hz\n", target, e.frequency);
        }
        break;
      }
      
      case RemoteKeys::GREEN:
        if (effects[selectedIndex].type == EffectType::FADE_IN_OUT) {
          stopEffect(selectedIndex);
        } else {
          startEffect(selectedIndex, EffectType::FADE_IN_OUT, false);
        }
        break;
        
      case RemoteKeys::RED:
        if (effects[selectedIndex].type == EffectType::PULSE) {
          stopEffect(selectedIndex);
        } else {
          startEffect(selectedIndex, EffectType::PULSE, false);
        }
        break;
        
      case RemoteKeys::SOURCE:
        saveSettings();
        break;
        
      case RemoteKeys::CH_LIST:
        isChaining = true;
        sourceChainChannel = selectedIndex;
        Serial.printf("Chaining mode: ch%d active. Press digit for target.\n", selectedIndex);
        startFlash(selectedIndex);
        break;
        
      default:
        Serial.printf("Unhandled key: 0x%02X\n", cmd);
        break;
    }
  }

  // -------------------- Persistence --------------------
  
  void saveSettings() {
    Preferences prefs;
    prefs.begin("led_settings", false);
    prefs.putBytes("brightness", brightness, sizeof(brightness));
    prefs.putBytes("effects", effects, sizeof(effects));
    prefs.end();
    
    indicateSave();
    Serial.println("Settings saved.");
  }
  
  void loadSettings() {
    Preferences prefs;
    prefs.begin("led_settings", true);
    prefs.getBytes("brightness", brightness, sizeof(brightness));
    prefs.getBytes("effects", effects, sizeof(effects));
    prefs.end();
    
    // Validate loaded data
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      Effect &e = effects[i];
      if (e.frequency <= 0.0f) e.frequency = 1.0f;
      if (e.dutyCycle <= 0.0f || e.dutyCycle > 100.0f) {
        e.dutyCycle = Config::DEFAULT_DUTY_CYCLE;
      }
      if (e.duration == 0) e.duration = 500;
      e.state = EffectState::IDLE;
      e.lastStepTime = 0;
      e.nextUpdateMs = 0;
    }
    
    Serial.println("Settings loaded.");
  }

public:
  LEDController() {
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      lastDuty[i] = UINT32_MAX;  // Force first write
    }
  }
  
  void begin() {
    Serial.begin(115200);
    delay(500);
    
    printWelcomeMessage();
    
    loadSettings();
    setupLEDCDriver();
    
    // Initialize hardware to stored brightness
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      ledcWriteDuty(i, brightness[i]);
      yield();
    }
    
    // Autostart effects
    for (uint8_t i = 0; i < Config::CHANNEL_COUNT; ++i) {
      if (effects[i].type != EffectType::NONE) {
        applyFrequencyChangeToChannel(i);
        Serial.printf("Autostart %s on ch%d (%.3fHz)\n",
                     effectTypeToString(effects[i].type), i, effects[i].frequency);
        startEffect(i, effects[i].type, false);
      }
    }
    
    // Initialize IR receiver
    delay(200);
    IrReceiver.begin(Config::IR_PIN, DISABLE_LED_FEEDBACK);
    Serial.printf("IR receiver on GPIO %u\n", Config::IR_PIN);
    
    // Initial flash
    startFlash(selectedIndex);
    Serial.printf("Resolution: %u-bit (0..%lu)\n\n", 
                  currentResolution, (unsigned long)maxForRes(currentResolution));
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
    Serial.println("    ESP32 Advanced LED Controller with IR Remote");
    Serial.println("======================================================\n");
    Serial.println("Controls:");
    Serial.println("------------------------------------------------------");
    Serial.println("  0-7          Select LED channel");
    Serial.println("  VOL+/VOL-    Adjust brightness");
    Serial.println("  UP/DOWN      Change PWM resolution");
    Serial.println("  LEFT/RIGHT   Adjust effect frequency");
    Serial.println();
    Serial.println("  RED          Toggle Pulse effect");
    Serial.println("  GREEN        Toggle Fade effect");
    Serial.println("  CH LIST      Chain effects (source→target)");
    Serial.println();
    Serial.println("  SOURCE       Save settings");
    Serial.println("  MUTE         Toggle 0% / 50%");
    Serial.println("------------------------------------------------------\n");
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
