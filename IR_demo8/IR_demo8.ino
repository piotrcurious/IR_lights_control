// ESP32 LEDC PWM + IR remote (driver/ledc API — fixed initializer order)
// IR receiver on GPIO15
// Only accepts keys from SAMSUNG protocol with address 0x7
// Numeric keys 0-7 select PWM pin; VOL+/VOL- adjust brightness.
// KEY_UP/KEY_DOWN change LEDC resolution (rescale brightness automatically).
// KEY_LEFT / KEY_RIGHT change effect frequency (per-channel) and are persisted.

#include <Arduino.h>
#include <IRremote.h>
#include <Preferences.h>

#if defined(ARDUINO_ARCH_ESP32)
extern "C" {
#include "driver/ledc.h"
}
#endif

// =================================================================
// 1. Data Structures & Enums
// =================================================================

enum EffectState {
  STATE_IDLE, STATE_PULSE_ON, STATE_PULSE_OFF,
  STATE_FADE_UP, STATE_FADE_DOWN
};

enum EffectType {
  EFFECT_NONE, EFFECT_PULSE, EFFECT_FADE_IN_OUT
};

struct Effect {
  EffectType type = EFFECT_NONE;
  float frequency = 1.0;
  float dutyCycle = 50.0;
  uint32_t duration = 1000;
  uint32_t savedBrightness = 0;
  uint32_t onTime = 0;
  uint32_t offTime = 0;
  bool oneShot = false;
  EffectType nextEffectType = EFFECT_PULSE;
  EffectState state = STATE_IDLE;
  uint32_t lastStepTime = 0;
  uint8_t linkedChannel = 0xFF;
};

namespace RemoteKeys {
  enum KeyCode {
    KEY_POWER = 0xE6, KEY_0 = 0x11, KEY_1 = 0x04, KEY_2 = 0x05, KEY_3 = 0x06,
    KEY_4 = 0x08, KEY_5 = 0x09, KEY_6 = 0x0A, KEY_7 = 0x0C, KEY_8 = 0x0D,
    KEY_9 = 0x0E, KEY_UP = 0x60, KEY_DOWN = 0x61, KEY_LEFT = 0x65,
    KEY_RIGHT = 0x62, KEY_OK = 0x68, KEY_MENU = 0x79, KEY_RED = 0x6c,
    KEY_GREEN = 0x14, KEY_CH_LIST = 0x6B, KEY_YELLOW = 0x15, KEY_BLUE = 0x16,
    KEY_VOL_UP = 0x07, KEY_VOL_DOWN = 0x0b, KEY_CH_UP = 0x12, KEY_CH_DOWN = 0x10,
    KEY_REWIND = 0x45, KEY_PLAY = 0x47, KEY_PAUSE = 0x4A, KEY_FORWARD = 0x48,
    KEY_STOP = 0x46, KEY_SETTINGS = 0x1A, KEY_INFO = 0x1F, KEY_SUBTITLES = 0x25,
    KEY_MUTE = 0x0F, KEY_NETFLIX = 0xF3, KEY_PRIME_VIDEO = 0xF4,
    KEY_GUIDE = 0x4F, KEY_SOURCE = 0x01
  };
}

struct FlashState {
  bool active = false;
  uint32_t savedValue = 0;
  uint8_t channel = 0xFF;
  uint8_t blinksRemaining = 0;
  uint32_t nextToggleMs = 0;
  bool isBright = false;
};

struct SaveIndicatorState {
  bool active = false;
  uint8_t blinksRemaining = 0;
  uint32_t nextToggleMs = 0;
  bool isBright = false;
};

// =================================================================
// 2. Global Constants & Variables
// =================================================================

// Hardware & System Configuration
static const uint8_t IR_PIN = 15;
const uint8_t pwmPins[8] = {2, 4, 16, 17, 18, 19, 21, 22};
const int LEDC_CHANNEL_COUNT = 8;
const int LEDC_FREQ = 5000;
const ledc_mode_t LEDC_MODE = LEDC_HIGH_SPEED_MODE;
const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;

// Effect Configuration
const float MIN_EFFECT_FREQ = 0.1f;
const float MAX_EFFECT_FREQ = 50.0f;
const float FREQ_STEP_RATIO = 1.1f;

// Global State
Effect effects[LEDC_CHANNEL_COUNT];
uint8_t currentResolution = 8;
uint32_t brightness[LEDC_CHANNEL_COUNT];
int selectedIndex = 0;
bool isChaining = false;
int sourceChainChannel = -1;
FlashState flashState;
SaveIndicatorState saveIndicatorState;

// Constants for Resolution
const uint8_t MIN_RESOLUTION = 1;
const uint8_t MAX_RESOLUTION = 13;

// =================================================================
// 3. Forward Declarations
// =================================================================

// --- Utility ---
const char* effectTypeToString(EffectType type);

// --- LEDC & Hardware Control ---
void setupLEDCDriver();
void reconfigureLEDCTimer(uint8_t newRes);
static inline void ledcWriteDuty(uint8_t channel, uint32_t duty);
static inline uint32_t maxForRes(uint8_t res);
static inline uint32_t scaleValue(uint32_t value, uint8_t oldRes, uint8_t newRes);

// --- IR Remote Handling ---
void processSamsungCommand(uint32_t rawCmd);
void handleDigitSelection(int digit);
void adjustBrightnessByDelta(int32_t deltaSteps);

// --- Effect Management ---
void startEffect(uint8_t channel, EffectType type, bool oneShot);
void stopEffect(uint8_t channel);
void updateEffects();
void applyFrequencyChangeToChannel(uint8_t channel);

// --- Visual Feedback ---
void startFlash(uint8_t channel);
void updateFlash();
void indicateSave();
void updateSaveIndicator();

// --- Persistence ---
void saveSettings();
void loadSettings();

// =================================================================
// 4. Main Sketch (Setup & Loop)
// =================================================================

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 500) { /* tiny non-blocking wait */ }

  Serial.println();
  Serial.println("======================================================");
  Serial.println("    ESP32 Advanced LED Controller with IR Remote");
  Serial.println("======================================================");
  Serial.println();
  Serial.println("Usage Instructions:");
  Serial.println("-------------------");
  Serial.println("Digits (0-7)     - Select active LED channel.");
  Serial.println("VOL+ / VOL-      - Adjust brightness of the selected channel.");
  Serial.println("UP / DOWN        - Adjust PWM resolution (bit depth) for all channels.");
  Serial.println("LEFT / RIGHT     - Adjust effect frequency for the selected channel.");
  Serial.println();
  Serial.println("Effects:");
  Serial.println("-------------------");
  Serial.println("RED Key          - Toggle Pulse effect on/off for the selected channel.");
  Serial.println("GREEN Key        - Toggle Fade In/Out effect on/off for the selected channel.");
  Serial.println("CH LIST Key      - Enter Chaining mode. First press selects source, next digit (0-7) selects target.");
  Serial.println("  -> In chaining mode: UP/DOWN cycles next effect, LEFT/RIGHT adjusts frequency.");
  Serial.println();
  Serial.println("System:");
  Serial.println("-------------------");
  Serial.println("SOURCE Key       - Save all current brightness and effect settings to persistent storage.");
  Serial.println("MUTE Key         - Toggle brightness between zero and 50%.");
  Serial.println();
  Serial.println("System starting...");

  memset(brightness, 0, sizeof(brightness));
  for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
    effects[i].frequency = 1.0f;
    effects[i].dutyCycle = 50.0f;
    effects[i].duration = 500;
    effects[i].state = STATE_IDLE;
    effects[i].lastStepTime = 0;
    effects[i].linkedChannel = 0xFF;
  }

  loadSettings();
  setupLEDCDriver();
  ledc_fade_func_install(0);

  for (int i = 0; i < LEDC_CHANNEL_COUNT; i++) {
    if (effects[i].type != EFFECT_NONE) {
      applyFrequencyChangeToChannel(i);
      Serial.printf("Autostarting effect %s on channel %d (freq=%.3fHz)\n",
                    effectTypeToString(effects[i].type), i, effects[i].frequency);
      startEffect(i, effects[i].type, false);
    }
  }

  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  Serial.printf("IR receiver started on pin %u\n", IR_PIN);
  startFlash(selectedIndex);
  Serial.printf("Initial resolution: %u, brightness range 0..%lu\n", currentResolution, (unsigned long)maxForRes(currentResolution));
}

void loop() {
  updateFlash();
  updateSaveIndicator();
  updateEffects();

  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == SAMSUNG &&
        IrReceiver.decodedIRData.address == 0x7) {
      Serial.printf("Command 0x%02X\n", (unsigned)IrReceiver.decodedIRData.command & 0xFF);
      processSamsungCommand(IrReceiver.decodedIRData.command);
    }
    IrReceiver.resume();
  }
  yield();
}

// =================================================================
// 5. Utility Functions
// =================================================================

const char* effectTypeToString(EffectType type) {
  switch (type) {
    case EFFECT_NONE: return "None";
    case EFFECT_PULSE: return "Pulse";
    case EFFECT_FADE_IN_OUT: return "Fade In/Out";
    default: return "Unknown";
  }
}

// =================================================================
// 6. LEDC & Hardware Control
// =================================================================

static inline uint32_t maxForRes(uint8_t res) {
  if (res >= 32) return UINT32_MAX;
  return ((uint32_t)1 << res) - 1;
}

static inline uint32_t scaleValue(uint32_t value, uint8_t oldRes, uint8_t newRes) {
  uint32_t oldMax = maxForRes(oldRes);
  uint32_t newMax = maxForRes(newRes);
  if (oldMax == 0) return 0;
  uint64_t tmp = (uint64_t)value * (uint64_t)newMax;
  tmp = (tmp + oldMax/2) / oldMax;
  if (tmp > newMax) tmp = newMax;
  return (uint32_t)tmp;
}

static inline void ledcWriteDuty(uint8_t channel, uint32_t duty) {
  if (channel >= LEDC_CHANNEL_COUNT) return;
  uint32_t maxV = maxForRes(currentResolution);
  if (duty > maxV) duty = maxV;
  ledc_set_duty(LEDC_MODE, (ledc_channel_t)channel, duty);
  ledc_update_duty(LEDC_MODE, (ledc_channel_t)channel);
}

void setupLEDCDriver() {
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.speed_mode = LEDC_MODE;
  ledc_timer.duty_resolution = (ledc_timer_bit_t)currentResolution;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.freq_hz = LEDC_FREQ;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
    ledc_channel_config_t ch_conf;
    memset(&ch_conf, 0, sizeof(ch_conf));
    ch_conf.channel = (ledc_channel_t)ch;
    ch_conf.duty = 0;
    ch_conf.gpio_num = pwmPins[ch];
    ch_conf.speed_mode = LEDC_MODE;
    ch_conf.hpoint = 0;
    ch_conf.timer_sel = LEDC_TIMER;
    ledc_channel_config(&ch_conf);
    ledcWriteDuty(ch, brightness[ch]);
  }
}

void reconfigureLEDCTimer(uint8_t newRes) {
  if (newRes < MIN_RESOLUTION) newRes = MIN_RESOLUTION;
  if (newRes > MAX_RESOLUTION) newRes = MAX_RESOLUTION;
  if (newRes == currentResolution) return;

  uint8_t oldRes = currentResolution;
  Serial.printf("Reconfiguring LEDC resolution %u -> %u\n", oldRes, newRes);

  for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
    brightness[ch] = scaleValue(brightness[ch], oldRes, newRes);
  }
  if (flashState.active && flashState.channel < LEDC_CHANNEL_COUNT) {
    flashState.savedValue = scaleValue(flashState.savedValue, oldRes, newRes);
  }

  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.speed_mode = LEDC_MODE;
  ledc_timer.duty_resolution = (ledc_timer_bit_t)newRes;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.freq_hz = LEDC_FREQ;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
    ledc_channel_config_t ch_conf;
    memset(&ch_conf, 0, sizeof(ch_conf));
    ch_conf.channel = (ledc_channel_t)ch;
    ch_conf.duty = 0;
    ch_conf.gpio_num = pwmPins[ch];
    ch_conf.speed_mode = LEDC_MODE;
    ch_conf.hpoint = 0;
    ch_conf.timer_sel = LEDC_TIMER;
    ledc_channel_config(&ch_conf);

    if (flashState.active && flashState.channel == ch) {
      ledcWriteDuty(ch, flashState.isBright ? maxForRes(newRes) : 0);
    } else {
      ledcWriteDuty(ch, brightness[ch]);
    }
  }
  currentResolution = newRes;
}


// =================================================================
// 7. IR Remote Handling
// =================================================================

void handleDigitSelection(int digit) {
  if (digit < 0 || digit >= LEDC_CHANNEL_COUNT) {
    Serial.printf("Digit %d out of range (0..%d)\n", digit, LEDC_CHANNEL_COUNT - 1);
    return;
  }

  if (isChaining) {
    if (sourceChainChannel != -1 && sourceChainChannel < LEDC_CHANNEL_COUNT) {
      effects[sourceChainChannel].linkedChannel = digit;
      Serial.printf("Chained channel %d -> %d\n", sourceChainChannel, digit);
    }
    isChaining = false;
    sourceChainChannel = -1;
  } else {
    selectedIndex = digit;
    Serial.printf("Selected index %d (pin %u)\n", selectedIndex, (unsigned)pwmPins[selectedIndex]);
    startFlash(selectedIndex);
  }
}

void adjustBrightnessByDelta(int32_t deltaSteps) {
  uint32_t maxV = maxForRes(currentResolution);
  if ((deltaSteps > 0 && brightness[selectedIndex] >= maxV) ||
      (deltaSteps < 0 && brightness[selectedIndex] == 0)) {
    startFlash(selectedIndex);
    return;
  }

  uint32_t step = maxV / 16; if (step == 0) step = 1;
  int64_t newV = (int64_t)brightness[selectedIndex] + (int64_t)deltaSteps * step;
  if (newV < 0) newV = 0;
  if ((uint64_t)newV > maxV) newV = maxV;

  brightness[selectedIndex] = (uint32_t)newV;
  ledcWriteDuty(selectedIndex, brightness[selectedIndex]);
  Serial.printf("Channel %d brightness -> %lu/%lu\n", selectedIndex, (unsigned long)brightness[selectedIndex], (unsigned long)maxV);
}

void processSamsungCommand(uint32_t rawCmd) {
  uint8_t cmd = rawCmd & 0xFF;
  switch (cmd) {
    case RemoteKeys::KEY_0: handleDigitSelection(0); break;
    case RemoteKeys::KEY_1: handleDigitSelection(1); break;
    case RemoteKeys::KEY_2: handleDigitSelection(2); break;
    case RemoteKeys::KEY_3: handleDigitSelection(3); break;
    case RemoteKeys::KEY_4: handleDigitSelection(4); break;
    case RemoteKeys::KEY_5: handleDigitSelection(5); break;
    case RemoteKeys::KEY_6: handleDigitSelection(6); break;
    case RemoteKeys::KEY_7: handleDigitSelection(7); break;
    case RemoteKeys::KEY_8:
    case RemoteKeys::KEY_9:
      Serial.printf("Digits 8/9 unsupported\n");
      startFlash(selectedIndex);
      break;

    case RemoteKeys::KEY_VOL_UP: adjustBrightnessByDelta(+1); break;
    case RemoteKeys::KEY_VOL_DOWN: adjustBrightnessByDelta(-1); break;

    case RemoteKeys::KEY_MUTE:
      brightness[selectedIndex] = (brightness[selectedIndex] == 0) ? (maxForRes(currentResolution) / 2) : 0;
      ledcWriteDuty(selectedIndex, brightness[selectedIndex]);
      Serial.printf("Mute toggle channel %d\n", selectedIndex);
      break;

    case RemoteKeys::KEY_UP:
    case RemoteKeys::KEY_DOWN:
      if (isChaining && sourceChainChannel != -1) {
        int typeInt = (int)effects[sourceChainChannel].nextEffectType;
        typeInt += (cmd == RemoteKeys::KEY_UP) ? 1 : -1;
        if (typeInt > (int)EFFECT_FADE_IN_OUT) typeInt = (int)EFFECT_PULSE;
        if (typeInt < (int)EFFECT_PULSE) typeInt = (int)EFFECT_FADE_IN_OUT;
        effects[sourceChainChannel].nextEffectType = (EffectType)typeInt;
        Serial.printf("Next effect for chain: %s\n", effectTypeToString(effects[sourceChainChannel].nextEffectType));
      } else {
        reconfigureLEDCTimer(currentResolution + ((cmd == RemoteKeys::KEY_UP) ? 1 : -1));
      }
      break;

    case RemoteKeys::KEY_LEFT:
    case RemoteKeys::KEY_RIGHT: {
      int targetChannel = (isChaining && sourceChainChannel != -1) ? sourceChainChannel : selectedIndex;
      if (targetChannel < 0 || targetChannel >= LEDC_CHANNEL_COUNT) break;

      Effect &e = effects[targetChannel];
      if (e.frequency <= 0.0f) e.frequency = 1.0f;
      e.frequency *= (cmd == RemoteKeys::KEY_RIGHT) ? FREQ_STEP_RATIO : (1.0f / FREQ_STEP_RATIO);
      if (e.frequency < MIN_EFFECT_FREQ) e.frequency = MIN_EFFECT_FREQ;
      if (e.frequency > MAX_EFFECT_FREQ) e.frequency = MAX_EFFECT_FREQ;

      applyFrequencyChangeToChannel(targetChannel);
      Serial.printf("Channel %d frequency -> %.3f Hz\n", targetChannel, e.frequency);
      break;
    }

    case RemoteKeys::KEY_GREEN:
      if (effects[selectedIndex].type == EFFECT_FADE_IN_OUT) stopEffect(selectedIndex);
      else startEffect(selectedIndex, EFFECT_FADE_IN_OUT, false);
      break;

    case RemoteKeys::KEY_RED:
      if (effects[selectedIndex].type == EFFECT_PULSE) stopEffect(selectedIndex);
      else startEffect(selectedIndex, EFFECT_PULSE, false);
      break;

    case RemoteKeys::KEY_SOURCE: saveSettings(); break;

    case RemoteKeys::KEY_CH_LIST:
      isChaining = true;
      sourceChainChannel = selectedIndex;
      Serial.printf("Chaining active for channel %d. Select target channel.\n", selectedIndex);
      startFlash(selectedIndex);
      break;

    default:
      Serial.printf("Samsung key (unhandled): 0x%02X\n", cmd);
      break;
  }
}

// =================================================================
// 8. Effect Management
// =================================================================

void applyFrequencyChangeToChannel(uint8_t channel) {
  if (channel >= LEDC_CHANNEL_COUNT) return;
  Effect &e = effects[channel];

  if (e.frequency < MIN_EFFECT_FREQ) e.frequency = MIN_EFFECT_FREQ;
  if (e.frequency > MAX_EFFECT_FREQ) e.frequency = MAX_EFFECT_FREQ;

  if (e.type == EFFECT_NONE) {
    Serial.printf("Channel %u: frequency set to %.3f Hz (stored)\n", channel, e.frequency);
    return;
  }

  float period = 1000.0f / e.frequency;
  if (e.type == EFFECT_PULSE) {
    float duty = e.dutyCycle / 100.0f;
    e.onTime = (uint32_t)(period * duty + 0.5f);
    e.offTime = (uint32_t)(period * (1.0f - duty) + 0.5f);
    if (e.state == STATE_PULSE_ON) ledcWriteDuty(channel, e.savedBrightness);
    else if (e.state == STATE_PULSE_OFF) ledcWriteDuty(channel, 0);
    e.lastStepTime = millis();
    Serial.printf("Channel %u PULSE freq->%.3fHz, on=%ums off=%ums\n", channel, e.frequency, (unsigned)e.onTime, (unsigned)e.offTime);
  } else if (e.type == EFFECT_FADE_IN_OUT) {
    e.duration = (uint32_t)(period / 2.0f + 0.5f);
    if (e.state == STATE_FADE_UP) {
      ledc_set_fade_with_time(LEDC_MODE, (ledc_channel_t)channel, e.savedBrightness, e.duration);
      ledc_fade_start(LEDC_MODE, (ledc_channel_t)channel, LEDC_FADE_NO_WAIT);
    } else if (e.state == STATE_FADE_DOWN) {
      ledc_set_fade_with_time(LEDC_MODE, (ledc_channel_t)channel, 0, e.duration);
      ledc_fade_start(LEDC_MODE, (ledc_channel_t)channel, LEDC_FADE_NO_WAIT);
    }
    e.lastStepTime = millis();
    Serial.printf("Channel %u FADE freq->%.3fHz, duration=%ums\n", channel, e.frequency, (unsigned)e.duration);
  }
}

void startEffect(uint8_t channel, EffectType type, bool oneShot) {
  if (channel >= LEDC_CHANNEL_COUNT) return;

  if (effects[channel].frequency <= 0.0f) effects[channel].frequency = 1.0f;

  effects[channel].type = type;
  effects[channel].oneShot = oneShot;
  effects[channel].savedBrightness = brightness[channel];
  effects[channel].lastStepTime = millis();

  if (type == EFFECT_FADE_IN_OUT) {
    effects[channel].state = STATE_FADE_UP;
    float period = 1000.0f / effects[channel].frequency;
    effects[channel].duration = (uint32_t)(period / 2.0f + 0.5f);
    ledcWriteDuty(channel, 0);
    ledc_set_fade_with_time(LEDC_MODE, (ledc_channel_t)channel, effects[channel].savedBrightness, effects[channel].duration);
    ledc_fade_start(LEDC_MODE, (ledc_channel_t)channel, LEDC_FADE_NO_WAIT);
    Serial.printf("Started FADE on ch %u: freq=%.3fHz duration=%ums\n", channel, effects[channel].frequency, (unsigned)effects[channel].duration);
  } else if (type == EFFECT_PULSE) {
    effects[channel].state = STATE_PULSE_ON;
    float period = 1000.0f / effects[channel].frequency;
    float duty = effects[channel].dutyCycle / 100.0f;
    effects[channel].onTime = (uint32_t)(period * duty + 0.5f);
    effects[channel].offTime = (uint32_t)(period * (1.0f - duty) + 0.5f);
    ledcWriteDuty(channel, effects[channel].savedBrightness);
    Serial.printf("Started PULSE on ch %u: freq=%.3fHz on=%ums off=%ums\n", channel, effects[channel].frequency, (unsigned)effects[channel].onTime, (unsigned)effects[channel].offTime);
  }
}

void stopEffect(uint8_t channel) {
  if (channel >= LEDC_CHANNEL_COUNT) return;

  if (effects[channel].type == EFFECT_FADE_IN_OUT) {
    ledc_stop(LEDC_MODE, (ledc_channel_t)channel, 0);
  }
  effects[channel].state = STATE_IDLE;
  effects[channel].type = EFFECT_NONE;
  brightness[channel] = effects[channel].savedBrightness;
  ledcWriteDuty(channel, brightness[channel]);
  Serial.printf("Stopped effect on channel %u\n", channel);
}

void updateEffects() {
  uint32_t currentTime = millis();
  for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
    if (effects[i].state == STATE_IDLE) continue;

    bool effectFinished = false;
    if (effects[i].type == EFFECT_FADE_IN_OUT) {
      if (currentTime - effects[i].lastStepTime > effects[i].duration) {
        if (effects[i].oneShot) {
          effectFinished = true;
        } else {
          effects[i].lastStepTime = currentTime;
          if (effects[i].state == STATE_FADE_UP) {
            effects[i].state = STATE_FADE_DOWN;
            ledc_set_fade_with_time(LEDC_MODE, (ledc_channel_t)i, 0, effects[i].duration);
            ledc_fade_start(LEDC_MODE, (ledc_channel_t)i, LEDC_FADE_NO_WAIT);
          } else {
            effects[i].state = STATE_FADE_UP;
            ledc_set_fade_with_time(LEDC_MODE, (ledc_channel_t)i, effects[i].savedBrightness, effects[i].duration);
            ledc_fade_start(LEDC_MODE, (ledc_channel_t)i, LEDC_FADE_NO_WAIT);
          }
        }
      }
    } else if (effects[i].type == EFFECT_PULSE) {
      if (effects[i].state == STATE_PULSE_ON && currentTime - effects[i].lastStepTime > effects[i].onTime) {
        if (effects[i].oneShot) effectFinished = true;
        else {
          effects[i].lastStepTime = currentTime;
          effects[i].state = STATE_PULSE_OFF;
          ledcWriteDuty(i, 0);
        }
      } else if (effects[i].state == STATE_PULSE_OFF && currentTime - effects[i].lastStepTime > effects[i].offTime) {
        effects[i].lastStepTime = currentTime;
        effects[i].state = STATE_PULSE_ON;
        ledcWriteDuty(i, effects[i].savedBrightness);
      }
    }

    if (effectFinished) {
      uint8_t linkedChannel = effects[i].linkedChannel;
      EffectType nextEffect = effects[i].nextEffectType;
      stopEffect(i);
      if (linkedChannel != 0xFF && linkedChannel < LEDC_CHANNEL_COUNT) {
        startEffect(linkedChannel, nextEffect, true);
      }
    }
  }
}


// =================================================================
// 9. Visual Feedback
// =================================================================

void startFlash(uint8_t channel) {
  if (channel >= LEDC_CHANNEL_COUNT) return;

  if (flashState.active && flashState.channel < LEDC_CHANNEL_COUNT) {
    ledcWriteDuty(flashState.channel, flashState.savedValue);
  }

  const uint8_t totalCycles = 3;
  const uint32_t toggleInterval = 80;

  flashState.channel = channel;
  flashState.savedValue = brightness[channel];
  flashState.active = true;
  flashState.blinksRemaining = totalCycles * 2;
  flashState.isBright = true;
  ledcWriteDuty(channel, maxForRes(currentResolution));
  flashState.nextToggleMs = millis() + toggleInterval;
}

void updateFlash() {
  if (!flashState.active) return;

  if ((int32_t)(millis() - flashState.nextToggleMs) >= 0) {
    uint8_t ch = flashState.channel;
    const uint32_t toggleInterval = 80;

    if (flashState.blinksRemaining > 0) {
      flashState.isBright = !flashState.isBright;
      flashState.blinksRemaining--;
      ledcWriteDuty(ch, flashState.isBright ? maxForRes(currentResolution) : 0);
      flashState.nextToggleMs = millis() + toggleInterval;
    } else {
      if (ch < LEDC_CHANNEL_COUNT) {
        ledcWriteDuty(ch, flashState.savedValue);
      }
      flashState.active = false;
    }
  }
}

void indicateSave() {
  saveIndicatorState.active = true;
  saveIndicatorState.blinksRemaining = 4;
  saveIndicatorState.isBright = false;
  saveIndicatorState.nextToggleMs = millis();
}

void updateSaveIndicator() {
  if (!saveIndicatorState.active) return;

  if ((int32_t)(millis() - saveIndicatorState.nextToggleMs) >= 0) {
    const uint32_t toggleInterval = 80;

    if (saveIndicatorState.blinksRemaining > 0) {
      saveIndicatorState.isBright = !saveIndicatorState.isBright;
      saveIndicatorState.blinksRemaining--;
      uint32_t duty = saveIndicatorState.isBright ? maxForRes(currentResolution) : 0;
      for (int ch = 0; ch < LEDC_CHANNEL_COUNT; ++ch) {
        ledcWriteDuty(ch, duty);
      }
      saveIndicatorState.nextToggleMs = millis() + toggleInterval;
    } else {
      for (int i = 0; i < LEDC_CHANNEL_COUNT; ++i) {
        ledcWriteDuty(i, brightness[i]);
      }
      saveIndicatorState.active = false;
    }
  }
}

// =================================================================
// 10. Persistence
// =================================================================

void saveSettings() {
  Preferences preferences;
  preferences.begin("led_settings", false);
  preferences.putBytes("brightness", brightness, sizeof(brightness));
  preferences.putBytes("effects", effects, sizeof(effects));
  preferences.end();
  indicateSave();
  Serial.println("Settings saved to preferences.");
}

void loadSettings() {
  Preferences preferences;
  preferences.begin("led_settings", true);
  preferences.getBytes("brightness", brightness, sizeof(brightness));
  preferences.getBytes("effects", effects, sizeof(effects));
  for (int i = 0; i < LEDC_CHANNEL_COUNT; i++) {
    if (effects[i].frequency <= 0.0f) effects[i].frequency = 1.0f;
    if (effects[i].dutyCycle <= 0.0f || effects[i].dutyCycle > 100.0f) effects[i].dutyCycle = 50.0f;
    if (effects[i].duration == 0) effects[i].duration = 500;
    effects[i].state = STATE_IDLE;
    effects[i].lastStepTime = 0;
  }
  preferences.end();
  Serial.println("Settings loaded from preferences.");
}
