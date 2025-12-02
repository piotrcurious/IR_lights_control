/*
  ESP32 Mesh Sequential Light Controller (single-file)
  - Master forms a token ring and assigns positions to joiners
  - Join with 3s button hold, factory reset/unlock with 10s hold
  - Master IR on GPIO 15 controls start/stop, speed, lock broadcast
  - Full peer list persisted in Preferences on master
  - ACK + retry for ASSIGN, SET_DOWN, LOCK (configurable)
  - Lock flow: master broadcasts small lock string; nodes derive PMK/LMK (SHA256)
    and install them (esp_now_set_pmk + per-peer LMK). After lock, master rejects joins.
  - Nodes still process TRIGGER messages normally while locked.
  - 8 LEDs driven by LEDC; sequence stored in PROGMEM
  - Uses ESP-NOW; designed for neighbor-relay topology (each node forwards to its downstream)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <IRremote.h>
#include "mbedtls/sha256.h"

// ----------------------------- Config -----------------------------------
#define JOIN_BTN_PIN       0      // button pin (active LOW). Adjust if needed
#define IR_PIN             15     // IR receiver pin for master
#define LED_COUNT          8
const int LED_PINS[LED_COUNT] = {2, 4, 5, 12, 13, 14, 16, 17}; // customize

const int LEDC_FREQ = 5000;
const int LEDC_RESOLUTION = 8; // 8-bit

const uint32_t JOIN_HOLD_MS = 3000;
const uint32_t FACTORY_RESET_MS = 10000;

#define MAX_NODES 60
const uint8_t BROADCAST_MAC[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

uint32_t trigger_interval_ms = 500;
bool master_running = false;

Preferences prefs;

// ACK/retry tuning
const uint8_t ACK_RETRIES = 3;
const uint32_t ACK_RETRY_MS = 350;
const int MAX_PENDING_ACK = 16;

// Lock/key
#define LOCK_KEY_MAXLEN 32
char lock_key[LOCK_KEY_MAXLEN+1] = {0};
bool locked = false;
uint8_t derived_pmk[16];
uint8_t derived_lmk[16];

// IR (placeholder codes — press buttons to see raw codes in Serial and update)
const uint32_t IR_CODE_START_STOP = 0xFFFFFFFF; // toggle run/pause (placeholder)
const uint32_t IR_CODE_FREQ_UP    = 0xFFFFFFFF;
const uint32_t IR_CODE_FREQ_DOWN  = 0xFFFFFFFF;
const uint32_t IR_CODE_LOCK       = 0xFFFFFFFF; // press to broadcast lock key
// You will see raw codes printed in Serial; replace above constants with your remote's codes.

IRrecv irrecv(IR_PIN);
decode_results irresults;

// ---------------------------- Message defs --------------------------------
enum MsgType : uint8_t {
  MSG_JOIN_REQ = 1,
  MSG_ASSIGN = 2,
  MSG_SET_DOWN = 3,
  MSG_TRIGGER = 4,
  MSG_PING = 5,
  MSG_ACK = 6,
  MSG_LOCK = 7,
};

enum MsgFlags : uint8_t {
  FLAG_ACK_REQ = 0x01
};

// keep messages compact for ESP-NOW
struct MeshMsg {
  uint8_t type;        // MsgType
  uint8_t flags;       // flags bitfield
  uint8_t src_pos;     // sender pos (255=unassigned)
  uint8_t dest_pos;    // target pos (255=broadcast)
  uint16_t msg_id;     // unique ID for ACKs
  uint16_t seq;        // trigger sequence
  uint8_t hops;        // TTL for relaying
  uint8_t payload[12]; // payload small
} __attribute__((packed));

// --------------------------- Globals -------------------------------------
uint8_t myMac[6];
uint8_t downstreamMac[6];
int myPos = -1;       // -1 unassigned; master is 0
int ring_size = 1;
uint16_t trigger_seq = 0;
uint16_t next_msg_id = 1;

// Master peer list (persisted)
struct PeerInfo {
  uint8_t mac[6];
  int pos;
};
PeerInfo peers[MAX_NODES];
int peerCount = 0;

// pending ACK entries
struct PendingAck {
  bool used;
  uint16_t msg_id;
  uint8_t dest_mac[6];
  MeshMsg msg;
  uint8_t retries_left;
  uint32_t last_sent_ms;
};
PendingAck pending[MAX_PENDING_ACK];

// LED frames in PROGMEM
const uint8_t led_seq_frames[][LED_COUNT] PROGMEM = {
  {255, 0,   0,   0,   0,   0,   0,   0},
  {0,   255, 0,   0,   0,   0,   0,   0},
  {0,   0,   255, 0,   0,   0,   0,   0},
  {0,   0,   0,   255, 0,   0,   0,   0},
  {0,   0,   0,   0,   255, 0,   0,   0},
  {0,   0,   0,   0,   0,   255, 0,   0},
  {0,   0,   0,   0,   0,   0,   255, 0},
  {0,   0,   0,   0,   0,   0,   0,   255},
};
const uint8_t FRAMES = sizeof(led_seq_frames) / LED_COUNT;
const uint32_t FRAME_MS = 40;

// -------------------------- Helper functions ------------------------------
String macToStr(const uint8_t *mac) {
  char buf[18];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(buf);
}

bool macIsBroadcast(const uint8_t *mac) {
  for (int i=0;i<6;i++) if (mac[i] != 0xff) return false;
  return true;
}

uint16_t allocateMsgId() {
  uint16_t id = next_msg_id++;
  if (next_msg_id == 0) next_msg_id = 1;
  return id;
}

// derive PMK/LMK from lock string using SHA-256: first 16 bytes = PMK, next 16 = LMK
void derive_keys_from_lock(const char *key) {
  uint8_t hash[32];
  size_t klen = strnlen(key, LOCK_KEY_MAXLEN);
  mbedtls_sha256_ret((const unsigned char*)key, klen, hash, 0);
  memcpy(derived_pmk, hash, 16);
  memcpy(derived_lmk, hash + 16, 16);
}

// ----------------------- ESP-NOW peer management --------------------------
void ensurePeerKnown(const uint8_t *mac) {
  if (macIsBroadcast(mac)) return;

  // check exist
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = locked ? true : false;
#if defined(ESP_IDF_VERSION) || defined(ESP32) // try to copy LMK if available
  // attempt to copy lmk if the structure contains it (some Arduino builds expose lmk in esp_now_peer_info_t)
  memcpy(peerInfo.lmk, derived_lmk, 16);
#endif
  esp_err_t res = esp_now_add_peer(&peerInfo);
  if (res != ESP_OK) {
    Serial.printf("esp_now_add_peer failed for %s err=%d\n", macToStr(mac).c_str(), res);
  } else {
    Serial.printf("Added peer %s (encrypt=%d)\n", macToStr(mac).c_str(), peerInfo.encrypt ? 1 : 0);
  }
}

// ------------------------- Persistence -----------------------------------
void saveStateToPrefs() {
  prefs.putInt("pos", myPos);
  prefs.putInt("ringsz", ring_size);
  prefs.putBytes("down", downstreamMac, 6);
  prefs.putBool("locked", locked);
  prefs.putString("lockk", lock_key);
  Serial.println("Saved local node prefs");
}

void loadStateFromPrefs() {
  myPos = prefs.getInt("pos", -1);
  ring_size = prefs.getInt("ringsz", 1);
  size_t got = prefs.getBytes("down", downstreamMac, 6);
  if (got != 6) for (int i=0;i<6;i++) downstreamMac[i] = 0xff;
  locked = prefs.getBool("locked", false);
  String k = prefs.getString("lockk", "");
  strncpy(lock_key, k.c_str(), LOCK_KEY_MAXLEN);
  lock_key[LOCK_KEY_MAXLEN] = 0;
}

// Master peer list persistence
void saveMasterPeersToPrefs() {
  prefs.putInt("peerCount", peerCount);
  for (int i=0;i<peerCount;i++) {
    char key[16];
    snprintf(key, sizeof(key), "peer%02d", i);
    prefs.putBytes(key, peers[i].mac, 6);
  }
  Serial.printf("Master persisted %d peers\n", peerCount);
}

void loadMasterPeersFromPrefs() {
  int cnt = prefs.getInt("peerCount", -1);
  if (cnt <= 0) {
    // minimal: ensure master at pos0 later
    return;
  }
  peerCount = cnt;
  for (int i=0;i<peerCount;i++) {
    char key[16];
    snprintf(key, sizeof(key), "peer%02d", i);
    uint8_t mac[6];
    size_t got = prefs.getBytes(key, mac, 6);
    if (got == 6) {
      memcpy(peers[i].mac, mac, 6);
      peers[i].pos = i;
      Serial.printf("Loaded master peer pos %d -> %s\n", i, macToStr(mac).c_str());
    } else {
      for (int b=0;b<6;b++) peers[i].mac[b] = 0xff;
      peers[i].pos = i;
    }
  }
}

// add master peer entry
void addMasterPeer(const uint8_t *mac, int pos) {
  if (pos < 0 || pos >= MAX_NODES) return;
  memcpy(peers[pos].mac, mac, 6);
  peers[pos].pos = pos;
  if (pos >= peerCount) peerCount = pos + 1;
  Serial.printf("Master: peer added pos %d -> %s\n", pos, macToStr(mac).c_str());
}

// find pos by mac in master list
int findPosByMac(const uint8_t *mac) {
  for (int i=0;i<peerCount;i++) {
    if (memcmp(peers[i].mac, mac, 6) == 0) return peers[i].pos;
  }
  return -1;
}

// ----------------------- Pending ACK logic -------------------------------
void queuePendingAck(const uint8_t *dest_mac, MeshMsg &m) {
  for (int i=0;i<MAX_PENDING_ACK;i++) {
    if (!pending[i].used) {
      pending[i].used = true;
      pending[i].msg_id = m.msg_id;
      memcpy(pending[i].dest_mac, dest_mac, 6);
      memcpy(&pending[i].msg, &m, sizeof(MeshMsg));
      pending[i].retries_left = ACK_RETRIES;
      pending[i].last_sent_ms = millis();
      Serial.printf("Queued pending ACK slot %d msg_id %u -> %s\n", i, m.msg_id, macToStr(dest_mac).c_str());
      return;
    }
  }
  Serial.println("No pending ACK slot free!");
}

void clearPendingForMsgId(uint16_t msg_id) {
  for (int i=0;i<MAX_PENDING_ACK;i++) {
    if (pending[i].used && pending[i].msg_id == msg_id) {
      pending[i].used = false;
      Serial.printf("Cleared pending slot %d for msg_id %u\n", i, msg_id);
    }
  }
}

void processPendingAcks() {
  uint32_t now = millis();
  for (int i=0;i<MAX_PENDING_ACK;i++) {
    if (!pending[i].used) continue;
    if ((uint32_t)(now - pending[i].last_sent_ms) >= ACK_RETRY_MS) {
      if (pending[i].retries_left == 0) {
        Serial.printf("Pending msg_id %u failed after retries; clearing slot %d\n", pending[i].msg_id, i);
        pending[i].used = false;
      } else {
        esp_err_t res = esp_now_send(pending[i].dest_mac, (uint8_t *)&pending[i].msg, sizeof(MeshMsg));
        if (res != ESP_OK) {
          Serial.printf("Resend msg_id %u err=%d\n", pending[i].msg_id, res);
        } else {
          pending[i].last_sent_ms = now;
          pending[i].retries_left--;
          Serial.printf("Resent msg_id %u to %s, retries left %d\n", pending[i].msg_id, macToStr(pending[i].dest_mac).c_str(), pending[i].retries_left);
        }
      }
    }
  }
}

// --------------------------- Sending wrapper -----------------------------
void sendMeshMsg(const uint8_t *dest_mac, MeshMsg &m, bool requireAck) {
  if (!macIsBroadcast(dest_mac)) ensurePeerKnown(dest_mac);
  esp_err_t res = esp_now_send(dest_mac, (uint8_t *)&m, sizeof(MeshMsg));
  if (res != ESP_OK) {
    Serial.printf("esp_now_send err=%d to %s\n", res, macToStr(dest_mac).c_str());
  } else {
    if (requireAck) queuePendingAck(dest_mac, m);
  }
}

// --------------------------- LED sequence --------------------------------
void playLedSequence() {
  for (uint8_t f=0; f<FRAMES; f++) {
    for (uint8_t ch=0; ch<LED_COUNT; ch++) {
      uint8_t b = pgm_read_byte(&(led_seq_frames[f][ch]));
      ledcWrite(ch, b);
    }
    delay(FRAME_MS);
  }
  for (uint8_t ch=0; ch<LED_COUNT; ch++) ledcWrite(ch, 0);
}

// ------------------------- Message handlers ------------------------------
void handleAssignMsg(const MeshMsg &msg, const uint8_t *sender_mac) {
  // payload[0]=assigned pos, payload[1]=ring_size, payload[2..7]=downstream mac
  int assigned = msg.payload[0];
  int reported_ring = msg.payload[1];
  uint8_t down[6];
  memcpy(down, &msg.payload[2], 6);
  myPos = assigned;
  ring_size = reported_ring;
  memcpy(downstreamMac, down, 6);
  Serial.printf("Assigned pos=%d ring=%d downstream=%s\n", myPos, ring_size, macToStr(downstreamMac).c_str());
  saveStateToPrefs();
}

void handleSetDownMsg(const MeshMsg &msg) {
  uint8_t newDown[6];
  memcpy(newDown, msg.payload, 6);
  memcpy(downstreamMac, newDown, 6);
  Serial.printf("SET_DOWN updated downstream=%s\n", macToStr(downstreamMac).c_str());
  saveStateToPrefs();
}

void handleLockMsg(const MeshMsg &msg) {
  uint8_t klen = msg.payload[0];
  if (klen == 0 || klen > LOCK_KEY_MAXLEN) return;
  char rkey[LOCK_KEY_MAXLEN+1];
  memcpy(rkey, &msg.payload[1], klen);
  rkey[klen] = 0;
  Serial.printf("Received LOCK key: %s\n", rkey);
  strncpy(lock_key, rkey, LOCK_KEY_MAXLEN);
  lock_key[LOCK_KEY_MAXLEN] = 0;
  derive_keys_from_lock(lock_key);
  // install PMK locally
  esp_err_t rr = esp_now_set_pmk(derived_pmk);
  Serial.printf("esp_now_set_pmk -> %d\n", rr);
  locked = true;
  prefs.putBool("locked", locked);
  prefs.putString("lockk", lock_key);
  // Note: peers will be re-added with encrypt=true when ensurePeerKnown is called
}

void handleTriggerMsg(const MeshMsg &msg) {
  if (msg.hops == 0) return;
  // Triggers are honored regardless of locked state
  if ((int)msg.dest_pos == myPos) {
    Serial.printf("Trigger for me (pos %d) seq %u\n", myPos, msg.seq);
    playLedSequence();
    // forward to next pos
    uint8_t nextpos = (myPos + 1) % ring_size;
    MeshMsg next = msg;
    next.hops = msg.hops - 1;
    next.src_pos = myPos;
    next.dest_pos = nextpos;
    next.msg_id = allocateMsgId();
    // send to downstream
    ensurePeerKnown(downstreamMac);
    bool ackReq = (msg.flags & FLAG_ACK_REQ);
    if (ackReq) next.flags |= FLAG_ACK_REQ;
    sendMeshMsg(downstreamMac, next, ackReq);
    Serial.printf("Forwarded TRIGGER to downstream %s (pos %u) ackReq=%d\n", macToStr(downstreamMac).c_str(), nextpos, ackReq ? 1 : 0);
  } else {
    // ignore if not for us
    Serial.printf("TRIGGER not for me (dest %u) - ignored\n", msg.dest_pos);
  }
}

// ---------------------- Master join handling -----------------------------
void handleJoinRequestMaster(const uint8_t *requester_mac) {
  if (locked) {
    Serial.println("Master locked: ignoring JOIN_REQ");
    return;
  }
  int existing = findPosByMac(requester_mac);
  if (existing >= 0) {
    Serial.printf("Join requester already assigned pos %d; resending ASSIGN\n", existing);
    MeshMsg m;
    memset(&m,0,sizeof(m));
    m.type = MSG_ASSIGN;
    m.flags = 0;
    m.src_pos = 0;
    m.dest_pos = existing;
    m.msg_id = allocateMsgId();
    m.seq = 0;
    m.hops = 8;
    m.payload[0] = (uint8_t)existing;
    m.payload[1] = (uint8_t)ring_size;
    memcpy(&m.payload[2], myMac, 6);
    sendMeshMsg(requester_mac, m, true);
    return;
  }
  if (peerCount >= MAX_NODES) {
    Serial.println("Master: max nodes reached, ignoring join.");
    return;
  }
  int newpos = peerCount;
  addMasterPeer(requester_mac, newpos);
  ring_size = peerCount;
  Serial.printf("Master assigns pos %d to %s (ring=%d)\n", newpos, macToStr(requester_mac).c_str(), ring_size);
  ensurePeerKnown(requester_mac); // add peer

  // send ASSIGN
  MeshMsg assign;
  memset(&assign,0,sizeof(assign));
  assign.type = MSG_ASSIGN;
  assign.flags = 0;
  assign.src_pos = 0;
  assign.dest_pos = newpos;
  assign.msg_id = allocateMsgId();
  assign.seq = 0;
  assign.hops = 8;
  assign.payload[0] = (uint8_t)newpos;
  assign.payload[1] = (uint8_t)ring_size;
  memcpy(&assign.payload[2], myMac, 6); // default downstream for new node set to master for closing ring
  sendMeshMsg(requester_mac, assign, true);

  // instruct previous node to set its downstream to this new node
  if (newpos - 1 >= 0) {
    MeshMsg setdown;
    memset(&setdown,0,sizeof(setdown));
    setdown.type = MSG_SET_DOWN;
    setdown.flags = 0;
    setdown.src_pos = 0;
    setdown.dest_pos = newpos - 1;
    setdown.msg_id = allocateMsgId();
    setdown.seq = 0;
    setdown.hops = 8;
    memcpy(&setdown.payload[0], requester_mac, 6);
    sendMeshMsg(peers[newpos - 1].mac, setdown, true);
  }

  // persist master peers
  saveMasterPeersToPrefs();
}

// ------------------------- ESP-NOW callbacks -----------------------------
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Send status to %s: %s\n", macToStr(mac_addr).c_str(),
                status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len < (int)sizeof(MeshMsg)) {
    Serial.println("Received too-short packet");
    return;
  }
  MeshMsg msg;
  memcpy(&msg, incomingData, sizeof(MeshMsg));
  Serial.printf("Recv type %u from %s (srcpos %u destpos %u msgid %u seq %u hops %u flags 0x%02X)\n",
                msg.type, macToStr(mac).c_str(), msg.src_pos, msg.dest_pos, msg.msg_id, msg.seq, msg.hops, msg.flags);

  // If ACK -> clear pending
  if (msg.type == MSG_ACK) {
    clearPendingForMsgId(msg.msg_id);
    return;
  }

  // LOCK received
  if (msg.type == MSG_LOCK) {
    handleLockMsg(msg);
    // ack back if msg_id present
    if (msg.msg_id) {
      MeshMsg ack; memset(&ack,0,sizeof(ack));
      ack.type = MSG_ACK;
      ack.src_pos = myPos < 0 ? 255 : myPos;
      ack.dest_pos = msg.src_pos;
      ack.msg_id = msg.msg_id;
      ack.hops = 4;
      sendMeshMsg(mac, ack, false);
    }
    return;
  }

  // Master: handle JOIN_REQ from payload
  if (myPos == 0 && msg.type == MSG_JOIN_REQ) {
    uint8_t requester[6];
    memcpy(requester, msg.payload, 6);
    Serial.printf("Master: JOIN_REQ payload-mac %s (esp-now src %s)\n", macToStr(requester).c_str(), macToStr(mac).c_str());
    handleJoinRequestMaster(requester);
    return;
  }

  // Other message types
  switch (msg.type) {
    case MSG_ASSIGN:
      handleAssignMsg(msg, mac);
      // ACK
      if (msg.msg_id) {
        MeshMsg ack; memset(&ack,0,sizeof(ack));
        ack.type = MSG_ACK;
        ack.src_pos = myPos < 0 ? 255 : myPos;
        ack.dest_pos = msg.src_pos;
        ack.msg_id = msg.msg_id;
        ack.hops = 4;
        sendMeshMsg(mac, ack, false);
      }
      break;
    case MSG_SET_DOWN:
      handleSetDownMsg(msg);
      if (msg.msg_id) {
        MeshMsg ack; memset(&ack,0,sizeof(ack));
        ack.type = MSG_ACK;
        ack.src_pos = myPos < 0 ? 255 : myPos;
        ack.dest_pos = msg.src_pos;
        ack.msg_id = msg.msg_id;
        ack.hops = 4;
        sendMeshMsg(mac, ack, false);
      }
      break;
    case MSG_TRIGGER:
      handleTriggerMsg(msg);
      // reply ACK if requested
      if (msg.flags & FLAG_ACK_REQ) {
        MeshMsg ack; memset(&ack,0,sizeof(ack));
        ack.type = MSG_ACK;
        ack.src_pos = myPos < 0 ? 255 : myPos;
        ack.dest_pos = msg.src_pos;
        ack.msg_id = msg.msg_id;
        ack.hops = 4;
        sendMeshMsg(mac, ack, false);
      }
      break;
    case MSG_PING:
      Serial.println("PING received");
      break;
    default:
      Serial.printf("Unhandled message type %u\n", msg.type);
  }
}

// --------------------------- Setup & Loop --------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // pins
  pinMode(JOIN_BTN_PIN, INPUT_PULLUP);

  // LEDC init
  for (int i=0;i<LED_COUNT;i++){
    ledcSetup(i, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(LED_PINS[i], i);
    ledcWrite(i, 0);
  }

  prefs.begin("mesh", false);

  WiFi.mode(WIFI_STA);
  esp_read_mac(myMac, ESP_MAC_WIFI_STA);
  Serial.printf("My MAC: %s\n", macToStr(myMac).c_str());

  loadStateFromPrefs();

  // init esp-now
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    ESP.restart();
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // load master peers if this device is master
  if (myPos == 0) {
    // ensure self at pos0
    addMasterPeer(myMac, 0);
    loadMasterPeersFromPrefs();
    // re-add peers to esp-now (fix encryption state)
    if (locked && lock_key[0] != 0) derive_keys_from_lock(lock_key);
    for (int i=0;i<peerCount;i++) {
      ensurePeerKnown(peers[i].mac);
    }
    // IR only for master
    irrecv.enableIRIn();
    Serial.println("Master IR receiver enabled");
  } else {
    // ensure downstream default
    bool empty=false;
    for (int i=0;i<6;i++) if (downstreamMac[i] == 0xff) empty=true;
    if (empty) memcpy(downstreamMac, BROADCAST_MAC, 6);
    // if locked at boot, install PMK
    if (locked && lock_key[0] != 0) {
      derive_keys_from_lock(lock_key);
      esp_now_set_pmk(derived_pmk);
    }
  }

  // init pending
  for (int i=0;i<MAX_PENDING_ACK;i++) pending[i].used = false;

  Serial.println("Setup done");
  Serial.printf("Loaded: pos=%d ring=%d locked=%d downstream=%s peers=%d\n",
                myPos, ring_size, locked, macToStr(downstreamMac).c_str(), peerCount);
}

uint32_t lastTriggerMillis = 0;
uint32_t lastBtnPressMs = 0;
bool btn_pressed = false;
bool btn_event_sent = false;
uint32_t lastPendingProcess = 0;

void loop() {
  // button press detection
  int b = digitalRead(JOIN_BTN_PIN);
  if (b == LOW) {
    if (!btn_pressed) {
      btn_pressed = true;
      lastBtnPressMs = millis();
      btn_event_sent = false;
    } else {
      uint32_t held = millis() - lastBtnPressMs;
      if (held >= FACTORY_RESET_MS && !btn_event_sent) {
        btn_event_sent = true;
        Serial.println("Factory reset / unlock (10s press) -> clearing prefs and rebooting");
        prefs.clear();
        delay(200);
        ESP.restart();
      } else if (held >= JOIN_HOLD_MS && !btn_event_sent) {
        btn_event_sent = true;
        Serial.println("Join request (3s press) -> clear local assignment and broadcast JOIN_REQ");
        myPos = -1;
        ring_size = 1;
        memcpy(downstreamMac, BROADCAST_MAC, 6);
        saveStateToPrefs();
        MeshMsg m; memset(&m,0,sizeof(m));
        m.type = MSG_JOIN_REQ;
        m.flags = 0;
        m.src_pos = 255;
        m.dest_pos = 255;
        m.msg_id = allocateMsgId();
        m.seq = 0;
        m.hops = 8;
        memcpy(m.payload, myMac, 6);
        sendMeshMsg(BROADCAST_MAC, m, false);
      }
    }
  } else {
    btn_pressed = false;
  }

  // master IR and trigger scheduling
  if (myPos == 0) {
    if (irrecv.decode(&irresults)) {
      unsigned long code = irresults.value;
      Serial.printf("IR raw code: 0x%08lX\n", code);
      if (code == IR_CODE_START_STOP) {
        master_running = !master_running;
        Serial.printf("Master running: %d\n", master_running);
      } else if (code == IR_CODE_FREQ_UP) {
        trigger_interval_ms = max((uint32_t)50, trigger_interval_ms - 50);
        Serial.printf("Trigger interval: %u ms\n", trigger_interval_ms);
      } else if (code == IR_CODE_FREQ_DOWN) {
        trigger_interval_ms += 50;
        Serial.printf("Trigger interval: %u ms\n", trigger_interval_ms);
      } else if (code == IR_CODE_LOCK) {
        // build a short key and broadcast
        snprintf(lock_key, LOCK_KEY_MAXLEN+1, "LK%05u", (unsigned)millis()%100000);
        Serial.printf("Master broadcasting LOCK key: %s\n", lock_key);
        derive_keys_from_lock(lock_key);
        esp_err_t r = esp_now_set_pmk(derived_pmk);
        Serial.printf("esp_now_set_pmk -> %d\n", r);
        // re-add peers with encryption enabled (remove/add)
        for (int i=0;i<peerCount;i++) {
          if (!macIsBroadcast(peers[i].mac)) {
            if (esp_now_is_peer_exist(peers[i].mac)) esp_now_del_peer(peers[i].mac);
            esp_now_peer_info_t pi; memset(&pi,0,sizeof(pi));
            memcpy(pi.peer_addr, peers[i].mac, 6);
            pi.channel = 0;
            pi.encrypt = true;
#if defined(ESP_IDF_VERSION) || defined(ESP32)
            memcpy(pi.lmk, derived_lmk, 16);
#endif
            esp_err_t rr = esp_now_add_peer(&pi);
            Serial.printf("Re-added peer %s encrypt -> %d\n", macToStr(peers[i].mac).c_str(), rr);
          }
        }
        locked = true;
        prefs.putBool("locked", locked);
        prefs.putString("lockk", lock_key);
        // broadcast LOCK (request ACKs)
        MeshMsg m; memset(&m,0,sizeof(m));
        m.type = MSG_LOCK;
        m.flags = 0;
        m.src_pos = 0;
        m.dest_pos = 255;
        m.msg_id = allocateMsgId();
        m.seq = 0;
        m.hops = 8;
        uint8_t klen = min((size_t)LOCK_KEY_MAXLEN, strlen(lock_key));
        m.payload[0] = klen;
        memcpy(&m.payload[1], lock_key, klen);
        sendMeshMsg(BROADCAST_MAC, m, true);
      }
      irrecv.resume();
    }

    // master trigger timer
    if (master_running && (millis() - lastTriggerMillis >= trigger_interval_ms)) {
      lastTriggerMillis = millis();
      Serial.printf("Master trigger seq %u\n", trigger_seq);
      // master plays its own sequence (even when locked triggers continue)
      playLedSequence();

      if (ring_size > 1 && peerCount > 1) {
        MeshMsg m; memset(&m,0,sizeof(m));
        m.type = MSG_TRIGGER;
        m.flags = 0; // set FLAG_ACK_REQ here if you want reliable forward ACKs
        m.src_pos = 0;
        m.dest_pos = 1;
        m.msg_id = allocateMsgId();
        m.seq = trigger_seq++;
        m.hops = ring_size + 4;
        sendMeshMsg(peers[1].mac, m, false); // don't require ack by default
        Serial.printf("Master sent TRIGGER to pos1 %s\n", macToStr(peers[1].mac).c_str());
      }
    }
  }

  // pending ACK processing
  if (millis() - lastPendingProcess >= 100) {
    processPendingAcks();
    lastPendingProcess = millis();
  }

  delay(10);
}
