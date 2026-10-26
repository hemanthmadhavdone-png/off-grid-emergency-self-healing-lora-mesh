#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>
#include <string.h>

// =================================================================
// CONFIG
// =================================================================

// ---- Node identity ----
// UI handheld nodes use 1-4 (this matches MAX_CHAT_NODES and the
// fixed-size per-node chat threads / device-select list below).
// A headless relay built with RELAY_NODE_MODE should use an ID
// outside that range (e.g. 90, 91, ...) so it never collides with a
// handheld's identity and never shows up as a selectable device or
// chat thread.
#define NODE_ID 1       // change per device: 1,2,3,4 (or 90+ for a relay)

// ---- Relay node mode ----
// Builds this device as a headless mesh relay/repeater: no OLED, no
// rotary encoder, no menu/chat state. It only listens for packets and
// rebroadcasts them -- reusing the existing flood-forward logic in
// LoRaManager unchanged -- to extend coverage between handheld nodes
// standing too far apart to hear each other directly. This includes
// the new delivery/read ack packets below: they are forwarded by the
// exact same forward() path as DATA, so a relay needs zero new code
// to carry receipts across a multi-hop link.
//
// To build a relay: set this to 1, set NODE_ID above to something
// outside 1-4, and flash a board that only has the LoRa radio wired
// up (the OLED_*/ENC_* pins below can be left unconnected).
#define RELAY_NODE_MODE   0
#define RELAY_STATUS_LED  13   // onboard LED, blinked on each relayed packet; free pin, doesn't overlap LORA_*/OLED_*/ENC_* below

// ---- OLED (SSD1306 128x64, I2C) ----
#define OLED_SDA 21
#define OLED_SCL 22
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// ---- Rotary encoder (HW-040) ----
#define ENC_CLK 27
#define ENC_DT  26
#define ENC_SW  25

// ---- LoRa (SX1278) ----
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_NSS  5
#define LORA_RST  14
#define LORA_DIO0 2
#define LORA_FREQUENCY 433E6

// ---- LoRa radio parameters --------------------------------------
// CRITICAL: these must be IDENTICAL on every node in the mesh,
// including any relay nodes.
// If even one node relies on library defaults while others set
// these explicitly (or a different library version changes its
// defaults), nodes can fail to hear each other completely even
// standing right next to one another -- this looks exactly like
// "no connection even at ideal distance."
#define LORA_SPREADING_FACTOR   9        // 6-12, higher = longer range, slower
#define LORA_SIGNAL_BANDWIDTH   125E3    // Hz
#define LORA_CODING_RATE        5        // 4/5
#define LORA_SYNC_WORD          0x12     // private network sync word, must match on ALL nodes
#define LORA_TX_POWER_DBM       20       // max for PA_BOOST pin on most SX1278 modules

// ---- Timing ----
#define LONG_PRESS_MS      600
#define BTN_DEBOUNCE_MS     30
#define SEND_ANIM_MS       700
#define SENT_CONFIRM_MS    900
#define RECEIVING_TIMEOUT_MS 5000
#define HIGHLIGHT_ANIM_MS   120

// ---- Message queue / node status timing ----
#define MSG_QUEUE_CAPACITY   8      // circular buffer size, see MessageQueue below
#define MSG_DWELL_MS        4000    // each queued message shown 3-5s (mid value 4s)
#define NODE_OFFLINE_TIMEOUT_MS 15000   // no heartbeat/data in this window -> offline
#define RECONNECT_POPUP_MS  2500
#define OFFLINE_WARN_MS     2000

// ---- Relay status LED timing ----
#define RELAY_LED_ON_MS      80    // how long the status LED stays lit per relayed packet

// ---- Relay path tracking ---------------------------------------
// Max number of intermediate relay hops recorded in a packet's path[].
// Keep in sync with common sense re: pkt.ttl (default 5) -- a path
// longer than the TTL could ever allow is impossible anyway.
#define MAX_RELAY_HOPS 6

// =================================================================
// PACKET
//
// refSeq / epoch additions (read-receipt + clock feature):
//  - refSeq is only meaningful on PKT_ACK_DELIVERED / PKT_ACK_READ:
//    it names the seq of the original DATA packet being acked. DATA
//    and HELLO leave it at 0.
//  - epoch carries the sender's wall-clock time (seconds since the
//    user last set the clock in Settings) at the moment this packet
//    was created. On DATA it's "when this message was sent". On an
//    ACK it's "when the delivery/read event happened, on the acking
//    node's own clock". 0 means "that node's clock wasn't set yet" --
//    every screen that shows a time falls back to a relative "Xs ago"
//    display in that case, so an unset clock never shows a bogus time.
//
// hopCount / path additions (relay-visibility feature):
//  - path[] records the node ID of every relay that has forwarded
//    THIS packet so far, in order, oldest hop first. hopCount is how
//    many of those slots are filled.
//  - Only PKT_DATA accumulates real hops as it's forwarded (see
//    LoRaManager::forward()). When the destination auto-acks a DATA
//    packet with PKT_ACK_DELIVERED, it copies that DATA packet's
//    hopCount/path into the ack UNCHANGED -- the ack's own return trip
//    across the mesh does not get appended on top. That's what lets
//    the original sender learn "my message went through node X, then
//    node Y" once the delivered ack makes it back, regardless of how
//    many hops the ack itself needed to get there.
//  - This doubles as a loop guard: forward() refuses to add a node to
//    a path it's already in (see forward()), on top of the existing
//    alreadySeen()/rememberSeen() dedup cache.
// =================================================================

#define PKT_DATA          0
#define PKT_HELLO         1
#define PKT_ACK_DELIVERED 2   // auto-sent by the radio layer the instant a DATA packet addressed to us arrives
#define PKT_ACK_READ      3   // sent by the UI layer only once a human actually opens the message/thread
#define BROADCAST_ID  0xFF

struct Packet
{
  uint8_t  type;
  uint8_t  source;
  uint8_t  destination;
  uint16_t seq;
  uint8_t  ttl;
  uint16_t refSeq;   // see comment above
  uint32_t epoch;    // see comment above
  uint8_t  hopCount;              // see "hopCount / path additions" above
  uint8_t  path[MAX_RELAY_HOPS];  // relay node IDs, in order; only path[0..hopCount-1] are meaningful
  char     payload[32];
};

// Delivery/read status of an outgoing message, as tracked locally.
// Ordinal order matters: never-regress logic below compares these
// with '>' so a message can only move SENT -> DELIVERED -> READ,
// never backward, even if acks arrive out of order over the mesh.
enum MsgStatus : uint8_t { MSG_SENT = 0, MSG_DELIVERED = 1, MSG_READ = 2 };

// =================================================================
// CLOCK (software wall clock -- no RTC chip in this build)
//
// There's no RTC hardware in the parts list, so this is a millis()
// based clock: the user sets "now" once from Settings > Set Clock,
// and nowEpoch() extrapolates forward from that from then on. This
// resets to "unset" on every reboot/power loss -- every place that
// displays a time checks isSet()/nonzero-epoch and falls back to a
// relative "Xs ago" display instead of showing a stale/bogus time.
//
// If you want the clock to survive power cycles, wire a DS3231 RTC
// (I2C, can share OLED_SDA/OLED_SCL) and replace nowEpoch()'s body
// with a read from the RTC chip instead of the millis() extrapolation
// below -- nothing else in this file needs to change.
// =================================================================

class Clock
{
public:
  void begin() { _epochAtSet = 0; _setAtMillis = millis(); _isSet = false; }
  void setEpoch(uint32_t epochSeconds) { _epochAtSet = epochSeconds; _setAtMillis = millis(); _isSet = true; }
  bool isSet() const { return _isSet; }
  uint32_t nowEpoch() const
  {
    if (!_isSet) return 0;
    return _epochAtSet + (millis() - _setAtMillis) / 1000UL;
  }
private:
  uint32_t _epochAtSet = 0;
  unsigned long _setAtMillis = 0;
  bool _isSet = false;
};

// Formats epochSeconds (seconds-of-day, UTC/local-as-set, no timezone
// math since there's no networked time source to derive one from) as
// "HH:MM:SS". Used everywhere a message timestamp is displayed.
static void formatClock(uint32_t epochSeconds, char* out, size_t outLen)
{
  uint32_t secOfDay = epochSeconds % 86400UL;
  uint32_t h = secOfDay / 3600UL;
  uint32_t m = (secOfDay % 3600UL) / 60UL;
  uint32_t s = secOfDay % 60UL;
  snprintf(out, outLen, "%02lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)s);
}

// =================================================================
// ROTARY (encoder + button, interrupt-driven)
// =================================================================

enum ButtonEvent { BTN_NONE, BTN_CLICK, BTN_LONG_PRESS };

class Rotary
{
public:
  void begin(uint8_t clkPin, uint8_t dtPin, uint8_t swPin);
  int readDelta();
  ButtonEvent pollButton();

private:
  static Rotary* instance;
  static void IRAM_ATTR isrTrampoline();
  void IRAM_ATTR handleInterrupt();

  uint8_t _clk, _dt, _sw;
  volatile int _delta = 0;
  volatile uint8_t _lastClk = HIGH;

  bool _btnDown = false;
  unsigned long _pressStart = 0;
};

Rotary* Rotary::instance = nullptr;

void Rotary::begin(uint8_t clkPin, uint8_t dtPin, uint8_t swPin)
{
  _clk = clkPin;
  _dt  = dtPin;
  _sw  = swPin;

  pinMode(_clk, INPUT_PULLUP);
  pinMode(_dt,  INPUT_PULLUP);
  pinMode(_sw,  INPUT_PULLUP);

  _lastClk = digitalRead(_clk);

  instance = this;
  attachInterrupt(digitalPinToInterrupt(_clk), isrTrampoline, CHANGE);
}

void IRAM_ATTR Rotary::isrTrampoline()
{
  if (instance) instance->handleInterrupt();
}

void IRAM_ATTR Rotary::handleInterrupt()
{
  uint8_t clkState = digitalRead(_clk);
  if (clkState != _lastClk) {
    if (digitalRead(_dt) != clkState) _delta++;
    else _delta--;
  }
  _lastClk = clkState;
}

int Rotary::readDelta()
{
  noInterrupts();
  int d = _delta;
  _delta = 0;
  interrupts();
  return d;
}

ButtonEvent Rotary::pollButton()
{
  bool pressed = (digitalRead(_sw) == LOW);
  ButtonEvent event = BTN_NONE;

  if (pressed && !_btnDown) {
    _btnDown = true;
    _pressStart = millis();
  } else if (!pressed && _btnDown) {
    _btnDown = false;
    unsigned long dur = millis() - _pressStart;
    if (dur > BTN_DEBOUNCE_MS) {
      event = (dur >= LONG_PRESS_MS) ? BTN_LONG_PRESS : BTN_CLICK;
    }
  }
  return event;
}

// =================================================================
// LORA MANAGER (send/receive/forward/neighbor table/acks)
//
// FIXES applied in earlier versions (kept, see comments in begin()):
//  - Explicit spreading factor / bandwidth / coding rate / sync word
//    / TX power set identically instead of relying on library
//    defaults.
//  - CRC enabled so corrupted packets are dropped by the radio.
//
// READ-RECEIPT / TIMESTAMP NOTES:
//  - PKT_ACK_DELIVERED is fired automatically by poll() the instant a
//    DATA packet addressed to this node arrives -- this is a radio-
//    layer concern, not a UI concern, so it lives here.
//  - PKT_ACK_READ is NOT fired automatically. The UI layer (Menu)
//    calls sendAck(..., PKT_ACK_READ, ...) only once a human actually
//    opens the message. That keeps "read" meaning what it should.
//  - Both ack types are dedup'd (alreadySeen/rememberSeen) and
//    forwarded via the exact same forward() used for DATA, so they
//    get carried across multiple relay hops for free and can't loop.
//  - Acks are addressed unicast (never broadcast): once an ack
//    reaches its destination it stops there and is delivered to
//    _onAckReceived, it is never re-forwarded past that point.
//  - Broadcast DATA packets never trigger an auto delivery ack --
//    there's no single "the" recipient for a broadcast, and having
//    every listening node ack it back would flood the channel.
// =================================================================

typedef void (*PacketReceivedCallback)(const Packet& pkt, int rssi);
typedef void (*NodeStatusCallback)(uint8_t nodeId, bool online);
// hopCount/path: the relay path the original DATA packet took, frozen
// into this ack (0/nullptr-equivalent when the ack didn't carry one --
// see sendAck()'s default args and the PACKET comment above).
typedef void (*AckReceivedCallback)(uint8_t fromNode, uint16_t refSeq, uint8_t ackType, uint32_t epoch, uint8_t hopCount, const uint8_t* path);
typedef uint32_t (*EpochProviderCallback)();

class LoRaManager
{
public:
  bool begin(uint8_t nodeId, uint8_t nss, uint8_t rst, uint8_t dio0, long frequency);
  uint16_t send(uint8_t destination, const char* text, uint32_t epoch);
  // hopCount/path let the caller freeze a DATA packet's already-recorded
  // relay path into this ack (see PACKET comment + forward()). Defaults
  // to "no path" for acks that don't have one to report (e.g. the
  // UI-triggered PKT_ACK_READ in Menu::markMessageRead()).
  void sendAck(uint8_t destination, uint8_t ackType, uint16_t refSeq, uint32_t epoch, uint8_t hopCount = 0, const uint8_t* path = nullptr);
  void sendHello();
  void poll();
  void setOnReceive(PacketReceivedCallback cb) { _onReceive = cb; }
  void setOnNodeStatusChange(NodeStatusCallback cb) { _onStatusChange = cb; }
  void setOnAckReceived(AckReceivedCallback cb) { _onAckReceived = cb; }
  void setEpochProvider(EpochProviderCallback cb) { _epochProvider = cb; }

  bool isNeighborActive(uint8_t nodeId);
  int  neighborRssi(uint8_t nodeId);
  void pruneNeighbors();
  int  activeNeighborCount();

  // Returns true once (and clears itself) the first time it's called
  // after this node has relayed a packet on behalf of another node.
  // Intended for a headless relay build to drive a status LED.
  bool consumeRelayActivityFlag();

private:
  uint8_t _nodeId;
  uint16_t _seq = 0;

  bool alreadySeen(uint8_t src, uint16_t seq);
  void rememberSeen(uint8_t src, uint16_t seq);
  void forward(Packet& pkt);
  void updateNeighbor(uint8_t nodeId, int rssi);

  struct CacheEntry { uint8_t src; uint16_t seq; };
  CacheEntry _cache[20];
  int _cacheIdx = 0;

  // known  = this slot has been assigned to a nodeId at least once
  // active = heard from that node within the timeout window (i.e. "online")
  struct Neighbor { uint8_t nodeId; unsigned long lastSeen; int rssi; bool known; bool active; };
  static const int MAX_NEIGHBORS = 10;
  Neighbor _neighbors[MAX_NEIGHBORS];
  int findActiveNeighbor(uint8_t nodeId);

  PacketReceivedCallback _onReceive = nullptr;
  NodeStatusCallback _onStatusChange = nullptr;
  AckReceivedCallback _onAckReceived = nullptr;
  EpochProviderCallback _epochProvider = nullptr;

  bool _relayActivity = false;
};

bool LoRaManager::begin(uint8_t nodeId, uint8_t nss, uint8_t rst, uint8_t dio0, long frequency)
{
  _nodeId = nodeId;
  for (int i = 0; i < MAX_NEIGHBORS; i++) { _neighbors[i].known = false; _neighbors[i].active = false; }

  LoRa.setPins(nss, rst, dio0);
  if (!LoRa.begin(frequency)) return false;

  // --- FIX: explicit radio config, identical on every node -------
  // Do NOT let any node rely on library defaults. A board/version
  // mismatch here silently breaks node-to-node reception even at
  // point-blank range, which matches the "ideal distance but no
  // connection" symptom exactly.
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER_DBM, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.enableCrc();   // drop corrupted packets instead of misreading them
  // -----------------------------------------------------------------

  return true;
}

bool LoRaManager::alreadySeen(uint8_t src, uint16_t seq)
{
  for (int i = 0; i < 20; i++)
    if (_cache[i].src == src && _cache[i].seq == seq) return true;
  return false;
}

void LoRaManager::rememberSeen(uint8_t src, uint16_t seq)
{
  _cache[_cacheIdx].src = src;
  _cache[_cacheIdx].seq = seq;
  _cacheIdx = (_cacheIdx + 1) % 20;
}

uint16_t LoRaManager::send(uint8_t destination, const char* text, uint32_t epoch)
{
  Packet pkt;
  pkt.type = PKT_DATA;
  pkt.source = _nodeId;
  pkt.destination = destination;
  pkt.seq = _seq++;
  pkt.ttl = 5;
  pkt.refSeq = 0;
  pkt.epoch = epoch;
  pkt.hopCount = 0;   // no relay has touched this packet yet -- forward() grows this as it travels
  memset(pkt.path, 0, sizeof(pkt.path));
  memset(pkt.payload, 0, sizeof(pkt.payload));
  strncpy(pkt.payload, text, sizeof(pkt.payload) - 1);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  rememberSeen(pkt.source, pkt.seq);
  return pkt.seq;
}

void LoRaManager::sendAck(uint8_t destination, uint8_t ackType, uint16_t refSeq, uint32_t epoch, uint8_t hopCount, const uint8_t* path)
{
  Packet pkt;
  pkt.type = ackType;
  pkt.source = _nodeId;
  pkt.destination = destination;
  pkt.seq = _seq++;
  pkt.ttl = 5;
  pkt.refSeq = refSeq;
  pkt.epoch = epoch;

  // Freeze a copy of the original DATA packet's relay path into this
  // ack, unchanged -- this ack's own trip back across the mesh does
  // NOT get appended on top (forward() only grows path[] for PKT_DATA).
  // That's what lets the original sender see the path its *message*
  // took, not the path the acknowledgement happened to take back.
  pkt.hopCount = (hopCount > MAX_RELAY_HOPS) ? MAX_RELAY_HOPS : hopCount;
  memset(pkt.path, 0, sizeof(pkt.path));
  if (path && pkt.hopCount > 0) memcpy(pkt.path, path, pkt.hopCount);

  memset(pkt.payload, 0, sizeof(pkt.payload));   // acks carry no text

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  rememberSeen(pkt.source, pkt.seq);
}

void LoRaManager::sendHello()
{
  Packet pkt;
  pkt.type = PKT_HELLO;
  pkt.source = _nodeId;
  pkt.destination = BROADCAST_ID;
  pkt.seq = _seq++;
  pkt.ttl = 1;
  pkt.refSeq = 0;
  pkt.epoch = 0;
  pkt.hopCount = 0;
  memset(pkt.path, 0, sizeof(pkt.path));
  memset(pkt.payload, 0, sizeof(pkt.payload));

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();
}

void LoRaManager::forward(Packet& pkt)
{
  pkt.ttl--;
  if (pkt.ttl == 0) return;

  // --- Relay Process Visibility: path tracking + loop guard ---------
  // Only PKT_DATA accumulates hops (an ack's path[] is a frozen copy
  // set once in sendAck(), not something forward() should touch).
  // Loop guard: refuse to add ourselves to a path we're already in
  // (belt-and-suspenders on top of alreadySeen()/rememberSeen(), which
  // already stops this node from re-forwarding a packet it has already
  // handled), and refuse to grow path[] past its fixed capacity.
  if (pkt.type == PKT_DATA) {
    for (int i = 0; i < pkt.hopCount; i++) {
      if (pkt.path[i] == _nodeId) return;   // would be a loop -- drop instead of forwarding
    }
    if (pkt.hopCount < MAX_RELAY_HOPS) {
      pkt.path[pkt.hopCount++] = _nodeId;
    }
  }

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  _relayActivity = true;   // surfaced via consumeRelayActivityFlag() for a headless relay's status LED
}

bool LoRaManager::consumeRelayActivityFlag()
{
  bool v = _relayActivity;
  _relayActivity = false;
  return v;
}

int LoRaManager::findActiveNeighbor(uint8_t nodeId)
{
  for (int i = 0; i < MAX_NEIGHBORS; i++)
    if (_neighbors[i].known && _neighbors[i].active && _neighbors[i].nodeId == nodeId) return i;
  return -1;
}

// Any inbound packet (HELLO heartbeat, DATA, or an ack) is proof of
// life. This finds-or-creates the node's slot, then fires the status
// callback only on the offline->online edge (wasActive was false).
void LoRaManager::updateNeighbor(uint8_t nodeId, int rssi)
{
  int idx = -1;
  for (int i = 0; i < MAX_NEIGHBORS; i++)
    if (_neighbors[i].known && _neighbors[i].nodeId == nodeId) { idx = i; break; }

  if (idx == -1) {
    for (int i = 0; i < MAX_NEIGHBORS; i++)
      if (!_neighbors[i].known) { idx = i; break; }
    if (idx == -1) return; // neighbor table full, drop silently
    _neighbors[idx].nodeId = nodeId;
    _neighbors[idx].known = true;
    _neighbors[idx].active = false;
  }

  bool wasActive = _neighbors[idx].active;
  _neighbors[idx].lastSeen = millis();
  _neighbors[idx].rssi = rssi;
  _neighbors[idx].active = true;

  if (!wasActive && _onStatusChange) _onStatusChange(nodeId, true);
}

// Called roughly once a second from loop(). Marks any node quiet for
// longer than NODE_OFFLINE_TIMEOUT_MS as offline and fires the edge
// callback exactly once for that transition.
void LoRaManager::pruneNeighbors()
{
  unsigned long now = millis();
  for (int i = 0; i < MAX_NEIGHBORS; i++) {
    if (_neighbors[i].known && _neighbors[i].active &&
        (now - _neighbors[i].lastSeen > NODE_OFFLINE_TIMEOUT_MS)) {
      _neighbors[i].active = false;
      if (_onStatusChange) _onStatusChange(_neighbors[i].nodeId, false);
    }
  }
}

bool LoRaManager::isNeighborActive(uint8_t nodeId) { return findActiveNeighbor(nodeId) != -1; }

int LoRaManager::neighborRssi(uint8_t nodeId)
{
  int idx = findActiveNeighbor(nodeId);
  return (idx == -1) ? 0 : _neighbors[idx].rssi;
}

int LoRaManager::activeNeighborCount()
{
  int count = 0;
  for (int i = 0; i < MAX_NEIGHBORS; i++)
    if (_neighbors[i].known && _neighbors[i].active) count++;
  return count;
}

void LoRaManager::poll()
{
  int packetSize = LoRa.parsePacket();
  if (packetSize != sizeof(Packet)) return;

  Packet pkt;
  LoRa.readBytes((uint8_t*)&pkt, sizeof(pkt));
  int rssi = LoRa.packetRssi();

  updateNeighbor(pkt.source, rssi);   // heartbeat/liveness, regardless of packet type

  if (pkt.type == PKT_HELLO) return;

  // Acks are handled as their own small unicast flow: dedupe/forward
  // exactly like DATA (so they survive multiple relay hops and can't
  // loop), but once one reaches its destination it's delivered to
  // the app layer and stops -- it never needs a further ack.
  if (pkt.type == PKT_ACK_DELIVERED || pkt.type == PKT_ACK_READ) {
    if (alreadySeen(pkt.source, pkt.seq)) return;
    rememberSeen(pkt.source, pkt.seq);

    if (pkt.destination == _nodeId) {
      // pkt.hopCount/pkt.path here are the DATA packet's frozen relay
      // path, carried unchanged inside this ack -- see sendAck().
      if (_onAckReceived) _onAckReceived(pkt.source, pkt.refSeq, pkt.type, pkt.epoch, pkt.hopCount, pkt.path);
      return;
    }
    forward(pkt);
    return;
  }

  if (alreadySeen(pkt.source, pkt.seq)) return;
  rememberSeen(pkt.source, pkt.seq);

  if (pkt.destination == _nodeId || pkt.destination == BROADCAST_ID) {
    if (_onReceive) _onReceive(pkt, rssi);
    if (pkt.destination == _nodeId) {
      // Auto-ack "delivered" the instant a directly-addressed DATA
      // packet reaches us. Never done for broadcasts (no single
      // intended recipient) and never for non-DATA types. Embeds this
      // DATA packet's own hopCount/path so the original sender learns
      // the relay path its message took (Relay Process Visibility).
      if (pkt.type == PKT_DATA) {
        uint32_t now = _epochProvider ? _epochProvider() : 0;
        sendAck(pkt.source, PKT_ACK_DELIVERED, pkt.seq, now, pkt.hopCount, pkt.path);
      }
      return;
    }
  }

  forward(pkt);
}

// =================================================================
// MESSAGE DISPLAY QUEUE
//
// Design choice: fixed-size circular buffer (FIFO), not a dynamic
// list.
//   - O(1) push/pop with zero heap allocation -- safe to call from
//     the main poll loop on an MCU with a small, fixed RAM budget.
//   - FIFO ordering matches the requirement: messages are shown in
//     the order they arrived, each getting its dwell time on screen.
//   - Overflow policy: when full, push() drops the OLDEST unread
//     message (advances the tail) rather than rejecting the newest.
//     In an emergency-comms context the newest traffic is usually the
//     most operationally relevant, so we never block or lose the most
//     recent alert; the trade-off is that a very old, still-unread
//     message can be silently dropped under sustained heavy traffic.
//     (Every message is still logged permanently in Chat's inbox /
//     per-node thread -- this queue only governs the *live* on-screen
//     rotation, so nothing is lost from history.)
// =================================================================

struct DisplayMessage
{
  uint8_t nodeId;
  char    text[32];
  int     rssi;
  unsigned long receivedAt;
  uint16_t seq;          // needed so dismissing this popup can send a proper read-ack
  uint32_t senderEpoch;  // sender's wall-clock send time, 0 if sender's clock wasn't set
};

class MessageQueue
{
public:
  bool push(uint8_t nodeId, const char* text, int rssi, uint16_t seq, uint32_t senderEpoch);
  bool pop(DisplayMessage& out);
  int  count() const;
  bool isEmpty() const;

private:
  DisplayMessage _buf[MSG_QUEUE_CAPACITY];
  int _head = 0;   // next write slot
  int _tail = 0;   // next read slot
  int _count = 0;
};

bool MessageQueue::push(uint8_t nodeId, const char* text, int rssi, uint16_t seq, uint32_t senderEpoch)
{
  DisplayMessage& slot = _buf[_head];
  slot.nodeId = nodeId;
  strncpy(slot.text, text, 31); slot.text[31] = '\0';
  slot.rssi = rssi;
  slot.receivedAt = millis();
  slot.seq = seq;
  slot.senderEpoch = senderEpoch;

  if (_count == MSG_QUEUE_CAPACITY) {
    _tail = (_tail + 1) % MSG_QUEUE_CAPACITY;   // drop oldest, see note above
  } else {
    _count++;
  }
  _head = (_head + 1) % MSG_QUEUE_CAPACITY;
  return true;
}

bool MessageQueue::pop(DisplayMessage& out)
{
  if (_count == 0) return false;
  out = _buf[_tail];
  _tail = (_tail + 1) % MSG_QUEUE_CAPACITY;
  _count--;
  return true;
}

int  MessageQueue::count() const  { return _count; }
bool MessageQueue::isEmpty() const { return _count == 0; }

// =================================================================
// CHAT (inbox log + per-node threads)
//
// ChatMessage now tracks delivery/read status + three timestamps
// (sent / delivered / read) per outgoing message. markOutgoingStatus()
// is the only way status moves forward, and it refuses to move
// backward (see MsgStatus comment above) so a late/duplicate ack
// can't undo a more recent one.
// =================================================================

#define MAX_INBOX 20
#define MAX_CHAT_NODES 4
#define MAX_MSGS_PER_CHAT 10
#define MAX_OUTBOX 20        // transmitter-side sent log (unicast + broadcast), mirrors MAX_INBOX
#define MAX_BROADCAST_LOG 16 // shared "broadcast channel" thread: our own broadcasts + everyone else's

struct InboxEntry
{
  uint8_t nodeId;
  char    text[32];
  unsigned long timestamp;  // millis() at local receipt -- relative-time fallback
  int     rssi;
  uint16_t seq;
  uint32_t senderEpoch;     // sender's wall-clock send time embedded in the packet; 0 = unknown
  uint32_t localEpoch;      // our own wall-clock time at the moment we received it; 0 = our clock wasn't set
};

// One row of the transmitter's own "Outbox": every message *this*
// node has sent, unicast or broadcast, with its live delivery status.
// Pushed the instant send() is called (status = MSG_SENT) so the
// sender always has immediate local confirmation the send happened,
// then updated in place as delivered/read acks arrive -- exactly the
// same never-regress rule as ChatMessage.status (see MsgStatus above).
// Broadcast rows stay at MSG_SENT forever: there's no single "the"
// recipient to ack a broadcast, so delivered/read never apply to it.
struct OutboxEntry
{
  uint8_t  destination;     // node id, or BROADCAST_ID
  char     text[32];
  unsigned long timestamp;  // millis() fallback
  uint16_t seq;
  uint32_t sentEpoch;
  uint32_t deliveredEpoch;  // 0 until a delivered ack arrives (unicast only)
  uint32_t readEpoch;       // 0 until a read ack arrives (unicast only)
  MsgStatus status;
  // --- Relay Process Visibility ---
  bool     expectsRelay;    // true if destination wasn't a direct active neighbor at send time
  uint8_t  hopCount;        // filled in once the delivered ack reports the actual relay path; 0 = not known yet
  uint8_t  path[MAX_RELAY_HOPS];
};

// One row of the shared Broadcast Channel view: every broadcast this
// node has sent *or* received, in one combined timeline -- like a
// group thread. `outgoing` distinguishes "You" rows from a
// neighbor's; `peerNode` is only meaningful when !outgoing.
struct BroadcastLogEntry
{
  bool     outgoing;
  uint8_t  peerNode;        // sender's node id, for incoming rows
  char     text[32];
  unsigned long timestamp;
  uint16_t seq;
  uint32_t epoch;           // sender's wall-clock send time, 0 = unknown
};

struct ChatMessage
{
  bool     outgoing;
  char     text[32];
  unsigned long timestamp;  // millis() fallback, same role as before
  uint16_t seq;
  uint32_t sentEpoch;       // outgoing: our clock when sent. incoming: origin's embedded send time. 0 = unknown
  uint32_t deliveredEpoch;  // outgoing only: acker's clock when PKT_ACK_DELIVERED arrived, 0 until then
  uint32_t readEpoch;       // outgoing only: acker's clock when PKT_ACK_READ arrived, 0 until then
  MsgStatus status;         // outgoing only; meaningless for incoming rows
  // --- Relay Process Visibility (outgoing only, same idea as OutboxEntry) ---
  uint8_t  hopCount;
  uint8_t  path[MAX_RELAY_HOPS];
};

class Chat
{
public:
  void begin();
  void addInboxEntry(uint8_t nodeId, const char* text, int rssi, uint16_t seq, uint32_t senderEpoch, uint32_t localEpoch);
  int  inboxCount();
  const InboxEntry& inboxEntry(int indexFromNewest);
  void addChatMessage(uint8_t nodeId, bool outgoing, const char* text, uint16_t seq, uint32_t sentEpoch);
  int  chatMessageCount(uint8_t nodeId);
  const ChatMessage& chatMessage(uint8_t nodeId, int indexFromOldest);
  // hopCount/path: only meaningfully non-zero on the ack that actually
  // carries the relay path (see LoRaManager::sendAck()); pass 0/nullptr
  // for acks that don't have one (e.g. a plain PKT_ACK_READ) so an
  // already-known path from an earlier delivered ack isn't clobbered.
  void markOutgoingStatus(uint8_t peerNode, uint16_t refSeq, MsgStatus newStatus, uint32_t epoch, uint8_t hopCount = 0, const uint8_t* path = nullptr);
  bool nodeHasHistory(uint8_t nodeId);
  void clearAll();

  // --- Transmitter Outbox (feature: immediate sent/delivered/read visibility) ---
  void addOutboxEntry(uint8_t destination, const char* text, uint16_t seq, uint32_t sentEpoch, bool expectsRelay);
  int  outboxCount();
  const OutboxEntry& outboxEntry(int indexFromNewest);
  void markOutboxStatus(uint8_t destination, uint16_t refSeq, MsgStatus newStatus, uint32_t epoch, uint8_t hopCount = 0, const uint8_t* path = nullptr);

  // --- Broadcast Channel (feature: shared all-nodes thread) ---
  void addBroadcastSent(const char* text, uint16_t seq, uint32_t epoch);
  void addBroadcastReceived(uint8_t peerNode, const char* text, uint16_t seq, uint32_t epoch);
  int  broadcastLogCount();
  const BroadcastLogEntry& broadcastLogEntry(int indexFromOldest);

private:
  ChatMessage& slotMut(uint8_t nodeId, int indexFromOldest);

  InboxEntry _inbox[MAX_INBOX];
  int _inboxCount = 0;
  int _inboxHead = 0;

  ChatMessage _threads[MAX_CHAT_NODES][MAX_MSGS_PER_CHAT];
  int _threadCount[MAX_CHAT_NODES] = {0, 0, 0, 0};
  int _threadHead[MAX_CHAT_NODES] = {0, 0, 0, 0};

  OutboxEntry _outbox[MAX_OUTBOX];
  int _outboxCount = 0;
  int _outboxHead = 0;

  BroadcastLogEntry _broadcastLog[MAX_BROADCAST_LOG];
  int _broadcastCount = 0;
  int _broadcastHead = 0;
};

void Chat::begin() { clearAll(); }

void Chat::clearAll()
{
  _inboxCount = 0;
  _inboxHead = 0;
  for (int n = 0; n < MAX_CHAT_NODES; n++) {
    _threadCount[n] = 0;
    _threadHead[n] = 0;
  }
  _outboxCount = 0;
  _outboxHead = 0;
  _broadcastCount = 0;
  _broadcastHead = 0;
}

void Chat::addInboxEntry(uint8_t nodeId, const char* text, int rssi, uint16_t seq, uint32_t senderEpoch, uint32_t localEpoch)
{
  InboxEntry &e = _inbox[_inboxHead];
  e.nodeId = nodeId;
  strncpy(e.text, text, 31); e.text[31] = '\0';
  e.timestamp = millis();
  e.rssi = rssi;
  e.seq = seq;
  e.senderEpoch = senderEpoch;
  e.localEpoch = localEpoch;

  _inboxHead = (_inboxHead + 1) % MAX_INBOX;
  if (_inboxCount < MAX_INBOX) _inboxCount++;
}

int Chat::inboxCount() { return _inboxCount; }

const InboxEntry& Chat::inboxEntry(int indexFromNewest)
{
  int idx = (_inboxHead - 1 - indexFromNewest + 2 * MAX_INBOX) % MAX_INBOX;
  return _inbox[idx];
}

ChatMessage& Chat::slotMut(uint8_t nodeId, int indexFromOldest)
{
  int n = nodeId - 1;
  int idx;
  if (_threadCount[n] < MAX_MSGS_PER_CHAT) {
    idx = indexFromOldest;
  } else {
    idx = (_threadHead[n] + indexFromOldest) % MAX_MSGS_PER_CHAT;
  }
  return _threads[n][idx];
}

void Chat::addChatMessage(uint8_t nodeId, bool outgoing, const char* text, uint16_t seq, uint32_t sentEpoch)
{
  if (nodeId < 1 || nodeId > MAX_CHAT_NODES) return;
  int n = nodeId - 1;

  ChatMessage &m = _threads[n][_threadHead[n]];
  m.outgoing = outgoing;
  strncpy(m.text, text, 31); m.text[31] = '\0';
  m.timestamp = millis();
  m.seq = seq;
  m.sentEpoch = sentEpoch;
  m.deliveredEpoch = 0;
  m.readEpoch = 0;
  m.status = MSG_SENT;
  m.hopCount = 0;

  _threadHead[n] = (_threadHead[n] + 1) % MAX_MSGS_PER_CHAT;
  if (_threadCount[n] < MAX_MSGS_PER_CHAT) _threadCount[n]++;
}

int Chat::chatMessageCount(uint8_t nodeId)
{
  if (nodeId < 1 || nodeId > MAX_CHAT_NODES) return 0;
  return _threadCount[nodeId - 1];
}

const ChatMessage& Chat::chatMessage(uint8_t nodeId, int indexFromOldest)
{
  return slotMut(nodeId, indexFromOldest);
}

void Chat::markOutgoingStatus(uint8_t peerNode, uint16_t refSeq, MsgStatus newStatus, uint32_t epoch, uint8_t hopCount, const uint8_t* path)
{
  if (peerNode < 1 || peerNode > MAX_CHAT_NODES) return;
  int count = chatMessageCount(peerNode);
  for (int i = 0; i < count; i++) {
    ChatMessage &m = slotMut(peerNode, i);
    if (m.outgoing && m.seq == refSeq) {
      if (newStatus > m.status) {   // never regress SENT -> DELIVERED -> READ
        m.status = newStatus;
        if (newStatus == MSG_DELIVERED) m.deliveredEpoch = epoch;
        if (newStatus == MSG_READ)      m.readEpoch = epoch;
      }
      // Record the relay path the first time an ack actually carries
      // one (the delivered ack); a later plain read-ack (hopCount==0)
      // must not overwrite it -- see the declaration comment above.
      if (hopCount > 0 && path) {
        m.hopCount = hopCount;
        memcpy(m.path, path, hopCount);
      }
      return;
    }
  }
}

bool Chat::nodeHasHistory(uint8_t nodeId)
{
  if (nodeId < 1 || nodeId > MAX_CHAT_NODES) return false;
  return _threadCount[nodeId - 1] > 0;
}

// --- Transmitter Outbox -------------------------------------------

void Chat::addOutboxEntry(uint8_t destination, const char* text, uint16_t seq, uint32_t sentEpoch, bool expectsRelay)
{
  OutboxEntry &e = _outbox[_outboxHead];
  e.destination = destination;
  strncpy(e.text, text, 31); e.text[31] = '\0';
  e.timestamp = millis();
  e.seq = seq;
  e.sentEpoch = sentEpoch;
  e.deliveredEpoch = 0;
  e.readEpoch = 0;
  e.status = MSG_SENT;   // visible immediately -- this is the whole point of the feature
  e.expectsRelay = expectsRelay;
  e.hopCount = 0;

  _outboxHead = (_outboxHead + 1) % MAX_OUTBOX;
  if (_outboxCount < MAX_OUTBOX) _outboxCount++;
}

int Chat::outboxCount() { return _outboxCount; }

const OutboxEntry& Chat::outboxEntry(int indexFromNewest)
{
  int idx = (_outboxHead - 1 - indexFromNewest + 2 * MAX_OUTBOX) % MAX_OUTBOX;
  return _outbox[idx];
}

// Same never-regress rule as markOutgoingStatus(): SENT -> DELIVERED
// -> READ only, matched by (destination, seq) since our own seq
// numbers are unique per node. Broadcast rows never match here (no
// ack path exists for them), which is correct -- they simply stay
// MSG_SENT.
void Chat::markOutboxStatus(uint8_t destination, uint16_t refSeq, MsgStatus newStatus, uint32_t epoch, uint8_t hopCount, const uint8_t* path)
{
  for (int i = 0; i < _outboxCount; i++) {
    int idx = (_outboxHead - 1 - i + 2 * MAX_OUTBOX) % MAX_OUTBOX;
    OutboxEntry &e = _outbox[idx];
    if (e.destination == destination && e.seq == refSeq) {
      if (newStatus > e.status) {
        e.status = newStatus;
        if (newStatus == MSG_DELIVERED) e.deliveredEpoch = epoch;
        if (newStatus == MSG_READ)      e.readEpoch = epoch;
      }
      if (hopCount > 0 && path) {   // see markOutgoingStatus() for why this is guarded
        e.hopCount = hopCount;
        memcpy(e.path, path, hopCount);
      }
      return;
    }
  }
}

// --- Broadcast Channel ----------------------------------------------

void Chat::addBroadcastSent(const char* text, uint16_t seq, uint32_t epoch)
{
  BroadcastLogEntry &e = _broadcastLog[_broadcastHead];
  e.outgoing = true;
  e.peerNode = 0;
  strncpy(e.text, text, 31); e.text[31] = '\0';
  e.timestamp = millis();
  e.seq = seq;
  e.epoch = epoch;

  _broadcastHead = (_broadcastHead + 1) % MAX_BROADCAST_LOG;
  if (_broadcastCount < MAX_BROADCAST_LOG) _broadcastCount++;
}

void Chat::addBroadcastReceived(uint8_t peerNode, const char* text, uint16_t seq, uint32_t epoch)
{
  BroadcastLogEntry &e = _broadcastLog[_broadcastHead];
  e.outgoing = false;
  e.peerNode = peerNode;
  strncpy(e.text, text, 31); e.text[31] = '\0';
  e.timestamp = millis();
  e.seq = seq;
  e.epoch = epoch;

  _broadcastHead = (_broadcastHead + 1) % MAX_BROADCAST_LOG;
  if (_broadcastCount < MAX_BROADCAST_LOG) _broadcastCount++;
}

int Chat::broadcastLogCount() { return _broadcastCount; }

const BroadcastLogEntry& Chat::broadcastLogEntry(int indexFromOldest)
{
  int start = (_broadcastHead - _broadcastCount + MAX_BROADCAST_LOG) % MAX_BROADCAST_LOG;
  int idx = (start + indexFromOldest) % MAX_BROADCAST_LOG;
  return _broadcastLog[idx];
}

// =================================================================
// DISPLAY MANAGER (all screen rendering)
// =================================================================

class DisplayManager
{
public:
  bool begin(uint8_t sdaPin, uint8_t sclPin, uint8_t screenW, uint8_t screenH);

  void drawSplash(uint8_t nodeId, const char* status, int neighborCount, int batteryPct);
  void drawList(const char* title, int neighborCount, int batteryPct,
                const char** items, int count, int selectedIndex, const char* clockStr);
  void drawListWithStatus(const char* title, int neighborCount, int batteryPct,
                           uint8_t selfId, const bool* onlineFlags,
                           const char** items, int count, int selectedIndex, const char* clockStr);
  void drawEmergencyGrid(const char* title, int neighborCount, int batteryPct,
                          const char** items, int count, int selectedIndex);
  void drawCompose(const char* destName, const char* buffer, char currentChar, bool isDel, bool isSend);
  void drawSending(int dotCount);
  void drawSentConfirm(const char* timeLabel);
  void drawReceiving(const char* fromName, const char* message, int rssi, int pendingCount, const char* sentLabel);
  void drawMessageDetail(const char* title, const char* nodeName, const char* text,
                          const char* sentLabel, const char* recvLabel, int rssi);
  void drawChat(const char* title, int neighborCount, int batteryPct,
                const char** lines, const bool* outgoingFlags, const MsgStatus* statuses,
                int lineCount, int scrollOffset, int visibleLines, const char* clockStr);
  void drawInfoScreen(const char* title, const char** lines, int lineCount);
  void drawOfflineWarning(const char* destLabel);
  void drawReconnectPopup(uint8_t nodeId);
  void drawSetClock(int hour, int minute, int field);

  int width();
  int height();

private:
  Adafruit_SSD1306* _display = nullptr;

  void drawBattery(int x, int y, int pct);
  void drawSignalBars(int x, int y, int level);   // level 0-4
  void drawTopBar(const char* title, int neighborCount, int batteryPct, const char* clockStr);
  void drawNodeStatusRow(uint8_t selfId, const bool* onlineFlags, int y);
  void drawTicks(int x, int y, MsgStatus status);
  int  rssiLevel(int rssi);
};

bool DisplayManager::begin(uint8_t sdaPin, uint8_t sclPin, uint8_t screenW, uint8_t screenH)
{
  Wire.begin(sdaPin, sclPin);
  _display = new Adafruit_SSD1306(screenW, screenH, &Wire, -1);

  if (!_display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) return false;

  _display->setRotation(0);
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->display();
  return true;
}

int DisplayManager::width()  { return _display->width(); }
int DisplayManager::height() { return _display->height(); }

int DisplayManager::rssiLevel(int rssi)
{
  if (rssi >= -70)  return 4;
  if (rssi >= -85)  return 3;
  if (rssi >= -100) return 2;
  if (rssi >= -115) return 1;
  return 0;
}

void DisplayManager::drawBattery(int x, int y, int pct)
{
  int w = 14, h = 8;
  _display->drawRect(x, y, w, h, SSD1306_WHITE);
  _display->fillRect(x + w, y + 2, 2, h - 4, SSD1306_WHITE);
  int fillW = ((w - 2) * pct) / 100;
  if (fillW > 0) _display->fillRect(x + 1, y + 1, fillW, h - 2, SSD1306_WHITE);
}

void DisplayManager::drawSignalBars(int x, int y, int level)
{
  if (level > 4) level = 4;
  if (level < 0) level = 0;
  int barW = 3, gap = 1;
  for (int i = 0; i < 4; i++) {
    int barH = 2 + i * 2;
    int bx = x + i * (barW + gap);
    int by = y + (8 - barH);
    if (i < level) _display->fillRect(bx, by, barW, barH, SSD1306_WHITE);
    else           _display->drawRect(bx, by, barW, barH, SSD1306_WHITE);
  }
}

// Draws a WhatsApp-style tick indicator for an OUTGOING message, with
// all three states distinct even on a monochrome 1-bit display:
//   MSG_SENT      -> one outline checkmark                    ( \/ )
//   MSG_DELIVERED -> two outline checkmarks                   ( \/\/ )
//   MSG_READ      -> two checkmarks on a small filled/inverted
//                    background tile, standing in for WhatsApp's
//                    "blue ticks" the way a 1-bit screen can: a solid
//                    highlight is unmistakably a different state at a
//                    glance, not just "the same icon, one more of it".
void DisplayManager::drawTicks(int x, int y, MsgStatus status)
{
  auto tick = [&](int tx, uint16_t color) {
    _display->drawLine(tx, y + 3, tx + 2, y + 5, color);
    _display->drawLine(tx + 2, y + 5, tx + 6, y - 1, color);
  };

  if (status == MSG_READ) {
    // Inverted tile behind both ticks: fill white, draw the ticks in
    // black on top of it, so "read" is visually a solid block rather
    // than just two more lines -- reads as distinct at a glance even
    // at this tiny size.
    _display->fillRect(x - 1, y - 2, 11, 9, SSD1306_WHITE);
    tick(x, SSD1306_BLACK);
    tick(x + 4, SSD1306_BLACK);
    return;
  }

  tick(x, SSD1306_WHITE);
  if (status == MSG_DELIVERED) tick(x + 4, SSD1306_WHITE);
}

void DisplayManager::drawTopBar(const char* title, int neighborCount, int batteryPct, const char* clockStr)
{
  _display->setTextColor(SSD1306_WHITE);
  _display->setTextSize(1);
  _display->setCursor(0, 0);
  _display->print(title);

  int battX = width() - 15;
  drawBattery(battX, 0, batteryPct);
  int sigX = battX - 18;
  drawSignalBars(sigX, 0, neighborCount);

  if (clockStr && clockStr[0]) {
    int16_t bx, by; uint16_t bw, bh;
    _display->getTextBounds(clockStr, 0, 0, &bx, &by, &bw, &bh);
    int cx = sigX - (int)bw - 4;
    int titleEnd = (int)strlen(title) * 6 + 2;   // rough textSize1 char width, avoids overlapping the title
    if (cx > titleEnd) {
      _display->setCursor(cx, 0);
      _display->print(clockStr);
    }
  }

  _display->drawFastHLine(0, 10, width(), SSD1306_WHITE);
}

void DisplayManager::drawNodeStatusRow(uint8_t selfId, const bool* onlineFlags, int y)
{
  _display->setTextSize(1);
  int x = 0;
  for (uint8_t id = 1; id <= 4; id++) {
    if (id == selfId) continue;
    char label[4];
    snprintf(label, sizeof(label), "%d:", id);
    _display->setCursor(x, y);
    _display->print(label);
    x += 10;
    if (onlineFlags[id]) {
      _display->fillCircle(x + 2, y + 3, 3, SSD1306_WHITE);
    } else {
      _display->drawLine(x, y, x + 5, y + 6, SSD1306_WHITE);
      _display->drawLine(x, y + 6, x + 5, y, SSD1306_WHITE);
    }
    x += 11;
  }
  _display->drawFastHLine(0, y + 10, width(), SSD1306_WHITE);
}

void DisplayManager::drawSplash(uint8_t nodeId, const char* status, int neighborCount, int batteryPct)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);

  _display->drawRoundRect(0, 0, width(), height(), 4, SSD1306_WHITE);

  _display->setTextSize(2);
  const char* titleTxt = "MESH LINK";
  int16_t x1, y1; uint16_t tw, th;
  _display->getTextBounds(titleTxt, 0, 0, &x1, &y1, &tw, &th);
  _display->setCursor((width() - (int)tw) / 2, 6);
  _display->print(titleTxt);

  _display->setTextSize(1);
  char nodeLine[20];
  snprintf(nodeLine, sizeof(nodeLine), "NODE %d", nodeId);
  _display->getTextBounds(nodeLine, 0, 0, &x1, &y1, &tw, &th);
  _display->setCursor((width() - (int)tw) / 2, 26);
  _display->print(nodeLine);

  _display->setCursor(6, 40);
  _display->print(status);

  drawSignalBars(width() - 34, 40, neighborCount);
  drawBattery(width() - 16, 40, batteryPct);

  _display->setCursor(6, 54);
  _display->print("Click to continue");

  _display->display();
}

void DisplayManager::drawList(const char* title, int neighborCount, int batteryPct,
                               const char** items, int count, int selectedIndex, const char* clockStr)
{
  _display->clearDisplay();
  drawTopBar(title, neighborCount, batteryPct, clockStr);

  const int rowH = 12;
  const int top = 13;
  int maxVisible = (height() - top) / rowH;
  if (maxVisible < 1) maxVisible = 1;

  int start = 0;
  if (count > maxVisible) {
    start = selectedIndex - maxVisible / 2;
    if (start < 0) start = 0;
    if (start > count - maxVisible) start = count - maxVisible;
  }

  int y = top;
  for (int i = start; i < count && i < start + maxVisible; i++) {
    bool sel = (i == selectedIndex);
    _display->setCursor(4, y + 2);
    if (sel) {
      _display->fillRoundRect(0, y, width(), rowH - 1, 2, SSD1306_WHITE);
      _display->setTextColor(SSD1306_BLACK);
      _display->print(">");
    } else {
      _display->setTextColor(SSD1306_WHITE);
    }
    _display->print(items[i]);
    y += rowH;
  }
  _display->setTextColor(SSD1306_WHITE);

  if (start > 0) {
    _display->fillTriangle(width() - 6, top + 1, width() - 2, top + 1, width() - 4, top - 2, SSD1306_WHITE);
  }
  if (start + maxVisible < count) {
    int by = top + maxVisible * rowH;
    _display->fillTriangle(width() - 6, by - 2, width() - 2, by - 2, width() - 4, by + 1, SSD1306_WHITE);
  }

  _display->display();
}

void DisplayManager::drawListWithStatus(const char* title, int neighborCount, int batteryPct,
                                         uint8_t selfId, const bool* onlineFlags,
                                         const char** items, int count, int selectedIndex, const char* clockStr)
{
  _display->clearDisplay();
  drawTopBar(title, neighborCount, batteryPct, clockStr);
  drawNodeStatusRow(selfId, onlineFlags, 12);

  const int rowH = 12;
  const int top = 23;
  int maxVisible = (height() - top) / rowH;
  if (maxVisible < 1) maxVisible = 1;

  int start = 0;
  if (count > maxVisible) {
    start = selectedIndex - maxVisible / 2;
    if (start < 0) start = 0;
    if (start > count - maxVisible) start = count - maxVisible;
  }

  int y = top;
  for (int i = start; i < count && i < start + maxVisible; i++) {
    bool sel = (i == selectedIndex);
    _display->setCursor(4, y + 2);
    if (sel) {
      _display->fillRoundRect(0, y, width(), rowH - 1, 2, SSD1306_WHITE);
      _display->setTextColor(SSD1306_BLACK);
      _display->print(">");
    } else {
      _display->setTextColor(SSD1306_WHITE);
    }
    _display->print(items[i]);
    y += rowH;
  }
  _display->setTextColor(SSD1306_WHITE);

  if (start > 0) {
    _display->fillTriangle(width() - 6, top + 1, width() - 2, top + 1, width() - 4, top - 2, SSD1306_WHITE);
  }
  if (start + maxVisible < count) {
    int by = top + maxVisible * rowH;
    _display->fillTriangle(width() - 6, by - 2, width() - 2, by - 2, width() - 4, by + 1, SSD1306_WHITE);
  }

  _display->display();
}

void DisplayManager::drawEmergencyGrid(const char* title, int neighborCount, int batteryPct,
                                        const char** items, int count, int selectedIndex)
{
  _display->clearDisplay();
  drawTopBar(title, neighborCount, batteryPct, nullptr);

  int cols = 2;
  int top = 13;
  int cellW = width() / cols;
  int rows = (count + cols - 1) / cols;
  if (rows < 1) rows = 1;
  int gap = 2;
  int cellH = (height() - top - gap) / rows;

  for (int i = 0; i < count; i++) {
    int col = i % cols;
    int row = i / cols;
    int x = col * cellW + gap;
    int y = top + row * cellH + gap / 2;
    int w = cellW - gap * 2;
    int h = cellH - gap;

    bool sel = (i == selectedIndex);
    if (sel) {
      _display->fillRoundRect(x, y, w, h, 3, SSD1306_WHITE);
      _display->setTextColor(SSD1306_BLACK);
    } else {
      _display->drawRoundRect(x, y, w, h, 3, SSD1306_WHITE);
      _display->setTextColor(SSD1306_WHITE);
    }

    int16_t bx, by; uint16_t bw, bh;
    _display->setTextSize(1);
    _display->getTextBounds(items[i], 0, 0, &bx, &by, &bw, &bh);
    _display->setCursor(x + (w - (int)bw) / 2, y + (h - (int)bh) / 2);
    _display->print(items[i]);
  }
  _display->setTextColor(SSD1306_WHITE);
  _display->display();
}

void DisplayManager::drawCompose(const char* destName, const char* buffer, char currentChar, bool isDel, bool isSend)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->setTextSize(1);
  _display->setCursor(0, 0);
  _display->print("To: "); _display->print(destName);
  _display->drawFastHLine(0, 9, width(), SSD1306_WHITE);

  _display->setCursor(0, 12);
  _display->print(">"); _display->print(buffer);

  _display->drawRoundRect(width() / 2 - 20, 26, 40, 18, 3, SSD1306_WHITE);
  if (isSend) {
    _display->setTextSize(2);
    _display->setCursor(width() / 2 - 13, 29);
    _display->print("OK");
  } else if (isDel) {
    _display->setTextSize(1);
    _display->setCursor(width() / 2 - 12, 32);
    _display->print("DEL");
  } else {
    _display->setTextSize(2);
    _display->setCursor(width() / 2 - 5, 29);
    _display->print(currentChar);
  }

  _display->setTextSize(1);
  _display->setCursor(0, 47);
  _display->print("Turn=Char Click=Pick");
  _display->setCursor(0, 56);
  _display->print("<Del  #Send  Hold=Bk");
  _display->display();
}

void DisplayManager::drawSending(int dotCount)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->setTextSize(2);
  _display->setCursor(6, 24);
  _display->print("Sending");
  for (int i = 0; i < dotCount; i++) _display->print(".");
  _display->display();
}

void DisplayManager::drawSentConfirm(const char* timeLabel)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->setTextSize(1);
  _display->setCursor(4, 16);
  _display->print("Message Sent");

  _display->drawRoundRect(width() / 2 - 20, 28, 40, 28, 4, SSD1306_WHITE);
  _display->setTextSize(2);
  _display->setCursor(width() / 2 - 12, 34);
  _display->print("OK");

  if (timeLabel && timeLabel[0]) {
    _display->setTextSize(1);
    int16_t bx, by; uint16_t bw, bh;
    _display->getTextBounds(timeLabel, 0, 0, &bx, &by, &bw, &bh);
    _display->setCursor((width() - (int)bw) / 2, 58);
    _display->print(timeLabel);
  }
  _display->display();
}

void DisplayManager::drawReceiving(const char* fromName, const char* message, int rssi, int pendingCount, const char* sentLabel)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->setTextSize(1);
  _display->setCursor(0, 0);
  _display->print("! INCOMING ");
  _display->print(fromName);

  if (pendingCount > 0) {
    char badge[8];
    snprintf(badge, sizeof(badge), "+%d", pendingCount);
    int16_t bx, by; uint16_t bw, bh;
    _display->getTextBounds(badge, 0, 0, &bx, &by, &bw, &bh);
    _display->setCursor(width() - (int)bw - 2, 0);
    _display->print(badge);
  }
  _display->drawFastHLine(0, 9, width(), SSD1306_WHITE);

  _display->drawRoundRect(2, 12, width() - 4, 26, 3, SSD1306_WHITE);

  int msgLen = strlen(message);
  int fontSize = (msgLen > 9) ? 1 : 2;
  _display->setTextSize(fontSize);
  int16_t bx, by; uint16_t bw, bh;
  _display->getTextBounds(message, 0, 0, &bx, &by, &bw, &bh);
  int tx = (width() - (int)bw) / 2;
  if (tx < 4) tx = 4;
  _display->setCursor(tx, fontSize == 2 ? 18 : 22);
  _display->print(message);

  _display->setTextSize(1);
  drawSignalBars(2, 40, rssiLevel(rssi));
  _display->setCursor(24, 40);
  _display->print(rssi); _display->print(" dBm");

  if (sentLabel && sentLabel[0]) {
    _display->setCursor(2, 49);
    _display->print(sentLabel);
  }

  _display->setCursor(2, 57);
  _display->print("Hold=Dismiss+Read");

  _display->display();
}

void DisplayManager::drawMessageDetail(const char* title, const char* nodeName, const char* text,
                                        const char* sentLabel, const char* recvLabel, int rssi)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->setTextSize(1);
  _display->setCursor(0, 0);
  _display->print(title); _display->print(" "); _display->print(nodeName);
  drawSignalBars(width() - 20, 0, rssiLevel(rssi));
  _display->drawFastHLine(0, 9, width(), SSD1306_WHITE);

  int msgLen = strlen(text);
  int fontSize = (msgLen > 9) ? 1 : 2;
  _display->setTextSize(fontSize);
  int16_t bx, by; uint16_t bw, bh;
  _display->getTextBounds(text, 0, 0, &bx, &by, &bw, &bh);
  int tx = (width() - (int)bw) / 2;
  if (tx < 0) tx = 0;
  _display->setCursor(tx, 14);
  _display->print(text);

  _display->setTextSize(1);
  _display->setCursor(0, 38);
  _display->print(sentLabel);
  _display->setCursor(0, 48);
  _display->print(recvLabel);

  _display->setCursor(0, height() - 9);
  _display->print("Hold=Back");
  _display->display();
}

void DisplayManager::drawChat(const char* title, int neighborCount, int batteryPct,
                               const char** lines, const bool* outgoingFlags, const MsgStatus* statuses,
                               int lineCount, int scrollOffset, int visibleLines, const char* clockStr)
{
  _display->clearDisplay();
  drawTopBar(title, neighborCount, batteryPct, clockStr);

  int y = 13;
  int rowH = 11;
  _display->setTextSize(1);
  for (int i = 0; i < visibleLines; i++) {
    int idx = scrollOffset + i;
    if (idx >= lineCount) break;

    bool out = outgoingFlags[idx];
    int16_t bx, by; uint16_t bw, bh;
    _display->getTextBounds(lines[idx], 0, 0, &bx, &by, &bw, &bh);

    if (out) {
      // Reserve room on the right for the tick(s) -- they replace the
      // old ">" marker, so an outgoing row now reads "<text>  \/\/".
      int textRight = width() - 10;
      int x = textRight - (int)bw;
      if (x < 8) x = 8;
      _display->setCursor(x, y);
      _display->print(lines[idx]);
      drawTicks(width() - 9, y + 1, statuses[idx]);
    } else {
      _display->setCursor(0, y);
      _display->print("<");
      _display->setCursor(8, y);
      _display->print(lines[idx]);
    }
    y += rowH;
  }
  _display->display();
}

void DisplayManager::drawInfoScreen(const char* title, const char** lines, int lineCount)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->setTextSize(1);
  _display->setCursor(0, 0);
  _display->print(title);
  _display->drawFastHLine(0, 9, width(), SSD1306_WHITE);

  int y = 13;
  for (int i = 0; i < lineCount; i++) {
    _display->setCursor(0, y);
    _display->print(lines[i]);
    y += 10;
  }

  _display->setCursor(0, height() - 9);
  _display->print("Hold=Back");
  _display->display();
}

void DisplayManager::drawOfflineWarning(const char* destLabel)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->drawRoundRect(0, 0, width(), height(), 4, SSD1306_WHITE);

  _display->setTextSize(1);
  _display->setCursor(8, 8);
  _display->print(destLabel);
  _display->print(" OFFLINE");

  _display->drawLine(width() / 2 - 8, 20, width() / 2 + 8, 36, SSD1306_WHITE);
  _display->drawLine(width() / 2 - 8, 36, width() / 2 + 8, 20, SSD1306_WHITE);

  _display->setCursor(4, 42);
  _display->print("Not delivered - node");
  _display->setCursor(4, 52);
  _display->print("unreachable. Sending");
  _display->display();
}

void DisplayManager::drawReconnectPopup(uint8_t nodeId)
{
  _display->clearDisplay();
  _display->drawRoundRect(2, 10, width() - 4, height() - 20, 4, SSD1306_WHITE);

  _display->setTextSize(1);
  char line[24];
  snprintf(line, sizeof(line), "Node %d Reconnected", nodeId);
  int16_t x1, y1; uint16_t tw, th;
  _display->getTextBounds(line, 0, 0, &x1, &y1, &tw, &th);
  _display->setCursor((width() - (int)tw) / 2, 22);
  _display->print(line);

  const char* sub = "Back Online";
  _display->getTextBounds(sub, 0, 0, &x1, &y1, &tw, &th);
  _display->setCursor((width() - (int)tw) / 2, 38);
  _display->print(sub);

  _display->display();
}

void DisplayManager::drawSetClock(int hour, int minute, int field)
{
  _display->clearDisplay();
  _display->setTextColor(SSD1306_WHITE);
  _display->setTextSize(1);
  _display->setCursor(0, 0);
  _display->print("SET CLOCK (24h)");
  _display->drawFastHLine(0, 9, width(), SSD1306_WHITE);

  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", hour, minute);
  _display->setTextSize(3);
  int16_t x1, y1; uint16_t tw, th;
  _display->getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
  int tx = (width() - (int)tw) / 2;
  _display->setCursor(tx, 22);
  _display->print(buf);

  int fieldW = (int)tw / 2;
  int ux = (field == 0) ? tx : tx + fieldW + 6;
  _display->drawFastHLine(ux, 50, fieldW - 4, SSD1306_WHITE);

  _display->setTextSize(1);
  _display->setCursor(0, height() - 9);
  _display->print("Turn=Adj Click=Next");
  _display->display();
}

// =================================================================
// MENU (full app state machine)
// =================================================================

enum Screen {
  SCR_SPLASH, SCR_MAIN_MENU, SCR_DEVICE_SELECT, SCR_SEND_MENU, SCR_EMERGENCY,
  SCR_COMPOSE, SCR_SENDING, SCR_SENT, SCR_RECEIVING, SCR_INBOX,
  SCR_MESSAGE_DETAIL, SCR_PREV_CHATS, SCR_CHAT, SCR_SETTINGS, SCR_INFO,
  SCR_OFFLINE_WARNING, SCR_RECONNECT_POPUP, SCR_SET_CLOCK,
  SCR_OUTBOX,              // transmitter's own sent/delivered/read log
  SCR_BROADCAST_CHANNEL    // shared all-nodes broadcast thread
};

// Bridges LoRaManager's plain C function-pointer epoch provider to
// the Clock instance owned by main()/Menu (set once in Menu::begin).
static Clock* g_clockForProvider = nullptr;
static uint32_t epochProviderTrampoline() { return g_clockForProvider ? g_clockForProvider->nowEpoch() : 0; }

class Menu
{
public:
  void begin(DisplayManager* display, LoRaManager* lora, Rotary* rotary, Chat* chat, Clock* clock, uint8_t nodeId);
  void update();
  static void onPacketReceivedTrampoline(const Packet& pkt, int rssi);
  static void onNodeStatusChangeTrampoline(uint8_t nodeId, bool online);
  static void onAckReceivedTrampoline(uint8_t fromNode, uint16_t refSeq, uint8_t ackType, uint32_t epoch, uint8_t hopCount, const uint8_t* path);

private:
  static Menu* _instance;
  void handleIncoming(const Packet& pkt, int rssi);
  void handleNodeStatusChange(uint8_t nodeId, bool online);
  void handleAckReceived(uint8_t fromNode, uint16_t refSeq, uint8_t ackType, uint32_t epoch, uint8_t hopCount, const uint8_t* path);
  void formatPathLabel(uint8_t destination, uint8_t hopCount, const uint8_t* path, char* out, size_t outLen);

  // Dedupes outgoing PKT_ACK_READ sends: a message can be "opened" many
  // times (revisiting a chat thread, reopening a detail view) but should
  // only ever ack as read once. Small fixed ring, no heap allocation.
  static const int READ_ACK_CACHE_SIZE = 16;
  struct ReadAckRecord { uint8_t peer; uint16_t seq; };
  ReadAckRecord _readAckSent[READ_ACK_CACHE_SIZE];
  int _readAckSentCount = 0;
  int _readAckSentIdx = 0;
  bool alreadySentReadAck(uint8_t peer, uint16_t seq);
  void markMessageRead(uint8_t peer, uint16_t seq);

  DisplayManager* _display;
  LoRaManager* _lora;
  Rotary* _rotary;
  Chat* _chat;
  Clock* _clock;
  uint8_t _nodeId;

  Screen _screen = SCR_SPLASH;
  bool _dirty = true;
  unsigned long _screenEnteredAt = 0;

  int _delta = 0;
  ButtonEvent _btnAction = BTN_NONE;

  int _mainMenuIndex = 0;
  int _deviceIndex = 0;
  int _sendMenuIndex = 0;
  int _emergencyIndex = 0;
  int _inboxIndex = 0;
  int _prevChatsIndex = 0;
  int _settingsIndex = 0;
  int _chatScrollOffset = 0;
  int _outboxIndex = 0;
  int _broadcastIndex = 0;
  int _broadcastScrollOffset = 0;

  uint8_t _selectedDest = 0;
  uint8_t _selectedChatNode = 0;
  int _selectedInboxEntry = 0;

  bool _emergencyFromQuickAccess = false;
  Screen _composeReturnScreen = SCR_SEND_MENU;   // where Back from SCR_COMPOSE goes

  uint8_t _deviceIds[5];
  char _deviceLabels[5][16];
  const char* _deviceLabelPtrs[5];
  int _deviceCount = 0;

  char _composeBuffer[32];
  int _composeLen = 0;
  int _composeCharIndex = 0;

  char _pendingText[32];

  MessageQueue _msgQueue;
  DisplayMessage _currentDisplayMsg;
  char _currentDisplayFromLabel[16];

  bool _nodeOnline[5] = { false, false, false, false, false };
  uint8_t _reconnectedNodeId = 0;
  Screen _screenBeforePopup = SCR_MAIN_MENU;
  Screen _screenBeforeIncoming = SCR_MAIN_MENU;

  unsigned long _animTimer = 0;
  int _animFrame = 0;

  int _batteryPct = 82;

  int _setClockField = 0;
  int _setClockHour = 0;
  int _setClockMinute = 0;

  char _clockBuf[9];   // "HH:MM:SS" scratch, refreshed once per render()

  void goTo(Screen s);
  void render();
  void handleBack();

  void updateSplash();
  void updateMainMenu();
  void updateDeviceSelect();
  void updateSendMenu();
  void updateEmergency();
  void updateCompose();
  void updateSending();
  void updateSent();
  void updateReceiving();
  void updateInbox();
  void updateMessageDetail();
  void updatePrevChats();
  void updateChat();
  void updateSettings();
  void updateInfo();
  void updateOfflineWarning();
  void updateReconnectPopup();
  void updateSetClock();
  void updateOutbox();
  void updateBroadcastChannel();

  void buildDeviceList();
  bool sendCurrentPending();
  void beginSendFlow();
  void formatTimeLabel(const char* prefix, uint32_t epoch, unsigned long fallbackMillisTimestamp, char* out, size_t outLen);

  const char* _infoTitle;
  char _infoLines[4][24];
  int _infoLineCount;
  void showInfo(const char* title, const char* line0, const char* line1 = nullptr, const char* line2 = nullptr);
};

Menu* Menu::_instance = nullptr;

static const char* MAIN_MENU_ITEMS[]  = { "EMERGENCY", "Send Message", "Broadcast", "Inbox", "Outbox", "Prev Chats", "Settings" };
#define MAIN_MENU_COUNT 7
static const char* SEND_MENU_ITEMS[]  = { "Emergency", "Medical", "Fire", "Need Backup", "Hostage Found", "Custom Message" };
static const char* EMERGENCY_ITEMS[]  = { "HELP", "CALL ME", "URGENT", "SOS", "MEDICAL", "FIRE" };
static const char* SETTINGS_ITEMS[]   = { "Node ID", "Brightness", "LoRa Frequency", "Set Clock", "Factory Reset", "About" };
static const char  CHARSET[]          = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 <#";
static const int   CHARSET_LEN        = sizeof(CHARSET) - 1;
static const unsigned long SPLASH_DURATION_MS = 2000;

void Menu::begin(DisplayManager* display, LoRaManager* lora, Rotary* rotary, Chat* chat, Clock* clock, uint8_t nodeId)
{
  _display = display;
  _lora = lora;
  _rotary = rotary;
  _chat = chat;
  _clock = clock;
  _nodeId = nodeId;

  _instance = this;
  g_clockForProvider = clock;
  _lora->setOnReceive(onPacketReceivedTrampoline);
  _lora->setOnNodeStatusChange(onNodeStatusChangeTrampoline);
  _lora->setOnAckReceived(onAckReceivedTrampoline);
  _lora->setEpochProvider(epochProviderTrampoline);

  _screen = SCR_SPLASH;
  _screenEnteredAt = millis();
  _dirty = true;
}

void Menu::onPacketReceivedTrampoline(const Packet& pkt, int rssi)
{
  if (_instance) _instance->handleIncoming(pkt, rssi);
}

void Menu::onNodeStatusChangeTrampoline(uint8_t nodeId, bool online)
{
  if (_instance) _instance->handleNodeStatusChange(nodeId, online);
}

void Menu::onAckReceivedTrampoline(uint8_t fromNode, uint16_t refSeq, uint8_t ackType, uint32_t epoch, uint8_t hopCount, const uint8_t* path)
{
  if (_instance) _instance->handleAckReceived(fromNode, refSeq, ackType, epoch, hopCount, path);
}

void Menu::handleIncoming(const Packet& pkt, int rssi)
{
  uint32_t localEpoch = _clock->nowEpoch();
  _chat->addInboxEntry(pkt.source, pkt.payload, rssi, pkt.seq, pkt.epoch, localEpoch);
  if (pkt.destination == BROADCAST_ID) {
    // Feature: Broadcast Channel -- someone else's broadcast joins the
    // same shared thread our own broadcasts go into (addBroadcastSent
    // in sendCurrentPending()), so the channel reads as one timeline.
    _chat->addBroadcastReceived(pkt.source, pkt.payload, pkt.seq, pkt.epoch);
  } else {
    _chat->addChatMessage(pkt.source, false, pkt.payload, pkt.seq, pkt.epoch);
  }
  _msgQueue.push(pkt.source, pkt.payload, rssi, pkt.seq, pkt.epoch);

  if (_screen != SCR_RECEIVING) {
    _screenBeforeIncoming = _screen;
    if (_msgQueue.pop(_currentDisplayMsg)) {
      snprintf(_currentDisplayFromLabel, sizeof(_currentDisplayFromLabel), "Node %d", _currentDisplayMsg.nodeId);
      goTo(SCR_RECEIVING);
    }
  }
}

void Menu::handleNodeStatusChange(uint8_t nodeId, bool online)
{
  if (nodeId < 1 || nodeId > 4) return;
  _nodeOnline[nodeId] = online;

  if (online) {
    if (_screen == SCR_COMPOSE) return;
    _reconnectedNodeId = nodeId;
    _screenBeforePopup = _screen;
    goTo(SCR_RECONNECT_POPUP);
  } else {
    _dirty = true;
  }
}

void Menu::handleAckReceived(uint8_t fromNode, uint16_t refSeq, uint8_t ackType, uint32_t epoch, uint8_t hopCount, const uint8_t* path)
{
  MsgStatus newStatus = (ackType == PKT_ACK_READ) ? MSG_READ : MSG_DELIVERED;
  _chat->markOutgoingStatus(fromNode, refSeq, newStatus, epoch, hopCount, path);
  _chat->markOutboxStatus(fromNode, refSeq, newStatus, epoch, hopCount, path);   // keep the Outbox status in sync too
  _dirty = true;   // refresh ticks immediately if we're looking at that chat right now
}

// Builds a human-readable relay chain like "1>3>4" (us -> each
// recorded relay hop -> the final destination) for display. Empty
// string if there's nothing to show yet (broadcast, or no ack with a
// path has arrived yet).
void Menu::formatPathLabel(uint8_t destination, uint8_t hopCount, const uint8_t* path, char* out, size_t outLen)
{
  out[0] = '\0';
  if (hopCount == 0 || destination == BROADCAST_ID) return;

  int p = snprintf(out, outLen, "%d", _nodeId);
  for (int i = 0; i < hopCount && p < (int)outLen - 4; i++) {
    p += snprintf(out + p, outLen - p, ">%d", path[i]);
  }
  if (p < (int)outLen - 4) {
    snprintf(out + p, outLen - p, ">%d", destination);
  }
}

bool Menu::alreadySentReadAck(uint8_t peer, uint16_t seq)
{
  for (int i = 0; i < _readAckSentCount; i++)
    if (_readAckSent[i].peer == peer && _readAckSent[i].seq == seq) return true;
  return false;
}

void Menu::markMessageRead(uint8_t peer, uint16_t seq)
{
  if (alreadySentReadAck(peer, seq)) return;
  _readAckSent[_readAckSentIdx] = { peer, seq };
  _readAckSentIdx = (_readAckSentIdx + 1) % READ_ACK_CACHE_SIZE;
  if (_readAckSentCount < READ_ACK_CACHE_SIZE) _readAckSentCount++;

  _lora->sendAck(peer, PKT_ACK_READ, seq, _clock->nowEpoch());
}

void Menu::goTo(Screen s)
{
  _screen = s;
  _screenEnteredAt = millis();
  _dirty = true;
}

void Menu::handleBack()
{
  switch (_screen) {
    case SCR_DEVICE_SELECT:   goTo(SCR_MAIN_MENU); break;
    case SCR_SEND_MENU:       goTo(SCR_DEVICE_SELECT); buildDeviceList(); break;
    case SCR_EMERGENCY:
      if (_emergencyFromQuickAccess) goTo(SCR_MAIN_MENU);
      else                           goTo(SCR_SEND_MENU);
      break;
    case SCR_COMPOSE:         goTo(_composeReturnScreen); break;
    case SCR_RECEIVING:
      // "Hold=Dismiss" is a deliberate human action -- this is one of
      // the two ways a message counts as actually read (the other is
      // opening it via Inbox/Chat). A dwell timeout, by contrast, does
      // NOT count as read (see updateReceiving()).
      markMessageRead(_currentDisplayMsg.nodeId, _currentDisplayMsg.seq);
      if (_msgQueue.pop(_currentDisplayMsg)) {
        snprintf(_currentDisplayFromLabel, sizeof(_currentDisplayFromLabel), "Node %d", _currentDisplayMsg.nodeId);
        goTo(SCR_RECEIVING);
      } else {
        goTo(_screenBeforeIncoming);
      }
      break;
    case SCR_INBOX:           goTo(SCR_MAIN_MENU); break;
    case SCR_OUTBOX:          goTo(SCR_MAIN_MENU); break;
    case SCR_BROADCAST_CHANNEL: goTo(SCR_MAIN_MENU); break;
    case SCR_MESSAGE_DETAIL:  goTo(SCR_INBOX); break;
    case SCR_PREV_CHATS:      goTo(SCR_MAIN_MENU); break;
    case SCR_CHAT:            goTo(SCR_PREV_CHATS); break;
    case SCR_SETTINGS:        goTo(SCR_MAIN_MENU); break;
    case SCR_INFO:            goTo(SCR_SETTINGS); break;
    case SCR_SET_CLOCK:       goTo(SCR_SETTINGS); break;
    default: break;
  }
}

void Menu::buildDeviceList()
{
  _deviceCount = 0;
  for (uint8_t id = 1; id <= 4; id++) {
    if (id == _nodeId) continue;
    snprintf(_deviceLabels[_deviceCount], sizeof(_deviceLabels[_deviceCount]), "Node %d", id);
    _deviceIds[_deviceCount] = id;
    _deviceLabelPtrs[_deviceCount] = _deviceLabels[_deviceCount];
    _deviceCount++;
  }
  snprintf(_deviceLabels[_deviceCount], sizeof(_deviceLabels[_deviceCount]), "Broadcast");
  _deviceIds[_deviceCount] = BROADCAST_ID;
  _deviceLabelPtrs[_deviceCount] = _deviceLabels[_deviceCount];
  _deviceCount++;

  _deviceIndex = 0;
}

void Menu::showInfo(const char* title, const char* line0, const char* line1, const char* line2)
{
  _infoTitle = title;
  _infoLineCount = 0;
  if (line0) { strncpy(_infoLines[_infoLineCount], line0, 23); _infoLines[_infoLineCount][23] = '\0'; _infoLineCount++; }
  if (line1) { strncpy(_infoLines[_infoLineCount], line1, 23); _infoLines[_infoLineCount][23] = '\0'; _infoLineCount++; }
  if (line2) { strncpy(_infoLines[_infoLineCount], line2, 23); _infoLines[_infoLineCount][23] = '\0'; _infoLineCount++; }
  goTo(SCR_INFO);
}

// Builds a "<prefix>HH:MM:SS" label from an embedded epoch if one is
// available (nonzero), otherwise falls back to "<prefix>Xs ago" using
// a local millis() timestamp -- so every screen degrades gracefully
// when a clock (ours or the sender's) hasn't been set.
void Menu::formatTimeLabel(const char* prefix, uint32_t epoch, unsigned long fallbackMillisTimestamp, char* out, size_t outLen)
{
  if (epoch != 0) {
    char clockText[9];
    formatClock(epoch, clockText, sizeof(clockText));
    snprintf(out, outLen, "%s%s", prefix, clockText);
  } else {
    unsigned long ageSec = (millis() - fallbackMillisTimestamp) / 1000;
    snprintf(out, outLen, "%s%lus ago", prefix, ageSec);
  }
}

void Menu::update()
{
  _delta = _rotary->readDelta();
  _btnAction = _rotary->pollButton();

  if (_btnAction == BTN_LONG_PRESS) {
    handleBack();
    render();
    return;
  }

  switch (_screen) {
    case SCR_SPLASH:          updateSplash();         break;
    case SCR_MAIN_MENU:       updateMainMenu();       break;
    case SCR_DEVICE_SELECT:   updateDeviceSelect();   break;
    case SCR_SEND_MENU:       updateSendMenu();       break;
    case SCR_EMERGENCY:       updateEmergency();      break;
    case SCR_COMPOSE:         updateCompose();        break;
    case SCR_SENDING:         updateSending();        break;
    case SCR_SENT:            updateSent();           break;
    case SCR_RECEIVING:       updateReceiving();      break;
    case SCR_INBOX:           updateInbox();          break;
    case SCR_MESSAGE_DETAIL:  updateMessageDetail();  break;
    case SCR_PREV_CHATS:      updatePrevChats();      break;
    case SCR_CHAT:            updateChat();           break;
    case SCR_SETTINGS:        updateSettings();       break;
    case SCR_INFO:            updateInfo();           break;
    case SCR_OFFLINE_WARNING: updateOfflineWarning(); break;
    case SCR_RECONNECT_POPUP: updateReconnectPopup(); break;
    case SCR_SET_CLOCK:       updateSetClock();       break;
    case SCR_OUTBOX:          updateOutbox();         break;
    case SCR_BROADCAST_CHANNEL: updateBroadcastChannel(); break;
  }

  render();
}

void Menu::render()
{
  if (!_dirty) return;
  _dirty = false;

  int neighbors = _lora->activeNeighborCount();

  const char* clockStr = nullptr;
  if (_clock->isSet()) {
    formatClock(_clock->nowEpoch(), _clockBuf, sizeof(_clockBuf));
    clockStr = _clockBuf;
  }

  switch (_screen) {
    case SCR_SPLASH: {
      _display->drawSplash(_nodeId, "Mesh Ready", neighbors, _batteryPct);
      break;
    }
    case SCR_MAIN_MENU: {
      char title[20]; snprintf(title, sizeof(title), "N%d  MAIN MENU", _nodeId);
      _display->drawListWithStatus(title, neighbors, _batteryPct, _nodeId, _nodeOnline,
                                    MAIN_MENU_ITEMS, MAIN_MENU_COUNT, _mainMenuIndex, clockStr);
      break;
    }
    case SCR_DEVICE_SELECT: {
      static char rows[5][20];
      static const char* rowPtrs[5];
      for (int i = 0; i < _deviceCount; i++) {
        uint8_t id = _deviceIds[i];
        if (id == BROADCAST_ID) {
          snprintf(rows[i], sizeof(rows[i]), "Broadcast");
        } else {
          snprintf(rows[i], sizeof(rows[i]), "Node %d [%s]", id, _nodeOnline[id] ? "ON" : "OFF");
        }
        rowPtrs[i] = rows[i];
      }
      char title[20]; snprintf(title, sizeof(title), "N%d  SELECT DEST", _nodeId);
      _display->drawListWithStatus(title, neighbors, _batteryPct, _nodeId, _nodeOnline,
                                    rowPtrs, _deviceCount, _deviceIndex, clockStr);
      break;
    }
    case SCR_SEND_MENU: {
      char destLabel[12];
      if (_selectedDest == BROADCAST_ID) snprintf(destLabel, sizeof(destLabel), "ALL");
      else snprintf(destLabel, sizeof(destLabel), "N%d", _selectedDest);
      char title[24]; snprintf(title, sizeof(title), "SEND -> %s", destLabel);
      _display->drawListWithStatus(title, neighbors, _batteryPct, _nodeId, _nodeOnline,
                                    SEND_MENU_ITEMS, 6, _sendMenuIndex, clockStr);
      break;
    }
    case SCR_EMERGENCY: {
      char destLabel[12];
      if (_selectedDest == BROADCAST_ID) snprintf(destLabel, sizeof(destLabel), "ALL");
      else snprintf(destLabel, sizeof(destLabel), "N%d", _selectedDest);
      char title[24]; snprintf(title, sizeof(title), "SOS -> %s", destLabel);
      _display->drawEmergencyGrid(title, neighbors, _batteryPct, EMERGENCY_ITEMS, 6, _emergencyIndex);
      break;
    }
    case SCR_COMPOSE: {
      char destLabel[12];
      if (_selectedDest == BROADCAST_ID) snprintf(destLabel, sizeof(destLabel), "All");
      else snprintf(destLabel, sizeof(destLabel), "Node %d", _selectedDest);
      char c = CHARSET[_composeCharIndex];
      bool isDel = (c == '<');
      bool isSend = (c == '#');
      _display->drawCompose(destLabel, _composeBuffer, c, isDel, isSend);
      break;
    }
    case SCR_SENDING:
      _display->drawSending(_animFrame);
      break;
    case SCR_SENT: {
      char timeLabel[16] = "";
      if (_clock->isSet()) formatClock(_clock->nowEpoch(), timeLabel, sizeof(timeLabel));
      _display->drawSentConfirm(timeLabel);
      break;
    }
    case SCR_RECEIVING: {
      char sentLabel[24];
      formatTimeLabel("Sent: ", _currentDisplayMsg.senderEpoch, _currentDisplayMsg.receivedAt, sentLabel, sizeof(sentLabel));
      _display->drawReceiving(_currentDisplayFromLabel, _currentDisplayMsg.text,
                               _currentDisplayMsg.rssi, _msgQueue.count(), sentLabel);
      break;
    }
    case SCR_INBOX: {
      static char rows[20][28];
      static const char* rowPtrs[20];
      int count = _chat->inboxCount();
      for (int i = 0; i < count; i++) {
        const InboxEntry& e = _chat->inboxEntry(i);
        char timeTag[10];
        if (e.senderEpoch != 0) formatClock(e.senderEpoch, timeTag, sizeof(timeTag));
        else snprintf(timeTag, sizeof(timeTag), "%lus", (millis() - e.timestamp) / 1000);
        snprintf(rows[i], sizeof(rows[i]), "N%d %.9s %s", e.nodeId, e.text, timeTag);
        rowPtrs[i] = rows[i];
      }
      char title[16]; snprintf(title, sizeof(title), "INBOX (%d)", count);
      _display->drawList(title, neighbors, _batteryPct, rowPtrs, count, _inboxIndex, clockStr);
      break;
    }
    case SCR_OUTBOX: {
      // Transmitter-side status dashboard: every message *we* sent,
      // newest first, each tagged with its live S/D/R status so the
      // sender can see sent/delivered/read at a glance -- immediately
      // for the row itself (added the instant send() is called) and
      // live-updated in place as acks arrive.
      // Row buffer is wide enough for text + status + relay path
      // ("N4 hello world.. [D] 1>3>4"); drawList clips to screen width.
      static char rows[MAX_OUTBOX][40];
      static const char* rowPtrs[MAX_OUTBOX];
      int count = _chat->outboxCount();
      for (int i = 0; i < count; i++) {
        const OutboxEntry& e = _chat->outboxEntry(i);
        char destTag[6];
        if (e.destination == BROADCAST_ID) snprintf(destTag, sizeof(destTag), "ALL");
        else snprintf(destTag, sizeof(destTag), "N%d", e.destination);
        const char* statusTag = (e.status == MSG_READ) ? "R" : (e.status == MSG_DELIVERED) ? "D" : "S";

        // Relay Process Visibility: once the delivered ack reports a
        // real path, show it ("1>3>4"). Until then, if the destination
        // wasn't a direct neighbor at send time, show "..." so the
        // sender knows the message is being relayed through the mesh
        // right now rather than assuming it's stuck.
        char pathTag[20] = "";
        if (e.hopCount > 0) {
          char rawPath[16];
          formatPathLabel(e.destination, e.hopCount, e.path, rawPath, sizeof(rawPath));
          snprintf(pathTag, sizeof(pathTag), " %s", rawPath);
        } else if (e.expectsRelay && e.status == MSG_SENT) {
          snprintf(pathTag, sizeof(pathTag), " ...");
        }

        snprintf(rows[i], sizeof(rows[i]), "%s %.12s [%s]%s", destTag, e.text, statusTag, pathTag);
        rowPtrs[i] = rows[i];
      }
      char title[16]; snprintf(title, sizeof(title), "OUTBOX (%d)", count);
      _display->drawList(title, neighbors, _batteryPct, rowPtrs, count, _outboxIndex, clockStr);
      break;
    }
    case SCR_BROADCAST_CHANNEL: {
      // A shared group-chat-style thread: our own broadcasts plus
      // everyone else's, in the order they happened. Reuses drawChat's
      // outgoing/incoming layout -- outgoing rows here just mean "sent
      // by us to everyone", not "sent to one specific peer".
      static char lines[MAX_BROADCAST_LOG][40];
      static bool outFlags[MAX_BROADCAST_LOG];
      static MsgStatus statuses[MAX_BROADCAST_LOG];
      static const char* linePtrs[MAX_BROADCAST_LOG];
      int count = _chat->broadcastLogCount();
      for (int i = 0; i < count; i++) {
        const BroadcastLogEntry& m = _chat->broadcastLogEntry(i);
        char timeTag[10];
        if (m.epoch != 0) formatClock(m.epoch, timeTag, sizeof(timeTag));
        else snprintf(timeTag, sizeof(timeTag), "%lus", (millis() - m.timestamp) / 1000);
        if (m.outgoing) snprintf(lines[i], sizeof(lines[i]), "%s (%s)", m.text, timeTag);
        else snprintf(lines[i], sizeof(lines[i]), "N%d:%s (%s)", m.peerNode, m.text, timeTag);
        linePtrs[i] = lines[i];
        outFlags[i] = m.outgoing;
        statuses[i] = MSG_SENT;   // broadcasts never carry per-recipient ticks (see design note above)
      }
      const int visible = 4;
      int maxOffset = count > visible ? count - visible : 0;
      if (_broadcastScrollOffset > maxOffset) _broadcastScrollOffset = maxOffset;
      char title[20]; snprintf(title, sizeof(title), "BROADCAST (%d)", count);
      _display->drawChat(title, neighbors, _batteryPct, linePtrs, outFlags, statuses, count, _broadcastScrollOffset, visible, clockStr);
      break;
    }
    case SCR_MESSAGE_DETAIL: {
      const InboxEntry& e = _chat->inboxEntry(_selectedInboxEntry);
      char nodeLabel[12]; snprintf(nodeLabel, sizeof(nodeLabel), "Node %d", e.nodeId);
      char sentLabel[24], recvLabel[24];
      formatTimeLabel("Sent: ", e.senderEpoch, e.timestamp, sentLabel, sizeof(sentLabel));
      formatTimeLabel("Recv: ", e.localEpoch, e.timestamp, recvLabel, sizeof(recvLabel));
      _display->drawMessageDetail("MESSAGE", nodeLabel, e.text, sentLabel, recvLabel, e.rssi);
      break;
    }
    case SCR_PREV_CHATS: {
      static const char* names[4] = { "Node 1", "Node 2", "Node 3", "Node 4" };
      static const char* list[4];
      int count = 0;
      for (uint8_t id = 1; id <= 4; id++) {
        if (id == _nodeId) continue;
        list[count] = names[id - 1];
        count++;
      }
      _display->drawList("CHATS", neighbors, _batteryPct, list, count, _prevChatsIndex, clockStr);
      break;
    }
    case SCR_CHAT: {
      static char lines[10][40];
      static bool outFlags[10];
      static MsgStatus statuses[10];
      static const char* linePtrs[10];
      int count = _chat->chatMessageCount(_selectedChatNode);
      for (int i = 0; i < count; i++) {
        const ChatMessage& m = _chat->chatMessage(_selectedChatNode, i);
        char timeTag[10];
        if (m.sentEpoch != 0) formatClock(m.sentEpoch, timeTag, sizeof(timeTag));
        else snprintf(timeTag, sizeof(timeTag), "%lus", (millis() - m.timestamp) / 1000);
        snprintf(lines[i], sizeof(lines[i]), "%s (%s)", m.text, timeTag);
        linePtrs[i] = lines[i];
        outFlags[i] = m.outgoing;
        statuses[i] = m.status;
      }
      char title[16]; snprintf(title, sizeof(title), "CHAT N%d", _selectedChatNode);
      const int visible = 4;
      int maxOffset = count > visible ? count - visible : 0;
      if (_chatScrollOffset > maxOffset) _chatScrollOffset = maxOffset;
      _display->drawChat(title, neighbors, _batteryPct, linePtrs, outFlags, statuses, count, _chatScrollOffset, visible, clockStr);
      break;
    }
    case SCR_SETTINGS: {
      char title[20]; snprintf(title, sizeof(title), "N%d  SETTINGS", _nodeId);
      _display->drawList(title, neighbors, _batteryPct, SETTINGS_ITEMS, 6, _settingsIndex, clockStr);
      break;
    }
    case SCR_INFO: {
      const char* ptrs[4];
      for (int i = 0; i < _infoLineCount; i++) ptrs[i] = _infoLines[i];
      _display->drawInfoScreen(_infoTitle, ptrs, _infoLineCount);
      break;
    }
    case SCR_OFFLINE_WARNING: {
      char destLabel[12];
      if (_selectedDest == BROADCAST_ID) snprintf(destLabel, sizeof(destLabel), "Broadcast");
      else snprintf(destLabel, sizeof(destLabel), "Node %d", _selectedDest);
      _display->drawOfflineWarning(destLabel);
      break;
    }
    case SCR_RECONNECT_POPUP: {
      _display->drawReconnectPopup(_reconnectedNodeId);
      break;
    }
    case SCR_SET_CLOCK: {
      _display->drawSetClock(_setClockHour, _setClockMinute, _setClockField);
      break;
    }
  }
}

void Menu::updateSplash()
{
  bool timedOut = (millis() - _screenEnteredAt) > SPLASH_DURATION_MS;
  if (timedOut || _btnAction == BTN_CLICK) {
    goTo(SCR_MAIN_MENU);
  }
}

void Menu::updateMainMenu()
{
  if (_delta != 0) {
    _mainMenuIndex += (_delta > 0) ? 1 : -1;
    if (_mainMenuIndex < 0) _mainMenuIndex = MAIN_MENU_COUNT - 1;
    if (_mainMenuIndex > MAIN_MENU_COUNT - 1) _mainMenuIndex = 0;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK) {
    switch (_mainMenuIndex) {
      case 0:
        _selectedDest = BROADCAST_ID;
        _emergencyIndex = 0;
        _emergencyFromQuickAccess = true;
        goTo(SCR_EMERGENCY);
        break;
      case 1: buildDeviceList(); goTo(SCR_DEVICE_SELECT); break;
      case 2: _broadcastIndex = 0; _broadcastScrollOffset = 0; goTo(SCR_BROADCAST_CHANNEL); break;
      case 3: _inboxIndex = 0; goTo(SCR_INBOX); break;
      case 4: _outboxIndex = 0; goTo(SCR_OUTBOX); break;
      case 5: _prevChatsIndex = 0; goTo(SCR_PREV_CHATS); break;
      case 6: _settingsIndex = 0; goTo(SCR_SETTINGS); break;
    }
  }
}

void Menu::updateDeviceSelect()
{
  if (_delta != 0) {
    _deviceIndex += (_delta > 0) ? 1 : -1;
    if (_deviceIndex < 0) _deviceIndex = _deviceCount - 1;
    if (_deviceIndex >= _deviceCount) _deviceIndex = 0;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK && _deviceCount > 0) {
    _selectedDest = _deviceIds[_deviceIndex];
    _sendMenuIndex = 0;
    goTo(SCR_SEND_MENU);
  }
}

void Menu::updateSendMenu()
{
  if (_delta != 0) {
    _sendMenuIndex += (_delta > 0) ? 1 : -1;
    if (_sendMenuIndex < 0) _sendMenuIndex = 5;
    if (_sendMenuIndex > 5) _sendMenuIndex = 0;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK) {
    switch (_sendMenuIndex) {
      case 0:
        _emergencyIndex = 0;
        _emergencyFromQuickAccess = false;
        goTo(SCR_EMERGENCY);
        break;
      case 1:
        strncpy(_pendingText, "MEDICAL", 31); _pendingText[31] = '\0';
        beginSendFlow();
        break;
      case 2:
        strncpy(_pendingText, "FIRE", 31); _pendingText[31] = '\0';
        beginSendFlow();
        break;
      case 3:
        strncpy(_pendingText, "NEED BACKUP", 31); _pendingText[31] = '\0';
        beginSendFlow();
        break;
      case 4:
        strncpy(_pendingText, "HOSTAGE FOUND", 31); _pendingText[31] = '\0';
        beginSendFlow();
        break;
      case 5:
        _composeLen = 0; _composeBuffer[0] = '\0'; _composeCharIndex = 0;
        _composeReturnScreen = SCR_SEND_MENU;
        goTo(SCR_COMPOSE);
        break;
    }
  }
}

void Menu::updateEmergency()
{
  if (_delta != 0) {
    _emergencyIndex += (_delta > 0) ? 1 : -1;
    if (_emergencyIndex < 0) _emergencyIndex = 5;
    if (_emergencyIndex > 5) _emergencyIndex = 0;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK) {
    strncpy(_pendingText, EMERGENCY_ITEMS[_emergencyIndex], 31); _pendingText[31] = '\0';
    beginSendFlow();
  }
}

void Menu::updateCompose()
{
  if (_delta != 0) {
    _composeCharIndex += (_delta > 0) ? 1 : -1;
    if (_composeCharIndex < 0) _composeCharIndex = CHARSET_LEN - 1;
    if (_composeCharIndex >= CHARSET_LEN) _composeCharIndex = 0;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK) {
    char c = CHARSET[_composeCharIndex];
    if (c == '#') {
      strncpy(_pendingText, _composeBuffer, 31); _pendingText[31] = '\0';
      beginSendFlow();
    } else if (c == '<') {
      if (_composeLen > 0) {
        _composeLen--;
        _composeBuffer[_composeLen] = '\0';
      }
      _dirty = true;
    } else if (_composeLen < 31) {
      _composeBuffer[_composeLen++] = c;
      _composeBuffer[_composeLen] = '\0';
      _dirty = true;
    }
  }
}

bool Menu::sendCurrentPending()
{
  bool destOffline = (_selectedDest != BROADCAST_ID) && !_lora->isNeighborActive(_selectedDest);

  uint32_t epoch = _clock->nowEpoch();
  uint16_t seq = _lora->send(_selectedDest, _pendingText, epoch);

  // Feature: Transmitter Outbox -- every send, unicast or broadcast,
  // is logged the instant it happens so this node can immediately see
  // its own sent/delivered/read status without waiting on any ack.
  // destOffline doubles as "Relay Process Visibility" info: it's true
  // whenever the destination isn't a direct neighbor, i.e. this
  // message is expected to need at least one relay hop to arrive.
  _chat->addOutboxEntry(_selectedDest, _pendingText, seq, epoch, destOffline);

  if (_selectedDest != BROADCAST_ID) {
    // Per-peer chat thread with ticks: there IS a single recipient
    // here, so delivery/read tracking is meaningful.
    _chat->addChatMessage(_selectedDest, true, _pendingText, seq, epoch);
  } else {
    // Feature: Broadcast Channel -- record our own broadcast into the
    // shared thread too, alongside whatever the other nodes send.
    // Broadcasts intentionally never get per-recipient ticks: there's
    // no single "the" recipient to track delivery/read for.
    _chat->addBroadcastSent(_pendingText, seq, epoch);
  }
  return destOffline;
}

void Menu::beginSendFlow()
{
  bool offline = sendCurrentPending();
  if (offline) {
    goTo(SCR_OFFLINE_WARNING);
  } else {
    _animFrame = 0; _animTimer = millis();
    goTo(SCR_SENDING);
  }
}

void Menu::updateOfflineWarning()
{
  if (millis() - _screenEnteredAt > OFFLINE_WARN_MS) {
    _animFrame = 0; _animTimer = millis();
    goTo(SCR_SENDING);
  }
}

void Menu::updateReconnectPopup()
{
  if (millis() - _screenEnteredAt > RECONNECT_POPUP_MS) {
    goTo(_screenBeforePopup);
  }
}

void Menu::updateSending()
{
  unsigned long elapsed = millis() - _animTimer;

  int frame = (elapsed / 200) % 3 + 1;
  if (frame != _animFrame) {
    _animFrame = frame;
    _dirty = true;
  }

  if (elapsed > SEND_ANIM_MS) {
    goTo(SCR_SENT);
  }
}

void Menu::updateSent()
{
  if (millis() - _screenEnteredAt > SENT_CONFIRM_MS) {
    goTo(SCR_MAIN_MENU);
  }
}

void Menu::updateReceiving()
{
  bool dwellExpired = (millis() - _screenEnteredAt) > MSG_DWELL_MS;
  bool timedOutHard  = (millis() - _screenEnteredAt) > RECEIVING_TIMEOUT_MS;

  // NOTE: a natural dwell/hard timeout advances the popup but does NOT
  // send a read-ack -- only an explicit long-press dismiss (see
  // handleBack()) or deliberately opening the message elsewhere
  // (Inbox detail / Chat thread) counts as "read". This mirrors
  // WhatsApp: a notification banner appearing isn't the same as you
  // opening the conversation.
  if (dwellExpired || timedOutHard) {
    if (_msgQueue.pop(_currentDisplayMsg)) {
      snprintf(_currentDisplayFromLabel, sizeof(_currentDisplayFromLabel), "Node %d", _currentDisplayMsg.nodeId);
      _screenEnteredAt = millis();
      _dirty = true;
    } else {
      goTo(_screenBeforeIncoming);
    }
  }
}

void Menu::updateInbox()
{
  int count = _chat->inboxCount();
  if (_delta != 0 && count > 0) {
    _inboxIndex += (_delta > 0) ? 1 : -1;
    if (_inboxIndex < 0) _inboxIndex = count - 1;
    if (_inboxIndex >= count) _inboxIndex = 0;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK && count > 0) {
    _selectedInboxEntry = _inboxIndex;
    const InboxEntry& e = _chat->inboxEntry(_selectedInboxEntry);
    markMessageRead(e.nodeId, e.seq);   // deliberately opening a specific message = read
    goTo(SCR_MESSAGE_DETAIL);
  }
}

void Menu::updateMessageDetail() {}

void Menu::updatePrevChats()
{
  int count = 3;
  if (_delta != 0) {
    _prevChatsIndex += (_delta > 0) ? 1 : -1;
    if (_prevChatsIndex < 0) _prevChatsIndex = count - 1;
    if (_prevChatsIndex >= count) _prevChatsIndex = 0;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK) {
    int seen = 0;
    for (uint8_t id = 1; id <= 4; id++) {
      if (id == _nodeId) continue;
      if (seen == _prevChatsIndex) { _selectedChatNode = id; break; }
      seen++;
    }
    _chatScrollOffset = 0;

    // Opening a thread = reading everything currently in it. This
    // dedupes against re-sending acks via the cache in markMessageRead().
    int count2 = _chat->chatMessageCount(_selectedChatNode);
    for (int i = 0; i < count2; i++) {
      const ChatMessage& m = _chat->chatMessage(_selectedChatNode, i);
      if (!m.outgoing) markMessageRead(_selectedChatNode, m.seq);
    }

    goTo(SCR_CHAT);
  }
}

void Menu::updateChat()
{
  int count = _chat->chatMessageCount(_selectedChatNode);
  const int visible = 4;
  if (_delta != 0 && count > visible) {
    _chatScrollOffset += (_delta > 0) ? 1 : -1;
    int maxOffset = count - visible;
    if (_chatScrollOffset < 0) _chatScrollOffset = 0;
    if (_chatScrollOffset > maxOffset) _chatScrollOffset = maxOffset;
    _dirty = true;
  }
}

void Menu::updateSettings()
{
  if (_delta != 0) {
    _settingsIndex += (_delta > 0) ? 1 : -1;
    if (_settingsIndex < 0) _settingsIndex = 5;
    if (_settingsIndex > 5) _settingsIndex = 0;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK) {
    char buf[24];
    switch (_settingsIndex) {
      case 0:
        snprintf(buf, sizeof(buf), "Current: Node %d", _nodeId);
        showInfo("Node ID", buf, "Set via config.h", "before flashing");
        break;
      case 1:
        showInfo("Brightness", "Not wired to a", "control yet --", "hook up contrast");
        break;
      case 2:
        showInfo("LoRa Frequency", "433 MHz", "Set in config.h");
        break;
      case 3: {
        uint32_t nowE = _clock->nowEpoch();
        uint32_t sod = nowE % 86400UL;
        _setClockHour = sod / 3600;
        _setClockMinute = (sod % 3600) / 60;
        _setClockField = 0;
        goTo(SCR_SET_CLOCK);
        break;
      }
      case 4:
        _chat->clearAll();
        showInfo("Factory Reset", "Chat history", "cleared.");
        break;
      case 5:
        showInfo("About", "Self-Healing Mesh", "w/ Read Receipts", "v3.1");
        break;
    }
  }
}

void Menu::updateInfo() {}

void Menu::updateSetClock()
{
  if (_delta != 0) {
    if (_setClockField == 0) {
      _setClockHour = ((_setClockHour + _delta) % 24 + 24) % 24;
    } else {
      _setClockMinute = ((_setClockMinute + _delta) % 60 + 60) % 60;
    }
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK) {
    if (_setClockField == 0) {
      _setClockField = 1;
      _dirty = true;
    } else {
      uint32_t epoch = (uint32_t)_setClockHour * 3600UL + (uint32_t)_setClockMinute * 60UL;
      _clock->setEpoch(epoch);
      showInfo("Clock Set", "Time updated.", "Resets if power", "is lost (no RTC).");
    }
  }
}

void Menu::updateOutbox()
{
  int count = _chat->outboxCount();
  if (_delta != 0 && count > 0) {
    _outboxIndex += (_delta > 0) ? 1 : -1;
    if (_outboxIndex < 0) _outboxIndex = count - 1;
    if (_outboxIndex >= count) _outboxIndex = 0;
    _dirty = true;
  }
  // Browsing the Outbox is read-only by design -- it's a status
  // dashboard for messages *we* sent, not an inbox to mark read.
}

void Menu::updateBroadcastChannel()
{
  int count = _chat->broadcastLogCount();
  const int visible = 4;
  if (_delta != 0 && count > visible) {
    _broadcastScrollOffset += (_delta > 0) ? 1 : -1;
    int maxOffset = count - visible;
    if (_broadcastScrollOffset < 0) _broadcastScrollOffset = 0;
    if (_broadcastScrollOffset > maxOffset) _broadcastScrollOffset = maxOffset;
    _dirty = true;
  }
  if (_btnAction == BTN_CLICK) {
    // Quick-compose straight into the broadcast channel -- no need to
    // go via Device Select to reach "Broadcast" as a destination.
    _composeLen = 0; _composeBuffer[0] = '\0'; _composeCharIndex = 0;
    _selectedDest = BROADCAST_ID;
    _composeReturnScreen = SCR_BROADCAST_CHANNEL;
    goTo(SCR_COMPOSE);
  }
}

// =================================================================
// MAIN SKETCH
//
// RELAY NOTE: when RELAY_NODE_MODE is 1, this device skips the OLED,
// rotary, and Menu entirely and just runs the radio: LoRaManager.poll()
// already forwards any packet not addressed to this node -- including
// the new PKT_ACK_DELIVERED/PKT_ACK_READ types -- so a headless relay
// needs no new routing code -- only a smaller setup()/loop() that
// leaves the UI peripherals uninitialized and drives a status LED
// instead of a screen.
// =================================================================

DisplayManager displayMgr;
LoRaManager loraMgr;
Rotary rotary;
Chat chat;
Clock wallClock;
Menu menu;

unsigned long lastHelloSentAt = 0;
unsigned long lastPruneCheckAt = 0;
const unsigned long HELLO_INTERVAL_MS = 5000;
const unsigned long PRUNE_CHECK_INTERVAL_MS = 1000;

#if RELAY_NODE_MODE
unsigned long relayLedOffAt = 0;   // 0 == LED currently off / no pending turn-off
#endif

void setup()
{
  Serial.begin(115200);

#if RELAY_NODE_MODE
  pinMode(RELAY_STATUS_LED, OUTPUT);
  digitalWrite(RELAY_STATUS_LED, LOW);
#else
  if (!displayMgr.begin(OLED_SDA, OLED_SCL, SCREEN_WIDTH, SCREEN_HEIGHT)) {
    Serial.println("OLED init failed -- check wiring/address");
    while (true);
  }

  rotary.begin(ENC_CLK, ENC_DT, ENC_SW);
  wallClock.begin();
#endif

  if (!loraMgr.begin(NODE_ID, LORA_NSS, LORA_RST, LORA_DIO0, LORA_FREQUENCY)) {
    Serial.println("LoRa init failed -- check wiring");
    while (true);
  }

#if !RELAY_NODE_MODE
  chat.begin();
  menu.begin(&displayMgr, &loraMgr, &rotary, &chat, &wallClock, NODE_ID);
#endif

  // --- FIX: jitter the HELLO schedule ------------------------------
  // All nodes boot roughly together and would otherwise send their
  // first HELLO (and every 5s HELLO after) in near lockstep, causing
  // recurring collisions on shared air time. A random per-node offset
  // spreads transmissions out. NODE_ID is folded into the seed so
  // nodes flashed at the same instant still get different offsets.
  // This applies identically to a relay -- it sends HELLOs too, so it
  // shows up in every handheld's online/offline neighbor tracking.
  randomSeed(analogRead(0) + NODE_ID);
  lastHelloSentAt = millis() - random(0, HELLO_INTERVAL_MS);
  // -------------------------------------------------------------------

  Serial.println(RELAY_NODE_MODE ? "Mesh relay ready (headless)." : "Mesh radio ready.");
}

// Nothing in this loop ever calls delay(). LoRa.parsePacket() is
// non-blocking (returns 0 immediately if nothing arrived), and every
// timed UI behavior (send animation, message dwell, popups) is driven
// by comparing millis() snapshots rather than blocking sleeps. The
// OLED itself is only redrawn when Menu sets _dirty = true (on an
// actual state change), so the SPI/I2C bus isn't hammered every loop
// iteration -- this is what keeps radio polling responsive even while
// the screen is "animating". The relay-mode LED blink below follows
// the same non-blocking pattern: it's turned on immediately and
// turned off on a later loop iteration once its timer elapses, never
// via delay(). Delivery acks fire synchronously inside loraMgr.poll()
// (radio-layer, no UI dependency), so they work identically whether
// this is a handheld or a headless relay-adjacent node.
void loop()
{
  loraMgr.poll();

  unsigned long now = millis();
  if (now - lastHelloSentAt >= HELLO_INTERVAL_MS) {
    lastHelloSentAt = now;
    loraMgr.sendHello();
  }
  if (now - lastPruneCheckAt >= PRUNE_CHECK_INTERVAL_MS) {
    lastPruneCheckAt = now;
    loraMgr.pruneNeighbors();
  }

#if RELAY_NODE_MODE
  if (loraMgr.consumeRelayActivityFlag()) {
    digitalWrite(RELAY_STATUS_LED, HIGH);
    relayLedOffAt = now + RELAY_LED_ON_MS;
  }
  if (relayLedOffAt != 0 && now >= relayLedOffAt) {
    digitalWrite(RELAY_STATUS_LED, LOW);
    relayLedOffAt = 0;
  }

#else
  menu.update();
#endif
}
