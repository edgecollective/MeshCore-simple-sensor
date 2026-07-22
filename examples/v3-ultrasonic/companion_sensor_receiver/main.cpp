#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <RTClib.h>
#include <target.h>

#ifdef ESP32
  #include <WiFi.h>
  #include <HTTPClient.h>
#endif

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "companion_sensor_receiver v3-ultrasonic (build: May 2026) [distance_meters → Bayou; same receiver as v2/v3, pair label only]"

#ifndef LORA_FREQ
  #define LORA_FREQ   910.525
#endif
#ifndef LORA_BW
  #define LORA_BW     62.5
#endif
#ifndef LORA_SF
  #define LORA_SF     7
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS   32
#endif

#ifndef ADVERT_NAME
  #define ADVERT_NAME  "Receiver"
#endif

#ifndef BAYOU_BASE_URL
  #define BAYOU_BASE_URL "https://bayou.pvos.org/data/"
#endif

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

/* ---------------------------------- DISPLAY PAGES ------------------------------------- */

enum DisplayPage {
  PAGE_STATUS = 0,   // WiFi, Bayou, last reading
  PAGE_ADVERT,       // Send advert -- long press
  PAGE_COUNT
};

#define DISPLAY_REFRESH_MS    500
#define DISPLAY_AUTO_OFF_MS   30000
#define ALERT_DURATION_MS     1500

/* -------------------------------------------------------------------------------------- */

// Persisted receiver config
struct ReceiverPrefs {
  char node_name[32];
  char wifi_ssid[48];
  char wifi_password[48];
  char bayou_public_key[64];
  char bayou_private_key[64];
};

class ReceiverMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM* _fs;
  uint32_t expected_ack_crc;
  unsigned long last_msg_sent;
  char command[512];
  uint8_t tmp_buf[256];
  char hex_buf[512];

  ReceiverPrefs _prefs;

  // Last received sensor data
  float last_recv_dist_m;          // meters, parsed from "dist=" in payload
  float last_recv_batt;
  uint16_t last_recv_node_id;
  uint8_t last_recv_hops;          // hops as measured by us at arrival (aux_1)
  uint8_t last_recv_fwd_hops;      // v2: hops as claimed by sender's cached out_path (aux_2)
  char last_recv_fwd_path[48];     // v2: sender's cached path as "aa.bb.cc" or "none"
  bool last_recv_direct;
  char last_recv_from[32];
  bool has_recv;
  uint32_t last_recv_at_millis;    // millis() when last sensor packet arrived (0 = none yet)
  uint16_t recv_count;             // total sensor packets received since boot

  // v2: small LRU of recently-handled direct messages, to detect "sender retried
  // because our last ACK didn't land". Keyed by (sender pub_key prefix, sender timestamp).
  static constexpr uint8_t DUP_CACHE_SIZE = 8;
  struct DupEntry {
    uint8_t sender_prefix[4];
    uint32_t sender_timestamp;
    unsigned long seen_millis;
  };
  DupEntry dup_cache[DUP_CACHE_SIZE];
  uint8_t dup_cache_next;  // round-robin slot

  // Bayou posting state
  bool post_pending;
  unsigned long next_post_retry_at;
  unsigned long last_post_ok_at;
  char last_post_status[32];
  char wifi_status[20];

  // Periodic advert
  unsigned long next_advert_millis;

  // Auto-favourite on import: remember the pub_key of the just-imported advert
  uint8_t pending_fav_pub_key[PUB_KEY_SIZE];
  bool has_pending_fav;

  void loadContacts() {
    if (_fs->exists("/contacts")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/contacts", "r");
    #else
      File file = _fs->open("/contacts");
    #endif
      if (file) {
        bool full = false;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;
          uint32_t reserved;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read((uint8_t *) &c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read((uint8_t *) &reserved, 4) == 4);
          success = success && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
          success = success && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          c.gps_lat = c.gps_lon = 0;

          if (!success) break;

          c.id = mesh::Identity(pub_key);
          c.lastmod = 0;
          if (!addContact(c)) full = true;
        }
        file.close();
      }
    }
  }

  void saveContacts() {
  #if defined(NRF52_PLATFORM)
    _fs->remove("/contacts");
    File file = _fs->open("/contacts", FILE_O_WRITE);
  #elif defined(RP2040_PLATFORM)
    File file = _fs->open("/contacts", "w");
  #else
    File file = _fs->open("/contacts", "w", true);
  #endif
    if (file) {
      ContactsIterator iter;
      ContactInfo c;
      uint8_t unused = 0;
      uint32_t reserved = 0;

      while (iter.hasNext(this, c)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write((uint8_t *) &c.name, 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write((uint8_t *) &reserved, 4) == 4);
        success = success && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
        success = success && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);

        if (!success) break;
      }
      file.close();
    }
  }

  void loadPrefs() {
    if (_fs->exists("/recv_prefs")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/recv_prefs", "r");
    #else
      File file = _fs->open("/recv_prefs");
    #endif
      if (file) {
        ReceiverPrefs p;
        if (file.read((uint8_t*)&p, sizeof(p)) == sizeof(p)) {
          memcpy(&_prefs, &p, sizeof(_prefs));
        }
        file.close();
      }
    }
  }

  void savePrefs() {
  #if defined(NRF52_PLATFORM)
    _fs->remove("/recv_prefs");
    File file = _fs->open("/recv_prefs", FILE_O_WRITE);
  #elif defined(RP2040_PLATFORM)
    File file = _fs->open("/recv_prefs", "w");
  #else
    File file = _fs->open("/recv_prefs", "w", true);
  #endif
    if (file) {
      file.write((const uint8_t*)&_prefs, sizeof(_prefs));
      file.close();
    }
  }

  void importCard(const char* cmd) {
    while (*cmd == ' ') cmd++;
    if (memcmp(cmd, "meshcore://", 11) == 0) {
      cmd += 11;
      char *ep = strchr(cmd, 0);
      while (ep > cmd) {
        ep--;
        if (mesh::Utils::isHexChar(*ep)) break;
        *ep = 0;
      }
      int len = strlen(cmd);
      Serial.printf("   hex len=%d\n", len);
      if (len % 2 == 0) {
        len >>= 1;
        if (mesh::Utils::fromHex(tmp_buf, len, cmd)) {
          if (importContact(tmp_buf, len)) {
            // Parse pub_key from the advert packet bytes so we can auto-favourite
            // the specific contact that this import adds (not some incidental OTA advert).
            uint8_t off = 1;  // header
            uint8_t route = tmp_buf[0] & 0x03;
            if (route == 0x00 || route == 0x03) off += 4;  // transport codes
            uint8_t plen = tmp_buf[off++];
            uint8_t hash_count = plen & 63;
            uint8_t hash_size = (plen >> 6) + 1;
            off += hash_count * hash_size;
            if ((int)(off + PUB_KEY_SIZE) <= len) {
              memcpy(pending_fav_pub_key, &tmp_buf[off], PUB_KEY_SIZE);
              has_pending_fav = true;
            }
            Serial.printf("   Advert queued. Contacts before: %d/%d\n", getNumContacts(), MAX_CONTACTS);
            Serial.println("   Will auto-favourite on add (protected from eviction).");
          } else {
            Serial.println("   error: importContact failed (bad packet).");
          }
          return;
        }
      }
    }
    Serial.println("   error: invalid format");
  }

  // Parse "[SENSOR] node_id=1 dist=1.234m batt=3.85V [fwd_hops=N fwd_path=aa.bb.cc]".
  // v2-ultrasonic: extracts distance (meters) from "dist=" instead of temperature.
  // fwd_hops (uint8) and fwd_path (string) are unchanged; missing fwd_hops → 0xFF.
  bool parseSensorMessage(const char* text, float& dist_m, float& batt, uint16_t& nid,
                          uint8_t& fwd_hops, char* fwd_path, size_t fwd_path_sz) {
    if (memcmp(text, "[SENSOR]", 8) != 0) return false;

    const char* np = strstr(text, "node_id=");
    const char* dp = strstr(text, "dist=");
    const char* bp = strstr(text, "batt=");
    const char* fp = strstr(text, "fwd_hops=");
    const char* pp = strstr(text, "fwd_path=");

    nid = np ? (uint16_t)atoi(np + 8) : 0;
    dist_m = dp ? (float)atof(dp + 5) : 0.0f;
    batt = bp ? (float)atof(bp + 5) : 0.0f;
    fwd_hops = fp ? (uint8_t)atoi(fp + 9) : 0xFF;

    if (pp && fwd_path && fwd_path_sz > 0) {
      const char* src = pp + 9;  // skip "fwd_path="
      size_t i = 0;
      while (i < fwd_path_sz - 1 && src[i] && src[i] != ' ' && src[i] != '\n') {
        fwd_path[i] = src[i];
        i++;
      }
      fwd_path[i] = 0;
    } else if (fwd_path && fwd_path_sz > 0) {
      StrHelper::strncpy(fwd_path, "none", fwd_path_sz);
    }

    return (dp != NULL);  // at minimum we need distance
  }

  // v2: duplicate detection. Returns true if this (sender, timestamp) was seen
  // within the last 60 seconds (same message being retried).
  bool checkAndRecordDuplicate(const uint8_t* sender_pub, uint32_t sender_timestamp) {
    unsigned long now = millis();
    for (uint8_t i = 0; i < DUP_CACHE_SIZE; i++) {
      if (memcmp(dup_cache[i].sender_prefix, sender_pub, 4) == 0 &&
          dup_cache[i].sender_timestamp == sender_timestamp &&
          (now - dup_cache[i].seen_millis) < 60UL * 1000UL) {
        dup_cache[i].seen_millis = now;  // refresh
        return true;
      }
    }
    // Not a duplicate -- record in round-robin slot.
    memcpy(dup_cache[dup_cache_next].sender_prefix, sender_pub, 4);
    dup_cache[dup_cache_next].sender_timestamp = sender_timestamp;
    dup_cache[dup_cache_next].seen_millis = now;
    dup_cache_next = (dup_cache_next + 1) % DUP_CACHE_SIZE;
    return false;
  }

#ifdef ESP32
  bool ensureWifiConnected() {
    if (_prefs.wifi_ssid[0] == 0) {
      StrHelper::strncpy(wifi_status, "no SSID", sizeof(wifi_status));
      return false;
    }

    if (WiFi.status() == WL_CONNECTED) {
      StrHelper::strncpy(wifi_status, "connected", sizeof(wifi_status));
      return true;
    }

    // Try to reconnect
    WiFi.disconnect(false, false);
    WiFi.begin(_prefs.wifi_ssid, _prefs.wifi_password);
    StrHelper::strncpy(wifi_status, "connecting", sizeof(wifi_status));

    // Wait briefly for connection
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 5000) {
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      StrHelper::strncpy(wifi_status, "connected", sizeof(wifi_status));
      Serial.printf("   WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }

    StrHelper::strncpy(wifi_status, "failed", sizeof(wifi_status));
    return false;
  }

  bool postToBayou(float dist_m, float batt, const char* from_name, uint16_t nid,
                   uint8_t hops, uint8_t fwd_hops, const char* fwd_path, bool direct) {
    if (_prefs.bayou_public_key[0] == 0 || _prefs.bayou_private_key[0] == 0) {
      StrHelper::strncpy(last_post_status, "no bayou keys", sizeof(last_post_status));
      Serial.println("   Bayou keys not set, skipping post");
      return false;
    }

    if (!ensureWifiConnected()) {
      StrHelper::strncpy(last_post_status, "no wifi", sizeof(last_post_status));
      return false;
    }

    HTTPClient http;
    String url = String(BAYOU_BASE_URL) + String(_prefs.bayou_public_key);

    // v2: aux_1 = hops we measured at arrival; aux_2 = hops as claimed by sender
    // (255 = sender didn't include field; 255 also = sender's path was unknown).
    // log = sender's cached hash-byte path + route flag as a structured string;
    // Bayou's measurements table has `log VARCHAR(255)` for exactly this kind of
    // free-form metadata. Keep the format key=value so it parses easily downstream.
    char body[420];
    snprintf(body, sizeof(body),
             "{\"private_key\":\"%s\",\"node_id\":%u,\"distance_meters\":%.3f,\"battery_volts\":%.3f,"
             "\"aux_1\":%u,\"aux_2\":%u,\"log\":\"path=%s route=%s\",\"source\":\"%s\"}",
             _prefs.bayou_private_key, (unsigned)nid, (double)dist_m, (double)batt,
             (unsigned)hops, (unsigned)fwd_hops, fwd_path,
             direct ? "DIRECT" : "FLOOD", from_name);

    Serial.printf("   Posting to Bayou: %s\n", url.c_str());

    if (!http.begin(url)) {
      StrHelper::strncpy(last_post_status, "http begin fail", sizeof(last_post_status));
      return false;
    }

    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    http.addHeader("Content-Type", "application/json");
    int http_code = http.POST(reinterpret_cast<uint8_t*>(body), strlen(body));
    String response = (http_code > 0) ? http.getString() : String();
    http.end();

    if (http_code >= 200 && http_code < 300) {
      StrHelper::strncpy(last_post_status, "ok", sizeof(last_post_status));
      last_post_ok_at = millis();
      Serial.printf("   Bayou post OK (%d): %s\n", http_code, response.c_str());
      return true;
    }

    char status[32];
    snprintf(status, sizeof(status), "fail %d", http_code);
    StrHelper::strncpy(last_post_status, status, sizeof(last_post_status));
    Serial.printf("   Bayou post FAILED (%d): %s\n", http_code, response.c_str());
    return false;
  }
#endif

  void handleSensorData(const char* from_name, float dist_m, float batt, uint16_t nid,
                        uint8_t hops, uint8_t fwd_hops, const char* fwd_path, bool direct) {
    last_recv_dist_m = dist_m;
    last_recv_batt = batt;
    last_recv_node_id = nid;
    last_recv_hops = hops;
    last_recv_fwd_hops = fwd_hops;
    StrHelper::strncpy(last_recv_fwd_path, fwd_path, sizeof(last_recv_fwd_path));
    last_recv_direct = direct;
    StrHelper::strncpy(last_recv_from, from_name, sizeof(last_recv_from));
    has_recv = true;
    last_recv_at_millis = millis();
    recv_count++;

    Serial.printf("   [SENSOR DATA] from=%s node_id=%u dist=%.3fm batt=%.2fV "
                  "radio_hops=%u fwd_hops=%u fwd_path=%s route=%s\n",
                  from_name, (unsigned)nid, dist_m, batt,
                  (unsigned)hops, (unsigned)fwd_hops, fwd_path,
                  direct ? "DIRECT" : "FLOOD");

  #ifdef ESP32
    post_pending = true;
    next_post_retry_at = millis();
  #endif
  }

protected:
  float getAirtimeBudgetFactor() const override { return 1.0; }
  int calcRxDelay(float score, uint32_t air_time) const override { return 0; }
  bool allowPacketForward(const mesh::Packet* packet) override { return true; }

  // Auto-evict oldest non-favourite contact when list is full
  bool shouldOverwriteWhenFull() const override { return true; }

  void onContactOverwrite(const uint8_t* pub_key) override {
    Serial.print("   (evicted oldest non-favourite contact: ");
    mesh::Utils::printHex(Serial, pub_key, 4);
    Serial.println(")");
  }

  void onContactsFull() override {
    Serial.printf("   WARNING: contacts list is full (%d/%d). Use 'purge' to clear.\n", getNumContacts(), MAX_CONTACTS);
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    Serial.printf("ADVERT from -> %s%s\n", contact.name, is_new ? " [NEW]" : "");
    Serial.print("   key: "); mesh::Utils::printHex(Serial, contact.id.pub_key, 4); Serial.println();

    if (has_pending_fav &&
        memcmp(contact.id.pub_key, pending_fav_pub_key, PUB_KEY_SIZE) == 0) {
      contact.flags |= 0x01;  // mark as favourite
      has_pending_fav = false;
      Serial.printf("   auto-favourited %s (protected from eviction)\n", contact.name);
    }
    saveContacts();
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    // v2: log path bytes, not just length.
    Serial.printf("PATH to: %s, path_len=%d, hashes=", contact.name, (uint32_t) contact.out_path_len);
    for (uint8_t h = 0; h < contact.out_path_len; h++) {
      Serial.printf("%02x%s", contact.out_path[h], h < contact.out_path_len - 1 ? "." : "");
    }
    Serial.println();
    saveContacts();
  }

  ContactInfo* processAck(const uint8_t *data) override {
    if (memcmp(data, &expected_ack_crc, 4) == 0) {
      expected_ack_crc = 0;
      return NULL;
    }
    return NULL;
  }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    Serial.printf("(%s) MSG from %s:\n", pkt->isRouteDirect() ? "DIRECT" : "FLOOD", from.name);
    Serial.printf("   %s\n", text);

    // v2: if this is a DIRECT arrival of a message we already handled within 60s,
    // our previous ACK likely didn't land. Invalidate our cached reverse out_path
    // so the ACK the base class is about to send (via sendAckTo after this callback
    // returns) goes out via FLOOD instead of our stale direct path. The flood also
    // causes the sender to update its forward out_path to a fresh route.
    bool is_duplicate = checkAndRecordDuplicate(from.id.pub_key, sender_timestamp);
    if (is_duplicate && pkt->isRouteDirect()) {
      ContactInfo* c = lookupContactByPubKey(from.id.pub_key, PUB_KEY_SIZE);
      if (c && c->out_path_len != OUT_PATH_UNKNOWN) {
        Serial.printf("   DUP: duplicate of recent direct msg from %s -- invalidating reverse path "
                      "(was len=%d). ACK will FLOOD this time.\n",
                      from.name, (int)c->out_path_len);
        c->out_path_len = OUT_PATH_UNKNOWN;
        saveContacts();
      }
    }

    // Sensor data extraction (unchanged logic for hops; new fwd_hops + fwd_path in payload).
    float dist_m, batt;
    uint16_t nid;
    uint8_t fwd_hops;
    char fwd_path[48];
    if (parseSensorMessage(text, dist_m, batt, nid, fwd_hops, fwd_path, sizeof(fwd_path))) {
      // For FLOOD: the path accumulates as the packet travels, so getPathHashCount() is
      // the hop count at arrival. For DIRECT: each repeater strips its hash before
      // forwarding (Mesh.cpp: removeSelfFromPath), so the arrived path is always 0.
      // Fall back to the stored outbound path length (symmetric route assumption).
      uint8_t hops;
      if (pkt->isRouteDirect() && from.out_path_len != OUT_PATH_UNKNOWN) {
        hops = from.out_path_len;
      } else {
        hops = pkt->getPathHashCount();
      }
      handleSensorData(from.name, dist_m, batt, nid, hops, fwd_hops, fwd_path,
                       pkt->isRouteDirect());
    }
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {}
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override {}
  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override {}
  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override { return 0; }
  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {}

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    uint8_t path_hash_count = path_len & 63;
    return SEND_TIMEOUT_BASE_MILLIS +
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_hash_count + 1));
  }
  void onSendTimeout() override { Serial.println("   ERROR: timed out, no ACK."); }

public:
  ReceiverMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    command[0] = 0;
    expected_ack_crc = 0;
    last_msg_sent = 0;
    memset(&_prefs, 0, sizeof(_prefs));
    StrHelper::strncpy(_prefs.node_name, ADVERT_NAME, sizeof(_prefs.node_name));
    has_recv = false;
    post_pending = false;
    next_post_retry_at = 0;
    last_post_ok_at = 0;
    next_advert_millis = 0;
    has_pending_fav = false;
    memset(pending_fav_pub_key, 0, sizeof(pending_fav_pub_key));
    StrHelper::strncpy(wifi_status, "init", sizeof(wifi_status));
    StrHelper::strncpy(last_post_status, "idle", sizeof(last_post_status));
    memset(last_recv_from, 0, sizeof(last_recv_from));
    last_recv_dist_m = 0;
    last_recv_batt = 0;
    last_recv_node_id = 0;
    last_recv_hops = 0;
    last_recv_fwd_hops = 0xFF;  // v2
    last_recv_fwd_path[0] = 0;  // v2
    last_recv_direct = false;
    last_recv_at_millis = 0;
    recv_count = 0;
    // v2: zero out duplicate-detection LRU
    memset(dup_cache, 0, sizeof(dup_cache));
    dup_cache_next = 0;
  }

  const char* getNodeName() const { return _prefs.node_name; }
  const char* getWifiStatus() const { return wifi_status; }
  const char* getPostStatus() const { return last_post_status; }
  bool hasRecv() const { return has_recv; }
  float getLastDistMeters() const { return last_recv_dist_m; }
  float getLastBatt() const { return last_recv_batt; }
  uint16_t getLastNodeId() const { return last_recv_node_id; }
  uint8_t getLastHops() const { return last_recv_hops; }
  uint8_t getLastFwdHops() const { return last_recv_fwd_hops; }  // v2
  bool getLastDirect() const { return last_recv_direct; }
  const char* getLastFrom() const { return last_recv_from; }
  uint32_t getLastRecvAtMillis() const { return last_recv_at_millis; }
  uint16_t getRecvCount() const { return recv_count; }
  bool hasBayouKeys() const { return _prefs.bayou_public_key[0] != 0 && _prefs.bayou_private_key[0] != 0; }

  bool sendAdvert() {
    auto pkt = createSelfAdvert(_prefs.node_name, 0.0, 0.0);
    if (pkt) {
      sendZeroHop(pkt);
      Serial.println("   (advert sent, zero hop).");
      return true;
    }
    Serial.println("   ERR: unable to send");
    return false;
  }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;
    BaseChatMesh::begin();

  #if defined(NRF52_PLATFORM)
    IdentityStore store(fs, "");
  #elif defined(RP2040_PLATFORM)
    IdentityStore store(fs, "/identity");
    store.begin();
  #else
    IdentityStore store(fs, "/identity");
  #endif
    if (!store.load("_main", self_id, _prefs.node_name, sizeof(_prefs.node_name))) {
      Serial.println("Press ENTER to generate key:");
      char c = 0;
      while (c != '\n') {
        if (Serial.available()) c = Serial.read();
      }
      ((StdRNG *)getRNG())->begin(millis());

      self_id = mesh::LocalIdentity(getRNG());
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      store.save("_main", self_id);
    }

    loadPrefs();
    loadContacts();
  }

  void beginWifi() {
  #ifdef ESP32
    if (_prefs.wifi_ssid[0] != 0) {
      WiFi.mode(WIFI_STA);
      WiFi.begin(_prefs.wifi_ssid, _prefs.wifi_password);
      StrHelper::strncpy(wifi_status, "connecting", sizeof(wifi_status));
      Serial.printf("   WiFi connecting to: %s\n", _prefs.wifi_ssid);
    } else {
      StrHelper::strncpy(wifi_status, "no SSID", sizeof(wifi_status));
      Serial.println("   WiFi SSID not set (use 'set wifi_ssid <ssid>')");
    }
  #else
    StrHelper::strncpy(wifi_status, "n/a", sizeof(wifi_status));
  #endif
  }

  void showWelcome() {
    Serial.println("===== MeshCore Sensor Receiver =====");
    Serial.println();
    Serial.printf("Node: %s\n", _prefs.node_name);
    Serial.print("Public key: "); mesh::Utils::printHex(Serial, self_id.pub_key, PUB_KEY_SIZE); Serial.println();
    Serial.printf("WiFi SSID: %s\n", _prefs.wifi_ssid[0] ? _prefs.wifi_ssid : "(not set)");
    Serial.printf("Bayou public key: %s\n", _prefs.bayou_public_key[0] ? _prefs.bayou_public_key : "(not set)");
    Serial.printf("Bayou private key: %s\n", _prefs.bayou_private_key[0] ? "****" : "(not set)");
    Serial.println();
    Serial.println("Waiting for [SENSOR] messages...");
    Serial.println("   (enter 'help' for commands)");
    Serial.println();

    // Print biz card at boot
    auto pkt = createSelfAdvert(_prefs.node_name, 0.0, 0.0);
    if (pkt) {
      uint8_t len = pkt->writeTo(tmp_buf);
      releasePacket(pkt);
      mesh::Utils::toHex(hex_buf, tmp_buf, len);
      Serial.println("Your business card (share with sensor nodes):");
      Serial.print("meshcore://"); Serial.println(hex_buf);
      Serial.println();
    }
  }

  void sendSelfAdvert(int delay_millis) {
    auto pkt = createSelfAdvert(_prefs.node_name, 0.0, 0.0);
    if (pkt) {
      sendFlood(pkt, delay_millis);
    }
  }

  // ContactVisitor
  void onContactVisit(const ContactInfo& contact) override {
    bool fav = (contact.flags & 0x01) != 0;
    Serial.printf("   %s%s - ", contact.name, fav ? " *" : "");
    char tmp[40];
    int32_t secs = contact.last_advert_timestamp - getRTCClock()->getCurrentTime();
    AdvertTimeHelper::formatRelativeTimeDiff(tmp, secs, false);
    Serial.println(tmp);
  }

  bool setFavourite(const char* name_prefix, bool fav) {
    ContactInfo* c = searchContactsByPrefix(name_prefix);
    if (!c) return false;
    if (fav) c->flags |= 0x01;
    else     c->flags &= ~0x01;
    saveContacts();
    Serial.printf("   %s: %s\n", c->name, fav ? "favourite (protected from eviction)" : "unfavourited");
    return true;
  }

  void handleCommand(const char* command) {
    while (*command == ' ') command++;

    if (memcmp(command, "list", 4) == 0) {
      int n = 0;
      if (command[4] == ' ') n = atoi(&command[5]);
      Serial.printf("Contacts: %d/%d\n", getNumContacts(), MAX_CONTACTS);
      scanRecentContacts(n, this);
    } else if (strcmp(command, "purge") == 0) {
      int before = getNumContacts();
      resetContacts();
      saveContacts();
      Serial.printf("   Purged %d contacts (0/%d now)\n", before, MAX_CONTACTS);
    } else if (memcmp(command, "card", 4) == 0) {
      Serial.printf("Hello %s\n", _prefs.node_name);
      auto pkt = createSelfAdvert(_prefs.node_name, 0.0, 0.0);
      if (pkt) {
        uint8_t len = pkt->writeTo(tmp_buf);
        releasePacket(pkt);
        mesh::Utils::toHex(hex_buf, tmp_buf, len);
        Serial.println("Your MeshCore biz card:");
        Serial.print("meshcore://"); Serial.println(hex_buf);
        Serial.println();
      } else {
        Serial.println("  Error");
      }
    } else if (memcmp(command, "import ", 7) == 0) {
      importCard(&command[7]);
    } else if (memcmp(command, "favourite ", 10) == 0) {
      if (!setFavourite(&command[10], true)) Serial.println("   contact not found");
    } else if (memcmp(command, "unfavourite ", 12) == 0) {
      if (!setFavourite(&command[12], false)) Serial.println("   contact not found");
    } else if (strcmp(command, "advert") == 0) {
      sendAdvert();
    } else if (strcmp(command, "show_settings") == 0 || strcmp(command, "settings") == 0) {
      Serial.println("===== Settings =====");
      Serial.printf("   name:               %s\n", _prefs.node_name);
      Serial.printf("   wifi_ssid:          %s\n", _prefs.wifi_ssid[0] ? _prefs.wifi_ssid : "(not set)");
      Serial.printf("   wifi_password:      %s\n", _prefs.wifi_password[0] ? "****" : "(not set)");
      Serial.printf("   bayou_public_key:   %s\n", _prefs.bayou_public_key[0] ? _prefs.bayou_public_key : "(not set)");
      Serial.printf("   bayou_private_key:  %s\n", _prefs.bayou_private_key[0] ? "****" : "(not set)");
      Serial.printf("   contacts:           %d/%d\n", getNumContacts(), MAX_CONTACTS);
      Serial.print("   favourites:         ");
      int fav_count = 0;
      ContactInfo c;
      for (uint32_t i = 0; i < (uint32_t)getNumContacts(); i++) {
        if (getContactByIdx(i, c) && (c.flags & 0x01)) {
          Serial.printf("%s%s", fav_count ? ", " : "", c.name);
          fav_count++;
        }
      }
      if (fav_count == 0) Serial.print("(none)");
      Serial.println();
    } else if (strcmp(command, "status") == 0) {
      Serial.printf("   Contacts: %d/%d\n", getNumContacts(), MAX_CONTACTS);
      Serial.printf("   WiFi: %s\n", wifi_status);
      Serial.printf("   Bayou: %s\n", last_post_status);
      if (has_recv) {
        Serial.printf("   Last: from=%s node_id=%u dist=%.3fm batt=%.2fV "
                      "radio_hops=%u fwd_hops=%u fwd_path=%s route=%s\n",
                      last_recv_from, (unsigned)last_recv_node_id, last_recv_dist_m, last_recv_batt,
                      (unsigned)last_recv_hops, (unsigned)last_recv_fwd_hops,
                      last_recv_fwd_path, last_recv_direct ? "DIRECT" : "FLOOD");
      } else {
        Serial.println("   Last: no data received yet");
      }
    } else if (memcmp(command, "set ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "name ", 5) == 0) {
        StrHelper::strncpy(_prefs.node_name, &config[5], sizeof(_prefs.node_name));
        savePrefs();
        Serial.printf("   OK, name = %s\n", _prefs.node_name);
      } else if (memcmp(config, "wifi_ssid ", 10) == 0) {
        StrHelper::strncpy(_prefs.wifi_ssid, &config[10], sizeof(_prefs.wifi_ssid));
        savePrefs();
        Serial.printf("   OK, wifi_ssid = %s (reboot to apply)\n", _prefs.wifi_ssid);
      } else if (memcmp(config, "wifi_password ", 14) == 0) {
        StrHelper::strncpy(_prefs.wifi_password, &config[14], sizeof(_prefs.wifi_password));
        savePrefs();
        Serial.println("   OK, wifi_password set (reboot to apply)");
      } else if (memcmp(config, "bayou_public_key ", 17) == 0) {
        StrHelper::strncpy(_prefs.bayou_public_key, &config[17], sizeof(_prefs.bayou_public_key));
        savePrefs();
        Serial.printf("   OK, bayou_public_key = %s\n", _prefs.bayou_public_key);
      } else if (memcmp(config, "bayou_private_key ", 18) == 0) {
        StrHelper::strncpy(_prefs.bayou_private_key, &config[18], sizeof(_prefs.bayou_private_key));
        savePrefs();
        Serial.println("   OK, bayou_private_key set");
      } else {
        Serial.printf("   ERROR: unknown config: %s\n", config);
      }
    } else if (memcmp(command, "ver", 3) == 0) {
      Serial.println(FIRMWARE_VER_TEXT);
    } else if (memcmp(command, "help", 4) == 0) {
      Serial.println("Commands:");
      Serial.println("   card                          - show biz card");
      Serial.println("   import <biz card>             - import a contact");
      Serial.println("   list {n}                      - list contacts (* = favourite)");
      Serial.println("   purge                         - clear all contacts");
      Serial.println("   favourite <name>              - protect contact from eviction");
      Serial.println("   unfavourite <name>            - clear favourite flag");
      Serial.println("   advert                        - send advertisement");
      Serial.println("   status                        - show WiFi/Bayou/last data");
      Serial.println("   show_settings                 - show node config + favourites");
      Serial.println("   set name <name>               - set node name");
      Serial.println("   set wifi_ssid <ssid>          - set WiFi SSID (reboot)");
      Serial.println("   set wifi_password <pwd>       - set WiFi password (reboot)");
      Serial.println("   set bayou_public_key <key>    - set Bayou public key");
      Serial.println("   set bayou_private_key <key>   - set Bayou private key");
      Serial.println("   ver                           - firmware version");
      Serial.println("   help                          - show this help");
    } else {
      Serial.print("   ERROR: unknown command: "); Serial.println(command);
    }
  }

  void serviceBayou() {
  #ifdef ESP32
    if (!post_pending) return;

    unsigned long now = millis();
    if ((long)(now - next_post_retry_at) < 0) return;

    if (postToBayou(last_recv_dist_m, last_recv_batt, last_recv_from, last_recv_node_id,
                    last_recv_hops, last_recv_fwd_hops, last_recv_fwd_path,
                    last_recv_direct)) {
      post_pending = false;
    } else {
      next_post_retry_at = now + 15000;  // retry in 15 seconds
    }
  #endif
  }

  void loop() {
    BaseChatMesh::loop();

    // Service Bayou posting
    serviceBayou();

    // Periodic advertisement (every 15 minutes)
    unsigned long now = millis();
    if (next_advert_millis == 0) {
      next_advert_millis = now + (15UL * 60UL * 1000UL);
    }
    if (now >= next_advert_millis) {
      Serial.println("[ADVERT] Sending periodic advertisement...");
      sendSelfAdvert(0);
      next_advert_millis = now + (15UL * 60UL * 1000UL);
    }

    // Update WiFi status
  #ifdef ESP32
    if (WiFi.status() == WL_CONNECTED) {
      StrHelper::strncpy(wifi_status, "connected", sizeof(wifi_status));
    }
  #endif

    // Serial command handling
    int len = strlen(command);
    while (Serial.available() && len < (int)sizeof(command)-1) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {
        if (len > 0) {
          Serial.println();
          command[len] = 0;
          handleCommand(command);
          command[0] = 0;
          len = 0;
        }
        continue;
      }
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (len >= (int)sizeof(command)-1) {
      command[len] = 0;
      Serial.println();
      handleCommand(command);
      command[0] = 0;
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
ReceiverMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

/* ---------------------------------- DISPLAY & BUTTON ---------------------------------- */

#ifdef DISPLAY_CLASS
static uint8_t current_page = PAGE_STATUS;
static unsigned long next_display_refresh = 0;
static unsigned long last_button_activity = 0;
static char alert_text[32] = {0};
static unsigned long alert_expiry = 0;

static void showAlert(const char* text) {
  strncpy(alert_text, text, sizeof(alert_text)-1);
  alert_text[sizeof(alert_text)-1] = 0;
  alert_expiry = millis() + ALERT_DURATION_MS;
}

static void drawPageDots(DisplayDriver& d) {
  int y = 2;
  int x = d.width() / 2 - 5 * (PAGE_COUNT - 1);
  for (uint8_t i = 0; i < PAGE_COUNT; i++, x += 10) {
    if (i == current_page) {
      d.fillRect(x-1, y-1, 3, 3);
    } else {
      d.fillRect(x, y, 1, 1);
    }
  }
}

static void renderStatusPage(DisplayDriver& d) {
  d.setTextSize(1);
  d.setColor(DisplayDriver::LIGHT);

  d.setCursor(0, 10);
  d.print(the_mesh.getNodeName());

  char buf[32];
  snprintf(buf, sizeof(buf), "WiFi: %s", the_mesh.getWifiStatus());
  d.setCursor(0, 22);
  d.print(buf);

  snprintf(buf, sizeof(buf), "Bayou: %s", the_mesh.getPostStatus());
  d.setCursor(0, 34);
  d.print(buf);

  if (the_mesh.hasRecv()) {
    snprintf(buf, sizeof(buf), "%.2fm %.2fV", the_mesh.getLastDistMeters(), the_mesh.getLastBatt());
    d.setCursor(0, 46);
    d.print(buf);

    uint32_t age_s = (millis() - the_mesh.getLastRecvAtMillis()) / 1000;
    snprintf(buf, sizeof(buf), "RX #%u %us %s",
             (unsigned)the_mesh.getRecvCount(), (unsigned)age_s, the_mesh.getLastFrom());
    d.setCursor(0, 56);
    d.print(buf);
  } else {
    d.setCursor(0, 46);
    d.print("Waiting for data...");
  }
}

static void renderAdvertPage(DisplayDriver& d) {
  d.setTextSize(1);
  d.setColor(DisplayDriver::LIGHT);
  d.drawTextCentered(d.width() / 2, 24, "Send Advert");
  d.drawTextCentered(d.width() / 2, 44, "long press");
}

static void renderAlert(DisplayDriver& d) {
  int y = d.height() / 3;
  int p = d.height() / 16;
  d.setColor(DisplayDriver::DARK);
  d.fillRect(p, y, d.width() - p*2, y);
  d.setColor(DisplayDriver::LIGHT);
  d.drawRect(p, y, d.width() - p*2, y);
  d.drawTextCentered(d.width() / 2, y + p + 6, alert_text);
}

static void renderDisplay() {
  if (!display.isOn()) return;
  if (millis() < next_display_refresh) return;

  display.startFrame();
  drawPageDots(display);

  switch (current_page) {
    case PAGE_STATUS:  renderStatusPage(display); break;
    case PAGE_ADVERT:  renderAdvertPage(display); break;
  }

  if (millis() < alert_expiry) {
    renderAlert(display);
  }

  display.endFrame();
  next_display_refresh = millis() + DISPLAY_REFRESH_MS;
}

static void handleButtonEvents() {
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_NONE) return;

  last_button_activity = millis();
  if (!display.isOn()) {
    display.turnOn();
    next_display_refresh = 0;
    return;
  }

  if (ev == BUTTON_EVENT_CLICK) {
    current_page = (current_page + 1) % PAGE_COUNT;
    next_display_refresh = 0;
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    if (current_page == PAGE_ADVERT) {
      if (the_mesh.sendAdvert()) {
        showAlert("Advert sent!");
      } else {
        showAlert("Advert failed");
      }
    }
    next_display_refresh = 0;
  }
}

static void displayLoop() {
  handleButtonEvents();
  renderDisplay();

  if (display.isOn() && last_button_activity > 0 &&
      (millis() - last_button_activity) > DISPLAY_AUTO_OFF_MS) {
    display.turnOff();
  }
}
#endif  // DISPLAY_CLASS

/* ---------------------------------- SETUP & LOOP -------------------------------------- */

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);
  board.begin();

  if (!radio_init()) { halt(); }
  fast_rng.begin(radio_get_rng_seed());

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Starting...");
    display.endFrame();
  }
#endif

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  the_mesh.begin(InternalFS);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  the_mesh.begin(LittleFS);
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);
#else
  #error "need to define filesystem"
#endif

  radio_set_params(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
  radio_set_tx_power(LORA_TX_POWER);
  Serial.printf("[RADIO] freq=%.3f MHz  bw=%.1f kHz  sf=%d  cr=4/%d  txpwr=%d\n",
                (double)LORA_FREQ, (double)LORA_BW, (int)LORA_SF, (int)LORA_CR, (int)LORA_TX_POWER);

  the_mesh.beginWifi();
  the_mesh.showWelcome();

#ifdef DISPLAY_CLASS
  last_button_activity = millis();
#endif

#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvert(1200);
#endif
}

void loop() {
  the_mesh.loop();
#ifdef DISPLAY_CLASS
  displayLoop();
#endif
  rtc_clock.tick();
}
