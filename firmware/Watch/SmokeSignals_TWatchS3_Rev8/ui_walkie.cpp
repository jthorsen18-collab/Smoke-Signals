/**
 * @file      ui_walkie.cpp
 * @author
 * @license   MIT
 * @copyright Copyright (c) 2025  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2025-01-05
 *
 * ESP-NOW Walkie-Talkie (PTT) App
 *
 * Uses PCMFlowG722 G.722 wideband audio codec for real-time voice
 * communication over ESP-NOW broadcast. Half-duplex operation:
 *  - Hold PTT (encoder center button or touch) to talk
 *  - Release to listen for incoming audio
 *
 * Flow:
 *  1. Setup page: pick a device nickname and the ESP-NOW channel.
 *     When WiFi is already connected the channel is locked to the
 *     WiFi channel (ESP-NOW must share the radio channel).
 *  2. Talk page: a contact list (peer nicknames discovered through the
 *     ESP-NOW handshake) on the left and the PTT/status panel on the right.
 *
 * Handshake: each device periodically broadcasts a small "hello" packet
 * carrying its nickname so peers can populate their contact list and show
 * who is currently transmitting.
 *
 * Frame: 20ms G.722 @ 64kbps = 160 bytes per ESP-NOW packet
 *
 * Requirements:
 *  - PCMFlow library (https://github.com/lbuque/PCMFlow)
 *  - PCMFlowG722 library (https://github.com/tanakamasayuki/PCMFlowG722)
 */
#include "ui_define.h"
#include "esp_arduino_version.h"

// Version marked, new version not compatible
#if (ESP_ARDUINO_VERSION <= ESP_ARDUINO_VERSION_VAL(3,0,0)) && defined(ARDUINO_T_LORA_PAGER)

#include <esp_now.h>
#include <esp_wifi.h>
#include <PCMFlow.h>
#include <PCMFlowG722.h>

using namespace std;

LV_IMG_DECLARE(img_microphone);

// ============================================================
// Constants
// ============================================================
static constexpr uint16_t    kSampleRate    = 16000;
static constexpr uint8_t     kChannels      = 1;
static constexpr uint8_t     kBitsPerSample = 16;
static constexpr size_t      kFrameSamples  = 320;   // 20ms @ 16kHz
static constexpr size_t      kFrameBytes    = 160;   // G.722: 2 PCM -> 1 byte
static constexpr uint8_t     kWifiChannel   = 1;     // default when WiFi is offline
static constexpr size_t      kRxQueueDepth  = 8;
static constexpr uint32_t    kTaskStack     = 4096;
static constexpr UBaseType_t kTaskPrio      = 10;

static constexpr size_t      kNickMax       = 20;    // incl. terminating null
static const char            kHelloMagic[4] = {'W', 'L', 'K', 'H'};
static constexpr uint32_t    kAnnounceMs    = 2000;  // hello broadcast period
static constexpr uint32_t    kContactTtlMs  = 15000; // drop peers gone this long
static constexpr size_t      kHelloQueueLen = 8;

static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- Wire formats ------------------------------------------
// Hello packet length differs from kFrameBytes so the receiver can tell
// control packets and audio frames apart purely by length + magic.
typedef struct __attribute__((packed)) {
    char magic[4];
    char nickname[kNickMax];
} walkie_hello_t;

// Queued peer announcement passed from the ESP-NOW callback to the UI timer.
typedef struct {
    uint8_t mac[6];
    char    nickname[kNickMax];
} walkie_peer_t;

// A discovered contact tracked by the UI.
typedef struct {
    uint8_t  mac[6];
    char     nickname[kNickMax];
    uint32_t last_seen_ms;
} walkie_contact_t;

// ============================================================
// Module state
// ============================================================
typedef enum {
    WALKIE_STATE_IDLE,
    WALKIE_STATE_TRANSMIT,
    WALKIE_STATE_RECEIVE,
} walkie_state_t;

static walkie_state_t   s_state      = WALKIE_STATE_IDLE;
static volatile bool    s_ptt_active = false;
static uint32_t         s_last_rx_ms = 0;

// Local identity / radio config chosen on the setup page.
static char             s_nickname[kNickMax] = "Walkie_01";
static uint8_t          s_channel    = kWifiChannel;

// Last station that sent us audio (for the "who is talking" display).
static uint8_t          s_talk_mac[6] = {0};
static volatile bool    s_have_talker = false;

// Discovered peers (owned by the UI thread / timer only).
static std::vector<walkie_contact_t> s_contacts;
static uint32_t         s_announce_ms = 0;

// G.722 codec instances
static G722Encoder      s_enc;
static G722Decoder      s_dec;

// PCMFlow playback pipeline (jitter buffer + format conversion).
static PCMFlow          s_audio;
static constexpr size_t kAudioBufferFrames = 2048;

// FreeRTOS
static TaskHandle_t     s_tx_task    = NULL;
static TaskHandle_t     s_rx_task    = NULL;
static QueueHandle_t    s_rx_queue   = NULL;
static QueueHandle_t    s_hello_queue = NULL;
static SemaphoreHandle_t s_codec_mtx = NULL;

// Lifecycle guard: true once the talk session (codec/ESP-NOW/tasks) is up.
static bool             s_started    = false;

// LVGL
static lv_timer_t      *s_timer      = NULL;
static lv_obj_t        *s_menu       = NULL;
static lv_obj_t        *s_page       = NULL;
static lv_obj_t        *s_ptt_btn    = NULL;
static lv_obj_t        *s_status_label = NULL;
static lv_obj_t        *s_talker_label = NULL;
static lv_obj_t        *s_contact_list = NULL;

// Mic "ripple" indicator (talk/receive visualisation)
static lv_obj_t        *s_mic_icon   = NULL;
static lv_obj_t        *s_ripple[2]  = {NULL, NULL};
static constexpr int    kMicDia      = 72;   // centre mic circle diameter
static constexpr int    kRippleGrow  = 64;   // how far the rings expand

// Setup-page widgets
static lv_obj_t        *s_nick_ta    = NULL;
static lv_obj_t        *s_chan_dd    = NULL;
#ifdef USING_TOUCHPAD
static lv_obj_t        *s_keyboard   = NULL;
#endif

// ============================================================
// ESP-NOW receive callback (called from the WiFi task context)
// ============================================================
static void on_esp_now_recv(const uint8_t *mac, const uint8_t *data, int len)
{
    if (!data) return;

    // Control packet: peer announcing its nickname.
    if (len == (int)sizeof(walkie_hello_t) &&
        memcmp(data, kHelloMagic, sizeof(kHelloMagic)) == 0) {
        const walkie_hello_t *hello = (const walkie_hello_t *)data;
        walkie_peer_t peer = {};
        memcpy(peer.mac, mac, 6);
        memcpy(peer.nickname, hello->nickname, kNickMax);
        peer.nickname[kNickMax - 1] = '\0';
        BaseType_t woken = pdFALSE;
        if (s_hello_queue) xQueueSendFromISR(s_hello_queue, &peer, &woken);
        return;
    }

    // Audio frame.
    if (len != (int)kFrameBytes) return;
    if (s_ptt_active) return;  // skip while transmitting

    memcpy(s_talk_mac, mac, 6);
    s_have_talker = true;

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_rx_queue, data, &woken);

    s_state      = WALKIE_STATE_RECEIVE;
    s_last_rx_ms = millis();
}

// ============================================================
// ESP-NOW setup
// ============================================================
static bool setup_espnow(uint8_t channel, bool wifi_connected)
{
    // When WiFi is connected we must not retune the radio: ESP-NOW simply
    // rides on the WiFi channel. Otherwise bring up a bare STA on `channel`.
    if (!wifi_connected) {
        WiFi.mode(WIFI_STA);
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    }

    if (esp_now_init() != ESP_OK) {
        log_e("esp_now_init failed");
        return false;
    }
    if (esp_now_register_recv_cb(on_esp_now_recv) != ESP_OK) {
        log_e("esp_now_register_recv_cb failed");
        return false;
    }

    esp_now_peer_info_t peer = {};
    peer.channel = 0;            // 0 = use the current WiFi channel
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_broadcast_mac, 6);

    if (esp_now_add_peer(&peer) != ESP_OK) {
        log_e("esp_now_add_peer failed");
        return false;
    }
    return true;
}

// Broadcast our nickname so peers can list us / label our transmissions.
static void send_hello()
{
    walkie_hello_t hello = {};
    memcpy(hello.magic, kHelloMagic, sizeof(kHelloMagic));
    strncpy(hello.nickname, s_nickname, kNickMax - 1);
    esp_now_send(s_broadcast_mac, (const uint8_t *)&hello, sizeof(hello));
}

// ============================================================
// TX task: capture -> encode -> ESP-NOW send
// ============================================================
static void walkie_tx_task(void *arg)
{
    int16_t pcm[kFrameSamples];
    uint8_t g722[kFrameBytes];

    while (1) {
        // Wait for PTT press notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        s_state = WALKIE_STATE_TRANSMIT;

        while (s_ptt_active) {
            // Read 20ms PCM from microphone (blocks ~20ms)
            xSemaphoreTake(s_codec_mtx, portMAX_DELAY);
            instance.codec.read((uint8_t *)pcm, sizeof(pcm));
            xSemaphoreGive(s_codec_mtx);

            // G.722 encode: 320 PCM samples -> 160 bytes
            size_t encoded = s_enc.encode(pcm, kFrameSamples, g722, sizeof(g722));
            if (encoded > 0) {
                esp_now_send(s_broadcast_mac, g722, encoded);
            }
        }

        s_state = WALKIE_STATE_IDLE;
    }
}

// ============================================================
// RX task: receive -> decode -> play
// ============================================================
static void walkie_rx_task(void *arg)
{
    uint8_t g722[kFrameBytes];
    int16_t pcm[kFrameSamples];

    while (1) {
        if (xQueueReceive(s_rx_queue, g722, portMAX_DELAY) != pdTRUE) continue;

        // Skip if currently transmitting (stale frame)
        if (s_ptt_active) continue;

        // Enqueue the encoded bytes into the decoder's internal FIFO
        // (pcm == nullptr). PCMFlow pulls and decodes them in pump().
        s_dec.decode(g722, sizeof(g722), nullptr, 0);

        // Advance the PCMFlow pipeline and drain decoded PCM into the
        // speaker, one frame at a time.
        s_audio.pump();
        while (s_audio.availableFrames() >= kFrameSamples) {
            size_t got = s_audio.readFrames(pcm, kFrameSamples);
            if (got == 0) break;
            xSemaphoreTake(s_codec_mtx, portMAX_DELAY);
            instance.codec.write((uint8_t *)pcm,
                                  got * (kBitsPerSample / 8));
            xSemaphoreGive(s_codec_mtx);
        }
    }
}

// ============================================================
// Contact list helpers (UI thread only)
// ============================================================
static walkie_contact_t *find_contact(const uint8_t *mac)
{
    for (auto &c : s_contacts) {
        if (memcmp(c.mac, mac, 6) == 0) return &c;
    }
    return nullptr;
}

static const char *nickname_for(const uint8_t *mac)
{
    walkie_contact_t *c = find_contact(mac);
    return c ? c->nickname : "Unknown";
}

static void rebuild_contact_list()
{
    if (!s_contact_list) return;
    lv_obj_clean(s_contact_list);

    if (s_contacts.empty()) {
        lv_obj_t *empty = lv_label_create(s_contact_list);
        lv_label_set_text(empty, "Searching...");
        lv_obj_set_style_text_color(empty, lv_color_hex(0xAAAAAA), 0);
        return;
    }

    for (auto &c : s_contacts) {
        lv_obj_t *row = lv_label_create(s_contact_list);
        lv_label_set_text_fmt(row, "%s %s", LV_SYMBOL_CALL, c.nickname);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_style_text_color(row, lv_color_hex(0x2A2A2A), 0);
        lv_label_set_long_mode(row, LV_LABEL_LONG_DOT);
    }
}

// Drain queued hello packets, update the contact table and prune stale
// peers. Returns true if the visible list changed.
static bool update_contacts()
{
    bool changed = false;
    walkie_peer_t peer;

    while (s_hello_queue &&
           xQueueReceive(s_hello_queue, &peer, 0) == pdTRUE) {
        walkie_contact_t *c = find_contact(peer.mac);
        if (c) {
            if (strncmp(c->nickname, peer.nickname, kNickMax) != 0) {
                strncpy(c->nickname, peer.nickname, kNickMax);
                changed = true;
            }
            c->last_seen_ms = millis();
        } else {
            walkie_contact_t nc = {};
            memcpy(nc.mac, peer.mac, 6);
            strncpy(nc.nickname, peer.nickname, kNickMax);
            nc.last_seen_ms = millis();
            s_contacts.push_back(nc);
            changed = true;
        }
    }

    // Expire peers we have not heard from in a while.
    uint32_t now = millis();
    for (size_t i = 0; i < s_contacts.size();) {
        if (now - s_contacts[i].last_seen_ms > kContactTtlMs) {
            s_contacts.erase(s_contacts.begin() + i);
            changed = true;
        } else {
            i++;
        }
    }
    return changed;
}

// ============================================================
// LVGL UI update timer (100ms)
// ============================================================
static void walkie_timer_cb(lv_timer_t *t)
{
    // --- Periodic handshake broadcast ---
    if (millis() - s_announce_ms >= kAnnounceMs) {
        s_announce_ms = millis();
        send_hello();
    }

    // --- Refresh the contact list ---
    if (update_contacts()) {
        rebuild_contact_list();
    }

    walkie_state_t st = s_state;

    // Auto-revert RECEIVE -> IDLE after 500ms of silence
    if (st == WALKIE_STATE_RECEIVE && millis() - s_last_rx_ms > 500) {
        s_state = WALKIE_STATE_IDLE;
        st      = WALKIE_STATE_IDLE;
        s_have_talker = false;
    }

    if (!s_status_label) return;

    // Per-state caption + accent colour.
    const char *caption;
    lv_color_t  accent;
    bool        active = true;
    switch (st) {
    case WALKIE_STATE_TRANSMIT:
        caption = "TALKING";
        accent  = lv_color_hex(0xCC3333);   // red
        break;
    case WALKIE_STATE_RECEIVE:
        caption = "RECEIVING";
        accent  = lv_color_hex(0x33AA33);   // green
        break;
    default:
        caption = "IDLE";
        accent  = lv_color_hex(0x2A2A2A);
        active  = false;
        break;
    }

    lv_label_set_text(s_status_label, caption);
    lv_obj_set_style_text_color(s_status_label,
                                active ? accent : lv_color_hex(0x888888), 0);

    // Mic button colour: lit with the accent while active, dark when idle.
    if (s_ptt_btn) {
        lv_obj_set_style_bg_color(s_ptt_btn,
                                  active ? accent : lv_color_hex(0x2A2A2A), 0);
    }

    // Ripple animation: two rings radiating outward and fading, 50% out of
    // phase with each other. Driven straight off this 100ms tick.
    static uint8_t ripple_phase = 0;
    if (active) {
        ripple_phase = (ripple_phase + 7) % 100;
    }
    for (int i = 0; i < 2; i++) {
        if (!s_ripple[i]) continue;
        if (!active) {
            lv_obj_set_style_bg_opa(s_ripple[i], LV_OPA_TRANSP, 0);
            continue;
        }
        int ph  = (ripple_phase + i * 50) % 100;          // 0..99
        int dia = kMicDia + kRippleGrow * ph / 100;        // grow outward
        lv_opa_t opa = (lv_opa_t)(LV_OPA_50 * (100 - ph) / 100);  // fade out
        lv_obj_set_size(s_ripple[i], dia, dia);
        lv_obj_center(s_ripple[i]);
        lv_obj_set_style_bg_color(s_ripple[i], accent, 0);
        lv_obj_set_style_bg_opa(s_ripple[i], opa, 0);
    }

    // "Who is talking" line
    if (s_talker_label) {
        if (st == WALKIE_STATE_RECEIVE && s_have_talker) {
            lv_label_set_text_fmt(s_talker_label, "From: %s",
                                  nickname_for(s_talk_mac));
        } else if (st == WALKIE_STATE_TRANSMIT) {
            lv_label_set_text(s_talker_label, "On air");
        } else {
            lv_label_set_text(s_talker_label, "");
        }
    }
}

// ============================================================
// Talk-session teardown (idempotent)
// ============================================================
static void walkie_stop()
{
    if (!s_started) return;
    s_started = false;

    // Stop TX immediately
    s_ptt_active = false;

    // Delete FreeRTOS tasks
    if (s_tx_task) { vTaskDelete(s_tx_task); s_tx_task = NULL; }
    if (s_rx_task) { vTaskDelete(s_rx_task); s_rx_task = NULL; }

    // Delete synchronisation primitives
    if (s_rx_queue)    { vQueueDelete(s_rx_queue);    s_rx_queue    = NULL; }
    if (s_hello_queue) { vQueueDelete(s_hello_queue); s_hello_queue = NULL; }
    if (s_codec_mtx)   { vSemaphoreDelete(s_codec_mtx); s_codec_mtx = NULL; }

    // Stop audio codec
    instance.codec.close();

    // Tear down PCMFlow pipeline (safe: the RX task that pumps it is gone).
    s_audio.close();

    // Release G.722 codec resources
    s_enc.end();
    s_dec.end();

    // Cleanup ESP-NOW
    esp_now_unregister_recv_cb();
    esp_now_deinit();

    s_contacts.clear();
}

// ============================================================
// Back button handler (full cleanup)
// ============================================================
static void back_event_handler(lv_event_t *e)
{
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    if (!lv_menu_back_btn_is_root(s_menu, obj)) return;

    // Delete LVGL timer
    if (s_timer) { lv_timer_del(s_timer); s_timer = NULL; }

    walkie_stop();

    // Leaving via the back button while still on the setup page: make sure the
    // keypad indev is disabled so menu navigation is not captured by it.
    disable_keyboard();

#ifdef USING_TOUCHPAD
    if (s_keyboard) { lv_obj_del(s_keyboard); s_keyboard = NULL; }
#endif

    // Delete LVGL widgets
    lv_obj_clean(s_menu);
    lv_obj_del(s_menu);
    s_menu         = NULL;
    s_page         = NULL;
    s_ptt_btn      = NULL;
    s_status_label = NULL;
    s_talker_label = NULL;
    s_contact_list = NULL;
    s_mic_icon     = NULL;
    s_ripple[0]    = NULL;
    s_ripple[1]    = NULL;
    s_nick_ta      = NULL;
    s_chan_dd      = NULL;

    menu_show();
}

// ============================================================
// PTT button event handler (works for both encoder & touch)
// ============================================================
static void ptt_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED) {
        s_ptt_active = true;
        xTaskNotifyGive(s_tx_task);
    } else if (code == LV_EVENT_RELEASED ||
               code == LV_EVENT_CLICKED) {
        s_ptt_active = false;
    }
}

// ============================================================
// Talk page: contact list (left) + PTT/status panel (right)
// ============================================================
static void build_walkie_ui(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 4, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    // ---- Left: contacts ----
    lv_obj_t *left = lv_obj_create(cont);
    lv_obj_set_size(left, lv_pct(38), lv_pct(100));
    lv_obj_set_style_pad_all(left, 4, 0);
    lv_obj_set_style_border_width(left, 1, 0);
    lv_obj_set_style_border_color(left, lv_color_hex(0xDDDDDD), 0);
    lv_obj_set_style_radius(left, 6, 0);
    lv_obj_set_flex_flow(left, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *title = lv_label_create(left);
    lv_label_set_text(title, "Contacts");
    lv_obj_set_style_text_color(title, lv_color_hex(0x5f5f5f), 0);

    s_contact_list = lv_obj_create(left);
    lv_obj_set_width(s_contact_list, lv_pct(100));
    lv_obj_set_flex_grow(s_contact_list, 1);
    lv_obj_set_style_pad_all(s_contact_list, 2, 0);
    lv_obj_set_style_pad_row(s_contact_list, 4, 0);
    lv_obj_set_style_border_width(s_contact_list, 0, 0);
    lv_obj_set_flex_flow(s_contact_list, LV_FLEX_FLOW_COLUMN);
    rebuild_contact_list();

    // ---- Right: mic indicator + PTT ----
    // No layout manager here: the mic stage is centred in the panel, and the
    // captions are pinned to the top/bottom edges so the expanding ripple
    // rings never sit on top of the text.
    lv_obj_t *right = lv_obj_create(cont);
    lv_obj_set_flex_grow(right, 1);
    lv_obj_set_height(right, lv_pct(100));
    lv_obj_set_style_pad_all(right, 4, 0);
    lv_obj_set_style_border_width(right, 0, 0);
    lv_obj_clear_flag(right, LV_OBJ_FLAG_SCROLLABLE);

    // "Stage" with no layout so the ripple rings can be stacked behind the
    // mic button and centred on top of each other. Centred in `right`.
    int stage = kMicDia + kRippleGrow;
    lv_obj_t *mic_stage = lv_obj_create(right);
    lv_obj_set_size(mic_stage, stage, stage);
    lv_obj_set_style_bg_opa(mic_stage, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mic_stage, 0, 0);
    lv_obj_set_style_pad_all(mic_stage, 0, 0);
    lv_obj_remove_flag(mic_stage, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(mic_stage, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_center(mic_stage);

    // Ripple rings (created first so they render behind the mic button).
    for (int i = 0; i < 2; i++) {
        s_ripple[i] = lv_obj_create(mic_stage);
        lv_obj_set_size(s_ripple[i], kMicDia, kMicDia);
        lv_obj_set_style_radius(s_ripple[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(s_ripple[i], 0, 0);
        lv_obj_set_style_bg_color(s_ripple[i], lv_color_hex(0xCC3333), 0);
        lv_obj_set_style_bg_opa(s_ripple[i], LV_OPA_TRANSP, 0);
        lv_obj_remove_flag(s_ripple[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_remove_flag(s_ripple[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_center(s_ripple[i]);
    }

    // Centre mic button (the PTT control).
    s_ptt_btn = lv_btn_create(mic_stage);
    lv_obj_set_size(s_ptt_btn, kMicDia, kMicDia);
    lv_obj_set_style_radius(s_ptt_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_ptt_btn, lv_color_hex(0x2A2A2A), 0);
    lv_obj_center(s_ptt_btn);
    lv_obj_add_event_cb(s_ptt_btn, ptt_btn_event_cb, LV_EVENT_ALL, NULL);

    s_mic_icon = lv_image_create(s_ptt_btn);
    lv_image_set_src(s_mic_icon, &img_microphone);
    lv_obj_center(s_mic_icon);
    lv_obj_set_style_image_recolor(s_mic_icon, lv_color_white(), 0);
    lv_obj_set_style_image_recolor_opa(s_mic_icon, LV_OPA_COVER, 0);

    // Caption pinned to the top (IDLE / TALKING / RECEIVING). Created after the
    // stage so it renders above the rings.
    s_status_label = lv_label_create(right);
    lv_label_set_text(s_status_label, "IDLE");
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x888888), 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 2);

    // Talker line pinned to the bottom.
    s_talker_label = lv_label_create(right);
    lv_label_set_text(s_talker_label, "");
    lv_obj_set_style_text_color(s_talker_label, lv_color_hex(0x5f5f5f), 0);
    lv_obj_align(s_talker_label, LV_ALIGN_BOTTOM_MID, 0, -2);
}

// ============================================================
// Bring up the talk session, then swap the setup page for the talk page
// ============================================================
static lv_obj_t *s_err_mbox = NULL;
static void show_error_and_back(lv_obj_t *parent, const char *msg);

static bool start_session()
{
    bool wifi_connected = (hw_get_wifi_status() == WL_CONNECTED);

    // --- Init G.722 codec ---
    PCMFormat fmt;
    fmt.sampleRate    = kSampleRate;
    fmt.channels      = kChannels;
    fmt.bitsPerSample = kBitsPerSample;
    if (!s_enc.begin(fmt) || !s_dec.begin(fmt)) {
        log_e("G.722 codec init failed");
        return false;
    }

    // --- Configure PCMFlow playback pipeline ---
    s_audio.setOutputFormat(fmt);
    s_audio.setBufferFrames(kAudioBufferFrames);
    s_audio.setInputSource(s_dec);

    // --- Init ESP-NOW on the chosen channel ---
    if (!setup_espnow(s_channel, wifi_connected)) {
        s_enc.end();
        s_dec.end();
        return false;
    }

    // --- Create FreeRTOS primitives ---
    s_rx_queue    = xQueueCreate(kRxQueueDepth, kFrameBytes);
    s_hello_queue = xQueueCreate(kHelloQueueLen, sizeof(walkie_peer_t));
    s_codec_mtx   = xSemaphoreCreateMutex();
    if (!s_rx_queue || !s_hello_queue || !s_codec_mtx) {
        if (s_rx_queue)    { vQueueDelete(s_rx_queue);    s_rx_queue    = NULL; }
        if (s_hello_queue) { vQueueDelete(s_hello_queue); s_hello_queue = NULL; }
        if (s_codec_mtx)   { vSemaphoreDelete(s_codec_mtx); s_codec_mtx = NULL; }
        s_enc.end(); s_dec.end();
        esp_now_unregister_recv_cb();
        esp_now_deinit();
        return false;
    }

    // --- Create tasks ---
    xTaskCreate(walkie_tx_task, "wlk_tx", kTaskStack,
                NULL, kTaskPrio, &s_tx_task);
    xTaskCreate(walkie_rx_task, "wlk_rx", kTaskStack,
                NULL, kTaskPrio, &s_rx_task);

    // --- Open audio codec (mono, 16kHz, 16-bit) ---
    instance.codec.open(kBitsPerSample, kChannels, kSampleRate);
    instance.codec.setVolume(85);

    s_started     = true;
    s_announce_ms = 0;     // announce immediately on the first timer tick
    return true;
}

static void start_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (s_started) return;

    // Capture the nickname.
    const char *nick = s_nick_ta ? lv_textarea_get_text(s_nick_ta) : "";
    if (nick && nick[0]) {
        strncpy(s_nickname, nick, kNickMax - 1);
        s_nickname[kNickMax - 1] = '\0';
    }

    // Capture the channel (unless WiFi already pinned it).
    if (s_chan_dd && !lv_obj_has_state(s_chan_dd, LV_STATE_DISABLED)) {
        s_channel = (uint8_t)lv_dropdown_get_selected(s_chan_dd) + 1;
    }

    // Make sure the keypad indev is back off before we leave the setup page.
    disable_keyboard();

#ifdef USING_TOUCHPAD
    if (s_keyboard) { lv_obj_del(s_keyboard); s_keyboard = NULL; }
#endif

    if (!start_session()) {
        show_error_and_back(s_page, "Failed to start session.");
        return;
    }

    // Swap the setup content for the talk page.
    lv_obj_clean(s_page);
    s_nick_ta = NULL;
    s_chan_dd = NULL;
    build_walkie_ui(s_page);

    // Start UI updates (status + handshake + contact list).
    s_timer = lv_timer_create(walkie_timer_cb, 100, NULL);
}

// Route physical keyboard input into the nickname field.
//
// On devices with a rotary + physical keyboard (e.g. T-LoRa-Pager) the keypad
// indev is off until a textarea enters edit mode. We mirror that here: a rotary
// click toggles edit mode and enables/disables the keypad, and pressing ENTER
// on the keyboard confirms and leaves edit mode (so the user is never stuck in
// the field). On touch devices an on-screen keyboard is shown instead.
static void nick_ta_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
    lv_indev_t *indev = lv_indev_active();
    if (!indev) return;
    lv_indev_type_t type = lv_indev_get_type(indev);
    bool edited = lv_obj_has_state(ta, LV_STATE_EDITED);

    if (code == LV_EVENT_KEY) {
        lv_key_t key = *(lv_key_t *)lv_event_get_param(e);
        if (key == LV_KEY_ENTER) {
            lv_group_set_editing((lv_group_t *)lv_obj_get_group(ta), false);
            disable_keyboard();
            lv_event_stop_processing(e);
            return;
        }
    }

    if (type == LV_INDEV_TYPE_ENCODER) {
        if (code == LV_EVENT_CLICKED && edited) {
            lv_group_set_editing((lv_group_t *)lv_obj_get_group(ta), false);
            disable_keyboard();
        } else if (code == LV_EVENT_FOCUSED && edited) {
            enable_keyboard();
        }
    }
#ifdef USING_TOUCHPAD
    else if (type == LV_INDEV_TYPE_POINTER && s_keyboard) {
        if (code == LV_EVENT_CLICKED) {
            lv_keyboard_set_textarea(s_keyboard, ta);
            lv_obj_remove_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
        }
    }
#endif
}

// ============================================================
// Setup page: nickname + ESP-NOW channel
// ============================================================
static void build_setup_ui(lv_obj_t *parent)
{
    bool wifi_connected = (hw_get_wifi_status() == WL_CONNECTED);
    if (wifi_connected) {
        // ESP-NOW must share the radio channel with the active WiFi link.
        uint8_t primary = kWifiChannel;
        wifi_second_chan_t second;
        if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
            s_channel = primary;
        }
    }

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_color(cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 8, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_flex_cross_place(cont, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_row(cont, 6, 0);

    // ---- Nickname ----
    lv_obj_t *nick_info = lv_label_create(cont);
    lv_label_set_text(nick_info, "DEVICE NICKNAME");
    lv_obj_set_style_text_color(nick_info, lv_color_hex(0x5f5f5f), 0);

    s_nick_ta = lv_textarea_create(cont);
    lv_obj_set_width(s_nick_ta, lv_pct(90));
    lv_obj_set_height(s_nick_ta, 40);
    lv_textarea_set_one_line(s_nick_ta, true);
    lv_textarea_set_max_length(s_nick_ta, kNickMax - 1);
    lv_textarea_set_text(s_nick_ta, s_nickname);
    lv_obj_add_event_cb(s_nick_ta, nick_ta_event_cb, LV_EVENT_ALL, NULL);

    // ---- ESP-NOW channel ----
    lv_obj_t *chan_info = lv_label_create(cont);
    lv_obj_set_style_text_color(chan_info, lv_color_hex(0x5f5f5f), 0);

    s_chan_dd = lv_dropdown_create(cont);
    lv_obj_set_width(s_chan_dd, lv_pct(90));
    lv_dropdown_set_options(s_chan_dd,
                            "1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n13\n14");
    lv_dropdown_set_selected(s_chan_dd,
                             s_channel >= 1 ? s_channel - 1 : 0);

    if (wifi_connected) {
        // Lock the channel to WiFi's.
        lv_label_set_text_fmt(chan_info, "ESP-NOW Channel (WiFi: %u)",
                              (unsigned)s_channel);
        lv_obj_add_state(s_chan_dd, LV_STATE_DISABLED);
    } else {
        lv_label_set_text(chan_info, "ESP-NOW Channel");
    }

    // ---- Start ----
    lv_obj_t *start_btn = lv_btn_create(cont);
    lv_obj_set_width(start_btn, lv_pct(90));
    lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x2A2A2A), 0);
    lv_obj_set_style_radius(start_btn, 12, 0);
    lv_obj_add_event_cb(start_btn, start_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *start_label = lv_label_create(start_btn);
    lv_label_set_text(start_label, "Start");
    lv_obj_center(start_label);
    lv_obj_set_style_text_color(start_label, lv_color_white(), 0);

#ifdef USING_TOUCHPAD
    s_keyboard = lv_keyboard_create(lv_scr_act());
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_keyboard, [](lv_event_t *e) {
        lv_keyboard_set_textarea(s_keyboard, NULL);
        lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(s_keyboard, [](lv_event_t *e) {
        lv_keyboard_set_textarea(s_keyboard, NULL);
        lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_CANCEL, NULL);
#endif
}

// ============================================================
// Error dialog + return to menu
// ============================================================
static void err_ok_cb(lv_event_t *e)
{
    if (s_err_mbox) {
        destroy_msgbox(s_err_mbox);
        s_err_mbox = NULL;
    }
    menu_show();
}

static void show_error_and_back(lv_obj_t *parent, const char *msg)
{
    static const char *btns[] = {"OK", ""};
    s_err_mbox = create_msgbox(lv_scr_act(),
                                "Walkie Error", msg,
                                btns, err_ok_cb, NULL);
}

// ============================================================
// enter / exit
// ============================================================
void ui_walkie_enter(lv_obj_t *parent)
{
    // --- Check audio codec ---
    if (!(hw_get_device_online() & HW_CODEC_ONLINE)) {
        show_error_and_back(parent, "Audio codec not available.");
        return;
    }

    // --- Build the setup page; the talk session starts on "Start" ---
    s_menu = create_menu(parent, back_event_handler);
    s_page = lv_menu_page_create(s_menu, NULL);
    build_setup_ui(s_page);
    lv_menu_set_page(s_menu, s_page);
}

void ui_walkie_exit(lv_obj_t *parent)
{
    // Cleanup is done in back_event_handler
}

app_t ui_walkie_main = {
    .setup_func_cb = ui_walkie_enter,
    .exit_func_cb  = ui_walkie_exit,
    .user_data     = nullptr,
};
#endif 
