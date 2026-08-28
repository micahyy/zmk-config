#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/hid_indicators.h>
#include <zmk/rgb_underglow.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define STRIP_NODE      DT_NODELABEL(led_strip)
#define STRIP_LENGTH    DT_PROP(STRIP_NODE, chain_length)
#define PROFILE_COUNT   3
#define STATUS_LED      0
#define BLE_BASE_LED    0
#define BLINK_PERIOD_MS 300
#define BLE_SHOW_MS     3000

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels[STRIP_LENGTH];

static bool numlock_on;
static bool ble_show_active;
static bool ug_was_on;
static bool blink_on;
static uint8_t pending_profile = 0xFF;
static bool has_pending_profile;
static enum zmk_transport current_transport;

enum ble_state { BLE_OFF = 0, BLE_BLINK, BLE_SOLID };
static enum ble_state ble_states[PROFILE_COUNT];

static struct k_work_delayable ble_work;
static struct k_work_delayable blink_work;
static struct k_work_delayable render_work;
static struct k_work_delayable hide_work;

static const struct led_rgb white    = { .r = 0x60, .g = 0x60, .b = 0x60 };
static const struct led_rgb green    = { .r = 0x00, .g = 0x80, .b = 0x00 };
static const struct led_rgb blue_on  = { .r = 0x00, .g = 0x00, .b = 0xC0 };
static const struct led_rgb blue_dim = { .r = 0x00, .g = 0x00, .b = 0x30 };
static const struct led_rgb black    = { .r = 0x00, .g = 0x00, .b = 0x00 };

static int strip_update(void) {
    return led_strip_update_rgb(strip, pixels, STRIP_LENGTH);
}

static void fill_all(const struct led_rgb *c) {
    for (int i = 0; i < STRIP_LENGTH; i++) pixels[i] = *c;
}

static void do_render(void) {
    if (ble_show_active) {
        fill_all(&black);
        for (int p = 0; p < PROFILE_COUNT; p++) {
            int idx = BLE_BASE_LED + p;
            if (idx >= STRIP_LENGTH) continue;
            switch (ble_states[p]) {
            case BLE_BLINK: pixels[idx] = blink_on ? blue_on : blue_dim; break;
            case BLE_SOLID: pixels[idx] = blue_on; break;
            case BLE_OFF:   break;
            }
        }
    } else {
        fill_all(&black);
        if (current_transport == ZMK_TRANSPORT_USB) {
            pixels[STATUS_LED] = green;
        } else if (numlock_on) {
            pixels[STATUS_LED] = white;
        }
    }
    strip_update();
}

/* All BLE profile work done here, NOT in the event listener (avoids deadlock) */
static void ble_work_handler(struct k_work *w) {
    if (!has_pending_profile) return;
    uint8_t idx = pending_profile;
    has_pending_profile = false;

    LOG_INF("BLE work: profile=%d", idx);

    /* Rename BLE device */
    static char name[16];
    snprintf(name, sizeof(name), "czmao_dz17_%d", idx + 1);
    int ret = zmk_ble_set_device_name(name);
    LOG_INF("BLE name -> '%s' ret=%d", name, ret);

    /* Turn off underglow */
    bool ug = false;
    zmk_rgb_underglow_get_state(&ug);
    ug_was_on = ug;
    zmk_rgb_underglow_off();

    ble_show_active = true;
    blink_on = true;
    for (int p = 0; p < PROFILE_COUNT; p++) ble_states[p] = BLE_OFF;

    if (zmk_ble_profile_is_connected(idx)) {
        ble_states[idx] = BLE_SOLID;
    } else {
        ble_states[idx] = BLE_BLINK;
    }

    fill_all(&black);
    strip_update();
    k_work_reschedule(&render_work, K_MSEC(100));
    k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS + 100));
    k_work_reschedule(&hide_work, K_MSEC(BLE_SHOW_MS));
}

static void render_handler(struct k_work *w) { do_render(); }

static void hide_handler(struct k_work *w) {
    ble_show_active = false;
    for (int p = 0; p < PROFILE_COUNT; p++) ble_states[p] = BLE_OFF;
    fill_all(&black);
    if (current_transport == ZMK_TRANSPORT_USB) {
        pixels[STATUS_LED] = green;
    } else if (numlock_on) {
        pixels[STATUS_LED] = white;
    }
    strip_update();
    if (ug_was_on) zmk_rgb_underglow_on();
}

static void blink_handler(struct k_work *w) {
    bool any = false;
    for (int p = 0; p < PROFILE_COUNT; p++) {
        if (ble_states[p] == BLE_SOLID) ble_states[p] = BLE_OFF;
        if (ble_states[p] != BLE_OFF) any = true;
    }
    if (!any) { k_work_reschedule(&hide_work, K_MSEC(50)); return; }
    blink_on = !blink_on;
    do_render();
    k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
}

/* ---- HID indicators (NumLock) ---- */
static int hid_ind_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev =
        as_zmk_hid_indicators_changed(eh);
    if (!ev) return 0;
    numlock_on = (ev->indicators & 0x01) != 0;
    LOG_INF("HID ind: 0x%02x nl=%d", ev->indicators, numlock_on);
    if (!ble_show_active) k_work_reschedule(&render_work, K_MSEC(10));
    return 0;
}
ZMK_LISTENER(hid_ind_listener, hid_ind_listener);
ZMK_SUBSCRIPTION(hid_ind_listener, zmk_hid_indicators_changed);

/* ---- Endpoint changed (USB/BLE) ---- */
static int endpoint_listener(const zmk_event_t *eh) {
    const struct zmk_endpoint_changed *ev =
        as_zmk_endpoint_changed(eh);
    if (!ev) return 0;
    current_transport = ev->endpoint.transport;
    LOG_INF("Endpoint: transport=%d", current_transport);
    if (!ble_show_active) k_work_reschedule(&render_work, K_MSEC(10));
    return 0;
}
ZMK_LISTENER(endpoint_listener, endpoint_listener);
ZMK_SUBSCRIPTION(endpoint_listener, zmk_endpoint_changed);

/* ---- BLE profile change: just capture, defer to work queue ---- */
static int ble_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);
    if (!ev || ev->index >= PROFILE_COUNT) return 0;
    LOG_INF("BLE profile event: %d", ev->index);
    pending_profile = ev->index;
    has_pending_profile = true;
    k_work_reschedule(&ble_work, K_MSEC(50));
    return 0;
}
ZMK_LISTENER(ble_indicator, ble_listener);
ZMK_SUBSCRIPTION(ble_indicator, zmk_ble_active_profile_changed);

static int indicators_init(void) {
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip not ready");
        return -ENODEV;
    }
    k_work_init_delayable(&ble_work, ble_work_handler);
    k_work_init_delayable(&blink_work, blink_handler);
    k_work_init_delayable(&render_work, render_handler);
    k_work_init_delayable(&hide_work, hide_handler);

    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    numlock_on = (ind & 0x01) != 0;
    struct zmk_endpoint_instance ep = zmk_endpoints_selected();
    current_transport = ep.transport;

    fill_all(&black);
    if (current_transport == ZMK_TRANSPORT_USB) {
        pixels[STATUS_LED] = green;
    } else if (numlock_on) {
        pixels[STATUS_LED] = white;
    }
    strip_update();
    LOG_INF("init: nl=%d transport=%d", numlock_on, current_transport);
    return 0;
}
SYS_INIT(indicators_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
