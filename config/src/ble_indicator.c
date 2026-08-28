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
#define BLINK_PERIOD_MS 400
#define BLE_SHOW_MS     3000
#define UG_OFF_WAIT_MS  250  /* wait for async underglow_off to finish */

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels[STRIP_LENGTH];

/* State */
static bool numlock_on;
static enum zmk_transport current_transport;
static bool indicator_mode;
static bool ug_was_on;
static bool blink_on;
static uint8_t active_profile;

static struct k_work_delayable ble_start_work;
static struct k_work_delayable blink_work;
static struct k_work_delayable ble_end_work;

/* Colors */
static const struct led_rgb blue_on  = { .r = 0x00, .g = 0x00, .b = 0xCC };
static const struct led_rgb blue_dim = { .r = 0x00, .g = 0x00, .b = 0x30 };
static const struct led_rgb black    = { .r = 0x00, .g = 0x00, .b = 0x00 };

static void write_black_all(void) {
    for (int i = 0; i < STRIP_LENGTH; i++) pixels[i] = black;
}

/* ---- BLE indicator: blink the active profile LED ---- */
static void blink_handler(struct k_work *w) {
    if (!indicator_mode) return;
    write_black_all();
    blink_on = !blink_on;
    pixels[active_profile] = blink_on ? blue_on : blue_dim;
    led_strip_update_rgb(strip, pixels, STRIP_LENGTH);
    k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
}

/* ---- BLE indicator end: restore underglow ---- */
static void ble_end_handler(struct k_work *w) {
    indicator_mode = false;
    k_work_cancel_delayable(&blink_work);
    write_black_all();
    led_strip_update_rgb(strip, pixels, STRIP_LENGTH);
    if (ug_was_on) {
        zmk_rgb_underglow_on();
    }
    LOG_INF("BLE indicator ended, ug restored=%d", ug_was_on);
}

/* ---- BLE indicator start: runs after waiting for underglow_off ---- */
static void ble_start_handler(struct k_work *w) {
    if (!device_is_ready(strip)) return;

    /* Now underglow_off async work has completed, we own the strip */
    indicator_mode = true;
    blink_on = true;
    write_black_all();
    pixels[active_profile] = blue_on;
    led_strip_update_rgb(strip, pixels, STRIP_LENGTH);

    k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
    k_work_reschedule(&ble_end_work, K_MSEC(BLE_SHOW_MS));
    LOG_INF("BLE indicator showing profile %d", active_profile);
}

/* ---- BLE profile changed: just capture and defer ---- */
static int ble_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);
    if (!ev || ev->index >= PROFILE_COUNT) return 0;

    LOG_INF("BLE profile event: %d", ev->index);
    active_profile = ev->index;

    /* Set BLE name (safe here, just updates bt name) */
    static char name[16];
    snprintf(name, sizeof(name), "czmao_dz17_%d", ev->index + 1);
    int ret = zmk_ble_set_device_name(name);
    LOG_INF("name '%s' ret=%d", name, ret);

    /* If already in indicator mode, just update the LED index */
    if (indicator_mode) {
        write_black_all();
        pixels[active_profile] = blue_on;
        led_strip_update_rgb(strip, pixels, STRIP_LENGTH);
        k_work_reschedule(&ble_end_work, K_MSEC(BLE_SHOW_MS));
        return 0;
    }

    /* Turn off underglow, wait for it to finish writing black, then take over */
    bool ug = false;
    zmk_rgb_underglow_get_state(&ug);
    ug_was_on = ug;
    zmk_rgb_underglow_off();

    k_work_reschedule(&ble_start_work, K_MSEC(UG_OFF_WAIT_MS));
    return 0;
}
ZMK_LISTENER(ble_ind, ble_listener);
ZMK_SUBSCRIPTION(ble_ind, zmk_ble_active_profile_changed);

/* ---- HID indicators: only update strip if underglow is OFF ---- */
static int hid_ind_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev =
        as_zmk_hid_indicators_changed(eh);
    if (!ev) return 0;
    numlock_on = (ev->indicators & 0x01) != 0;
    LOG_INF("HID: 0x%02x nl=%d", ev->indicators, numlock_on);
    return 0;
}
ZMK_LISTENER(hid_ind, hid_ind_listener);
ZMK_SUBSCRIPTION(hid_ind, zmk_hid_indicators_changed);

/* ---- Endpoint changed: just track state ---- */
static int ep_listener(const zmk_event_t *eh) {
    const struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (!ev) return 0;
    current_transport = ev->endpoint.transport;
    LOG_INF("endpoint: %d", current_transport);
    return 0;
}
ZMK_LISTENER(ep_ind, ep_listener);
ZMK_SUBSCRIPTION(ep_ind, zmk_endpoint_changed);

/* ---- Init: set up work items ONLY, never touch the LED strip ---- */
static int indicators_init(void) {
    k_work_init_delayable(&ble_start_work, ble_start_handler);
    k_work_init_delayable(&blink_work, blink_handler);
    k_work_init_delayable(&ble_end_work, ble_end_handler);

    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    numlock_on = (ind & 0x01) != 0;
    current_transport = zmk_endpoints_selected().transport;
    indicator_mode = false;

    LOG_INF("init ok: nl=%d transport=%d (no strip access)", numlock_on, current_transport);
    return 0;
}
SYS_INIT(indicators_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
