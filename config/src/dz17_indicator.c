/*
 * DZ17 monochrome GPIO LED indicators.
 *
 * Wiring (per LED): anode -> VCC 3.3V rail, cathode -> 1k resistor -> GPIO.
 * Active-low: driving the GPIO LOW turns the LED on (handled by gpio-leds).
 *
 *   led0 (P0.22) = NumLock  -> solid while host reports NumLock on
 *   led1 (P0.12) = BLE prof 1
 *   led2 (P0.04) = BLE prof 2
 *   led3 (P0.26) = BLE prof 3
 *   led4 (P0.08) = USB output selected
 *
 * BLE profile indicator: solid when the selected profile is connected,
 * blinking while advertising/disconnected. USB indicator: solid while the
 * USB endpoint is selected. Only one channel family is active at a time.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/hid_indicators.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define BLINK_PERIOD_MS 500

#define LED_NUM   0   /* P0.22 */
#define LED_BLE0  1   /* P0.12 */
#define LED_BLE1  2   /* P0.04 */
#define LED_BLE2  3   /* P0.26 */
#define LED_USB   4   /* P0.08 */
#define BLE_COUNT 3

/* HID LED report bitmasks (USB HID Usage Tables, LED report) */
#define HID_LED_NUM_LOCK 0x01

static const struct device *const led_dev = DEVICE_DT_GET(DT_INST(0, gpio_leds));

static int blinking_led = -1;
static bool blink_on = false;
static struct k_work_delayable blink_work;

static void led_set(int idx, bool on) {
    if (on) {
        led_on(led_dev, idx);
    } else {
        led_off(led_dev, idx);
    }
}

static void blink_stop(void) {
    if (blinking_led >= 0) {
        led_set(blinking_led, false);
        blinking_led = -1;
    }
    k_work_cancel_delayable(&blink_work);
}

/* Reflect the given BLE profile on LEDs 1..3. Solid if connected,
 * otherwise blinking. Turns the other two BLE LEDs off. */
static void apply_ble_profile(uint8_t profile) {
    blink_stop();
    for (int i = 0; i < BLE_COUNT; i++) {
        led_set(LED_BLE0 + i, false);
    }
    if (profile >= BLE_COUNT) {
        return;
    }
    if (zmk_ble_profile_is_connected(profile)) {
        led_set(LED_BLE0 + profile, true);
    } else {
        blinking_led = LED_BLE0 + profile;
        blink_on = true;
        led_set(blinking_led, true);
        k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
    }
}

static void blink_handler(struct k_work *w) {
    if (blinking_led < 0) {
        return;
    }
    blink_on = !blink_on;
    led_set(blinking_led, blink_on);
    k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
}

/* ---- events ---- */
static int ble_profile_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);
    if (!ev || ev->index >= BLE_COUNT) {
        return 0;
    }

    /* Rename the advertiser so hosts show czm_ble_1/2/3 per channel. */
    static char name[16];
    snprintf(name, sizeof(name), "czm_ble_%d", ev->index + 1);
    zmk_ble_set_device_name(name);

    LOG_INF("BLE profile %d (connected=%d)", ev->index,
            zmk_ble_profile_is_connected(ev->index));
    apply_ble_profile(ev->index);
    return 0;
}
ZMK_LISTENER(dz17_ble_ind, ble_profile_listener);
ZMK_SUBSCRIPTION(dz17_ble_ind, zmk_ble_active_profile_changed);

static int endpoint_listener(const zmk_event_t *eh) {
    const struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (!ev) {
        return 0;
    }

    if (ev->endpoint.transport == ZMK_TRANSPORT_USB) {
        LOG_INF("Output endpoint: USB");
        led_set(LED_USB, true);
        blink_stop();
        for (int i = 0; i < BLE_COUNT; i++) {
            led_set(LED_BLE0 + i, false);
        }
    } else {
        LOG_INF("Output endpoint: BLE");
        led_set(LED_USB, false);
        int cur = zmk_ble_active_profile_index();
        if (cur >= 0 && cur < BLE_COUNT) {
            apply_ble_profile((uint8_t)cur);
        }
    }
    return 0;
}
ZMK_LISTENER(dz17_ep_ind, endpoint_listener);
ZMK_SUBSCRIPTION(dz17_ep_ind, zmk_endpoint_changed);

static int hid_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev =
        as_zmk_hid_indicators_changed(eh);
    if (!ev) {
        return 0;
    }
    bool num_on = (ev->indicators & HID_LED_NUM_LOCK) != 0;
    LOG_INF("HID indicators=0x%02X numlock=%d", ev->indicators, num_on);
    led_set(LED_NUM, num_on);
    return 0;
}
ZMK_LISTENER(dz17_hid_ind, hid_indicators_listener);
ZMK_SUBSCRIPTION(dz17_hid_ind, zmk_hid_indicators_changed);

/* ---- init: sync current state so LEDs are correct right after boot ---- */
static int dz17_led_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("gpio-leds device not ready");
        return -ENODEV;
    }

    for (int i = 0; i < 5; i++) {
        led_off(led_dev, i);
    }
    k_work_init_delayable(&blink_work, blink_handler);

    struct zmk_endpoint_instance ep = zmk_endpoints_selected();
    if (ep.transport == ZMK_TRANSPORT_USB) {
        led_set(LED_USB, true);
    } else {
        int cur = zmk_ble_active_profile_index();
        if (cur >= 0 && cur < BLE_COUNT) {
            apply_ble_profile((uint8_t)cur);
        }
    }

    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    led_set(LED_NUM, (ind & HID_LED_NUM_LOCK) != 0);

    LOG_INF("DZ17 mono LEDs ready: Num=P0.22 BLE1=P0.12 BLE2=P0.04 BLE3=P0.26 USB=P0.08");
    return 0;
}
SYS_INIT(dz17_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
