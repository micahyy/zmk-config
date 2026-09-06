/*
 * DZ17 monochrome GPIO LED indicators - minimal ZMK-default style.
 *
 * Wiring (per LED): 3.3V rail -> 1k resistor -> LED anode (+),
 *                   LED cathode (-) -> GPIO.
 * Active-low: driving the GPIO LOW turns the LED on (handled by gpio-leds).
 *
 *   led0 (P0.22) = NumLock -> follows host HID NumLock report (white)
 *   led1..4      = BLE profile 1/2/3 + USB -> stay OFF (custom blink /
 *                  confirmation behavior removed; ZMK-native behavior).
 *
 * Also keeps two non-light behaviors:
 *   - BLE advertiser name per active profile: czm_dz17_ble_1/2/3
 *   - Immediate persistence of the active profile on a real switch
 *     (ZMK's own save is debounced ~60s).
 */

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/ble.h>
#include <zmk/hid_indicators.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define LED_NUM   0   /* P0.22, NumLock (white) */
#define LED_COUNT 5

/* USB HID LED report bitmasks */
#define HID_LED_NUM_LOCK 0x01

static const struct device *const led_dev = DEVICE_DT_GET(DT_INST(0, gpio_leds));

static void set_numlock(bool on) {
    if (on) {
        led_on(led_dev, LED_NUM);
    } else {
        led_off(led_dev, LED_NUM);
    }
}

static void refresh_numlock(void) {
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    set_numlock((ind & HID_LED_NUM_LOCK) != 0);
}

static int hid_indicators_listener(const zmk_event_t *eh) {
    if (!as_zmk_hid_indicators_changed(eh)) {
        return 0;
    }
    refresh_numlock();
    return 0;
}
ZMK_LISTENER(dz17_hid_ind, hid_indicators_listener);
ZMK_SUBSCRIPTION(dz17_hid_ind, zmk_hid_indicators_changed);

/* Rename the advertiser per profile and persist a real switch immediately. */
static int ble_profile_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);
    if (!ev || ev->index >= CONFIG_BT_MAX_PAIRED) {
        return 0;
    }

    static int last_evt_profile = -1;
    bool real_switch = (last_evt_profile >= 0) &&
                       (last_evt_profile != ev->index);
    last_evt_profile = ev->index;

    static char name[20];
    snprintf(name, sizeof(name), "czm_dz17_ble_%d", ev->index + 1);
    zmk_ble_set_device_name(name);

#if defined(CONFIG_SETTINGS)
    if (real_switch) {
        uint8_t prof_idx = (uint8_t)ev->index;
        settings_save_one("ble/active_profile", &prof_idx, sizeof(prof_idx));
    }
#endif

    return 0;
}
ZMK_LISTENER(dz17_ble_ind, ble_profile_listener);
ZMK_SUBSCRIPTION(dz17_ble_ind, zmk_ble_active_profile_changed);

static int dz17_led_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("gpio-leds device not ready");
        return -ENODEV;
    }

    /* All LEDs off at boot; only NumLock is managed. */
    for (int i = 0; i < LED_COUNT; i++) {
        led_off(led_dev, i);
    }
    refresh_numlock();

    LOG_INF("DZ17 mono LEDs ready: only NumLock(P0.22) driven by HID state");
    return 0;
}
SYS_INIT(dz17_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
