/*
 * DZ17 monochrome GPIO LED indicators - ZMK default style.
 *
 * Wiring (per LED): 3.3V rail -> 1k resistor -> LED anode (+),
 *                   LED cathode (-) -> GPIO.
 * Active-low: driving the GPIO LOW turns the LED on (handled by gpio-leds).
 *
 *   All 5 LEDs are white; channels are told apart by POSITION, not color.
 *
 *   led0 (P0.22) = NumLock  -> follows host HID NumLock report
 *   led1 (P0.12) = BLE profile 1
 *   led2 (P0.04) = BLE profile 2
 *   led3 (P0.26) = BLE profile 3
 *   led4 (P0.08) = USB output selected
 *
 * Channel LED states (native ZMK indicator semantics; only the active
 * profile's LED is ever driven):
 *   connected .................... solid
 *   open / pairing (no bond) ..... fast blink (~2.5 Hz)
 *   bonded but not connected ..... slow blink (~1 Hz)
 *   inactive channels ............ off
 *
 * Also keeps two non-light behaviors:
 *   - BLE advertiser name per active profile: czm_dz17_ble_1/2/3
 *   - Immediate persistence of the active profile on a real switch
 *     (ZMK's own save is debounced ~60s).
 *
 * A 250ms tick recomputes the desired on/off state of all 5 LEDs from
 * current transport/profile/connection/NumLock state and force-writes
 * every pin. Events only kick an immediate refresh; they carry no
 * incremental state, so a missed/late event or stuck pin can never leave
 * an LED in the wrong state for more than one tick.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/hid_indicators.h>
#include <zmk/usb.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define TICK_MS       250   /* full resync interval */
#define BOOT_GRACE_MS 900   /* all LEDs dark after boot: VBUS reports only a
                             * few hundred ms after power-up. */
#define SLOW_HALF_MS  500   /* reconnect blink toggle interval (~1 Hz) */
#define FAST_HALF_MS  200   /* pairing blink toggle interval (~2.5 Hz) */

#define LED_NUM   0   /* P0.22, NumLock (white) */
#define LED_BLE0  1   /* P0.12, BLE profile 1 (blue) */
#define LED_BLE1  2   /* P0.04, BLE profile 2 (blue) */
#define LED_BLE2  3   /* P0.26, BLE profile 3 (blue) */
#define LED_USB   4   /* P0.08, USB powered (green) */
#define LED_COUNT 5
#define BLE_COUNT 3

/* USB HID LED report bitmasks */
#define HID_LED_NUM_LOCK 0x01

static const struct device *const led_dev = DEVICE_DT_GET(DT_INST(0, gpio_leds));

static struct k_work_delayable tick_work;
static int64_t boot_grace_end;

static void led_set(int idx, bool on) {
    if (on) {
        led_on(led_dev, idx);
    } else {
        led_off(led_dev, idx);
    }
}

/* Full recompute + force-write of every LED. Safe to call from any context. */
static void refresh_all(void) {
    int64_t now = k_uptime_get();
    bool on[LED_COUNT] = {0};

    /* Boot grace: keep every LED dark until VBUS/transport state settles. */
    if (now >= boot_grace_end) {
        /* NumLock (white): read straight from the current HID report. */
        zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
        on[LED_NUM] = (ind & HID_LED_NUM_LOCK) != 0;

        /* Output channel is mutually exclusive: USB mode shows only the
         * green LED; BLE mode shows only the active channel's blue LED.
         * (The BLE link may stay connected while the cable is in, but the
         * LEDs report the selected output, not the radio state.) */
        bool usb_mode =
            zmk_endpoints_selected().transport == ZMK_TRANSPORT_USB;
        on[LED_USB] = usb_mode;

        /* Active BLE channel only; inactive channels stay off. */
        if (!usb_mode) {
            int cur = zmk_ble_active_profile_index();
            if (cur >= 0 && cur < BLE_COUNT) {
                int led = LED_BLE0 + cur;
                if (zmk_ble_profile_is_connected((uint8_t)cur)) {
                    on[led] = true;                                 /* solid */
                } else if (zmk_ble_profile_is_open((uint8_t)cur)) {
                    on[led] = ((now / FAST_HALF_MS) % 2) == 0;      /* pairing */
                } else {
                    on[led] = ((now / SLOW_HALF_MS) % 2) == 0;      /* reconnect */
                }
            }
        }
    }

    for (int i = 0; i < LED_COUNT; i++) {
        led_set(i, on[i]);
    }
}

static void tick_handler(struct k_work *w) {
    ARG_UNUSED(w);
    refresh_all();
    k_work_reschedule(&tick_work, K_MSEC(TICK_MS));
}

/* ---- events: just kick an immediate refresh (no state carried) ---- */
static int hid_indicators_listener(const zmk_event_t *eh) {
    if (!as_zmk_hid_indicators_changed(eh)) {
        return 0;
    }
    refresh_all();
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

    refresh_all();
    return 0;
}
ZMK_LISTENER(dz17_ble_ind, ble_profile_listener);
ZMK_SUBSCRIPTION(dz17_ble_ind, zmk_ble_active_profile_changed);

/* Output (USB/BLE) switch: refresh immediately so the LEDs flip at once. */
static int endpoint_listener(const zmk_event_t *eh) {
    if (!as_zmk_endpoint_changed(eh)) {
        return 0;
    }
    refresh_all();
    return 0;
}
ZMK_LISTENER(dz17_ep_ind, endpoint_listener);
ZMK_SUBSCRIPTION(dz17_ep_ind, zmk_endpoint_changed);

static int dz17_led_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("gpio-leds device not ready");
        return -ENODEV;
    }

    for (int i = 0; i < LED_COUNT; i++) {
        led_off(led_dev, i);
    }
    boot_grace_end = k_uptime_get() + BOOT_GRACE_MS;
    k_work_init_delayable(&tick_work, tick_handler);

    /* First tick runs immediately; periodic tick keeps every pin honest. */
    refresh_all();
    k_work_schedule(&tick_work, K_MSEC(TICK_MS));

    LOG_INF("DZ17 mono LEDs ready: Num=P0.22 BLE1=P0.12 BLE2=P0.04 BLE3=P0.26 USB=P0.08");
    return 0;
}
SYS_INIT(dz17_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
