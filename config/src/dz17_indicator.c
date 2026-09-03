/*
 * DZ17 monochrome GPIO LED indicators.
 *
 * Wiring (per LED): 3.3V rail -> 1k resistor -> LED anode (+),
 *                   LED cathode (-) -> GPIO.
 * Active-low: driving the GPIO LOW turns the LED on (handled by gpio-leds).
 *
 *   led0 (P0.22) = NumLock  -> follows host HID NumLock report
 *   led1 (P0.12) = BLE profile 1
 *   led2 (P0.04) = BLE profile 2
 *   led3 (P0.26) = BLE profile 3
 *   led4 (P0.08) = USB output selected
 *
 * Behavior (channel LEDs are momentary, event style):
 *   - BLE, profile not connected/advertising: its blue LED blinks (~1 Hz).
 *   - BLE, profile connected: its blue LED is solid for 2s as confirmation,
 *     then turns off; resumes blinking if the link drops.
 *   - USB selected: green LED solid for 2s, then off; all blue LEDs off.
 *   - Switching channel/profil re-triggers the confirmation/blink.
 *   - NumLock (white) independent: solid while host reports NumLock on.
 *
 * Implementation: a single 250ms tick RECOMPUTES the desired on/off state
 * of all 5 LEDs from the current transport/profile/connection/NumLock state
 * and force-writes every pin. Events only kick an immediate refresh; they
 * carry no incremental state, so a missed/late event or a stuck pin can
 * never leave an LED on for more than one tick.
 *
 * Pure event-listener module: no custom behavior, no WS2812, no proxy.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/hid_indicators.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define TICK_MS        250   /* full resync interval                 */
#define BLINK_HALF_MS  500   /* blink toggle interval (~1 Hz)        */
#define CONFIRM_MS     2000  /* solid confirmation window            */

#define LED_NUM   0   /* P0.22 */
#define LED_BLE0  1   /* P0.12 */
#define LED_BLE1  2   /* P0.04 */
#define LED_BLE2  3   /* P0.26 */
#define LED_USB   4   /* P0.08 */
#define LED_COUNT 5
#define BLE_COUNT 3

/* HID LED report bitmasks (USB HID Usage Tables, LED report) */
#define HID_LED_NUM_LOCK 0x01

static const struct device *const led_dev = DEVICE_DT_GET(DT_INST(0, gpio_leds));

static struct k_work_delayable tick_work;

/* Edge-detection state (initialised to impossible values so the first
 * tick always sees the current state as a fresh edge). */
static int last_usb      = -1;   /* 1 = USB transport, 0 = BLE */
static int last_profile  = -1;
static int last_connected = -1;  /* -1 unknown, 0/1 */

/* Momentary solid-confirmation window: confirm_led stays on until deadline. */
static int confirm_led = -1;
static int64_t confirm_deadline = 0;

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

    struct zmk_endpoint_instance ep = zmk_endpoints_selected();
    int usb = (ep.transport == ZMK_TRANSPORT_USB) ? 1 : 0;

    int cur = zmk_ble_active_profile_index();
    int connected = (!usb && cur >= 0 && cur < BLE_COUNT &&
                     zmk_ble_profile_is_connected((uint8_t)cur)) ? 1 : 0;

    /* ---- edges: (re)arm the 2s solid-confirmation window ---- */
    if (usb != last_usb) {
        if (usb) {
            confirm_led = LED_USB;
            confirm_deadline = now + CONFIRM_MS;
        } else {
            /* Just entered BLE: force profile/connected edges below to
             * re-arm the confirmation for the active BLE profile. */
            confirm_led = -1;
            last_profile = -1;
            last_connected = -1;
        }
        last_usb = usb;
    }

    if (!usb) {
        if (cur != last_profile) {
            last_profile = cur;
            last_connected = connected;
            confirm_led = (connected && cur >= 0) ? (LED_BLE0 + cur) : -1;
            if (confirm_led >= 0) {
                confirm_deadline = now + CONFIRM_MS;
            }
        } else if (connected != last_connected) {
            last_connected = connected;
            if (connected && cur >= 0) {
                confirm_led = LED_BLE0 + cur;
                confirm_deadline = now + CONFIRM_MS;
            } else {
                confirm_led = -1;   /* dropped -> blink, no solid window */
            }
        }
    }

    /* ---- desired state of every LED ---- */
    bool on[LED_COUNT] = {0};

    /* NumLock: read straight from the current HID report each tick. */
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    on[LED_NUM] = (ind & HID_LED_NUM_LOCK) != 0;

    if (usb) {
        /* USB: green only during the confirmation window; blue never on. */
        if (confirm_led == LED_USB && now < confirm_deadline) {
            on[LED_USB] = true;
        }
    } else if (cur >= 0 && cur < BLE_COUNT) {
        int led = LED_BLE0 + cur;
        if (!connected) {
            /* Advertising / disconnected: blink. */
            on[led] = ((now / BLINK_HALF_MS) % 2) == 0;
        } else if (confirm_led == led && now < confirm_deadline) {
            /* Connected: solid 2s confirmation, then off. */
            on[led] = true;
        }
    }

    /* ---- force-write all pins (idempotent; fixes any drift) ---- */
    for (int i = 0; i < LED_COUNT; i++) {
        led_set(i, on[i]);
    }

    if (confirm_led >= 0 && now >= confirm_deadline) {
        confirm_led = -1;
    }
}

static void tick_handler(struct k_work *w) {
    ARG_UNUSED(w);
    refresh_all();
    k_work_reschedule(&tick_work, K_MSEC(TICK_MS));
}

/* ---- events: just kick an immediate refresh (no state carried) ---- */
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
    refresh_all();
    return 0;
}
ZMK_LISTENER(dz17_ble_ind, ble_profile_listener);
ZMK_SUBSCRIPTION(dz17_ble_ind, zmk_ble_active_profile_changed);

static int endpoint_listener(const zmk_event_t *eh) {
    const struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (!ev) {
        return 0;
    }
    LOG_INF("Output endpoint: %s",
            ev->endpoint.transport == ZMK_TRANSPORT_USB ? "USB" : "BLE");
    refresh_all();
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
    LOG_INF("HID indicators=0x%02X", ev->indicators);
    refresh_all();
    return 0;
}
ZMK_LISTENER(dz17_hid_ind, hid_indicators_listener);
ZMK_SUBSCRIPTION(dz17_hid_ind, zmk_hid_indicators_changed);

/* ---- init ---- */
static int dz17_led_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("gpio-leds device not ready");
        return -ENODEV;
    }

    for (int i = 0; i < LED_COUNT; i++) {
        led_off(led_dev, i);
    }
    k_work_init_delayable(&tick_work, tick_handler);

    /* First tick runs immediately and arms the correct window/edges. */
    refresh_all();
    k_work_schedule(&tick_work, K_MSEC(TICK_MS));

    LOG_INF("DZ17 mono LEDs ready: Num=P0.22 BLE1=P0.12 BLE2=P0.04 BLE3=P0.26 USB=P0.08");
    return 0;
}
SYS_INIT(dz17_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
