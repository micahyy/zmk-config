/*
 * DZ17 monochrome GPIO LED indicators.
 *
 * Wiring (per LED): 3.3V rail -> 1k resistor -> LED anode (+),
 *                   LED cathode (-) -> GPIO.
 * Active-low: driving the GPIO LOW turns the LED on (handled by gpio-leds).
 *
 *   led0 (P0.22) = NumLock  -> follows host HID NumLock report
 *   led1 (P0.12) = BLE profile 1 (blue)
 *   led2 (P0.04) = BLE profile 2 (blue)
 *   led3 (P0.26) = BLE profile 3 (blue)
 *   led4 (P0.08) = USB output selected (green)
 *
 * Channel indicators are momentary (event style):
 *   - Switching channel / connecting: the LED blinks while
 *     advertising/disconnected, then stays solid for 2 seconds once the
 *     channel is active, then turns off.
 *   - USB selected: green LED solid for 2 seconds, then off; all blue LEDs off.
 *   - BLE selected: current profile blue LED blinks while not connected,
 *     solid 2s once connected, then off; drops back to blinking if the
 *     connection is lost.
 *   - Only one channel family is shown at a time.
 * NumLock (white) is independent: solid while the host reports NumLock on.
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

#define BLINK_PERIOD_MS   500   /* blink toggle interval            */
#define SOLID_HOLD_MS     2000  /* solid confirmation before off    */
#define MONITOR_PERIOD_MS 2000  /* fallback state-resync interval   */

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

/* Channel indicator state machine */
enum ch_state { CH_IDLE, CH_BLINK, CH_SOLID };
static enum ch_state ch_state = CH_IDLE;
static int ch_led = -1;          /* LED index currently used as channel indicator */
static bool blink_on = false;

static struct k_work_delayable blink_work;    /* 500ms blink toggle   */
static struct k_work_delayable solid_off_work;/* turn solid LED off   */
static struct k_work_delayable monitor_work;  /* periodic resync      */

/* Last known NumLock state, used when re-syncing after init/wake. */
static bool numlock_on = false;

static void led_set(int idx, bool on) {
    if (on) {
        led_on(led_dev, idx);
    } else {
        led_off(led_dev, idx);
    }
}

static void channel_stop(void) {
    ch_state = CH_IDLE;
    ch_led = -1;
    k_work_cancel_delayable(&blink_work);
    k_work_cancel_delayable(&solid_off_work);
    for (int i = 0; i < BLE_COUNT; i++) {
        led_set(LED_BLE0 + i, false);
    }
    led_set(LED_USB, false);
}

static void channel_solid(int idx) {
    channel_stop();
    ch_state = CH_SOLID;
    ch_led = idx;
    blink_on = true;
    led_set(idx, true);
    k_work_reschedule(&solid_off_work, K_MSEC(SOLID_HOLD_MS));
}

static void channel_blink(int idx) {
    channel_stop();
    ch_state = CH_BLINK;
    ch_led = idx;
    blink_on = true;
    led_set(idx, true);
    k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
}

/* Reflect the current transport/profile on the channel LEDs. */
static void channel_refresh(void) {
    struct zmk_endpoint_instance ep = zmk_endpoints_selected();

    if (ep.transport == ZMK_TRANSPORT_USB) {
        /* USB channel: green on for 2s as confirmation, then off. */
        channel_solid(LED_USB);
        return;
    }

    int cur = zmk_ble_active_profile_index();
    if (cur < 0 || cur >= BLE_COUNT) {
        channel_stop();
        return;
    }

    int idx = LED_BLE0 + cur;
    if (zmk_ble_profile_is_connected((uint8_t)cur)) {
        /* Connected: solid 2s confirmation, then off. */
        channel_solid(idx);
    } else {
        /* Advertising / disconnected: keep blinking until connected. */
        channel_blink(idx);
    }
}

static void blink_handler(struct k_work *w) {
    ARG_UNUSED(w);
    if (ch_state != CH_BLINK || ch_led < 0) {
        return;
    }

    /* Got connected while blinking? -> switch to solid confirmation. */
    int cur = zmk_ble_active_profile_index();
    if (cur >= 0 && cur < BLE_COUNT &&
        zmk_ble_profile_is_connected((uint8_t)cur)) {
        channel_solid(LED_BLE0 + cur);
        return;
    }

    blink_on = !blink_on;
    led_set(ch_led, blink_on);
    k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
}

static void solid_off_handler(struct k_work *w) {
    ARG_UNUSED(w);
    if (ch_state == CH_SOLID && ch_led >= 0) {
        led_set(ch_led, false);
    }
    ch_state = CH_IDLE;
    ch_led = -1;
}

/* Periodic fallback: catch state changes that arrive without an event
 * (BLE drop while idle, stale LED after transport switch, wake from sleep). */
static void monitor_handler(struct k_work *w) {
    ARG_UNUSED(w);

    struct zmk_endpoint_instance ep = zmk_endpoints_selected();

    if (ep.transport == ZMK_TRANSPORT_USB) {
        /* In USB mode no blue LED may ever be on. */
        for (int i = 0; i < BLE_COUNT; i++) {
            led_set(LED_BLE0 + i, false);
        }
        /* Only re-assert USB state if a blue LED somehow became the
         * channel indicator. CH_IDLE / green-SOLID are the normal states. */
        bool blue_is_channel = (ch_led >= LED_BLE0 && ch_led <= LED_BLE2);
        if (ch_state == CH_BLINK || blue_is_channel) {
            channel_solid(LED_USB);
        }
    } else {
        int cur = zmk_ble_active_profile_index();
        if (cur < 0 || cur >= BLE_COUNT) {
            if (ch_state != CH_IDLE) {
                channel_stop();
            }
        } else {
            bool connected = zmk_ble_profile_is_connected((uint8_t)cur);
            int expected = LED_BLE0 + cur;

            if (!connected) {
                /* Disconnected/advertising: must be blinking on this LED. */
                if (ch_state != CH_BLINK || ch_led != expected) {
                    channel_blink(expected);
                }
            } else if (ch_state == CH_BLINK && ch_led == expected) {
                /* Connected while blinking without a dedicated event. */
                channel_solid(expected);
            } else if ((ch_state == CH_BLINK || ch_state == CH_SOLID) &&
                       ch_led != expected) {
                /* Profile switched without an event (wake/resync). */
                channel_refresh();
            }
            /* CH_IDLE (confirmation done) or correct SOLID: leave alone. */
        }
    }

    k_work_reschedule(&monitor_work, K_MSEC(MONITOR_PERIOD_MS));
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

    /* While USB is the active output, channel LEDs stay under USB logic;
     * the monitor keeps blue LEDs forced off. */
    struct zmk_endpoint_instance ep = zmk_endpoints_selected();
    if (ep.transport != ZMK_TRANSPORT_USB) {
        channel_refresh();
    }
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
    channel_refresh();
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
    numlock_on = (ev->indicators & HID_LED_NUM_LOCK) != 0;
    LOG_INF("HID indicators=0x%02X numlock=%d", ev->indicators, numlock_on);
    led_set(LED_NUM, numlock_on);
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

    for (int i = 0; i < LED_COUNT; i++) {
        led_off(led_dev, i);
    }
    k_work_init_delayable(&blink_work, blink_handler);
    k_work_init_delayable(&solid_off_work, solid_off_handler);
    k_work_init_delayable(&monitor_work, monitor_handler);

    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    numlock_on = (ind & HID_LED_NUM_LOCK) != 0;
    led_set(LED_NUM, numlock_on);

    channel_refresh();
    k_work_reschedule(&monitor_work, K_MSEC(MONITOR_PERIOD_MS));

    LOG_INF("DZ17 mono LEDs ready: Num=P0.22 BLE1=P0.12 BLE2=P0.04 BLE3=P0.26 USB=P0.08");
    return 0;
}
SYS_INIT(dz17_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
