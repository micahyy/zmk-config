/*
 * CZM ZMK87 monochrome GPIO LED indicators.
 *
 * Wiring (per LED): 3.3V rail -> resistor -> LED anode (+),
 *                   LED cathode (-) -> GPIO.
 * Active-low: driving the GPIO LOW turns the LED on (handled by gpio-leds).
 *
 *   led0 (P0.20) = CapsLock    -> follows host HID CapsLock report
 *   led1 (P0.22) = ScrollLock  -> follows host HID ScrollLock report
 *   led2 (P1.00) = BLE profile 1
 *   led3 (P1.02) = BLE profile 2
 *   led4 (P1.04) = BLE profile 3
 *   led5 (P1.06) = USB output selected
 *
 * Behavior (Logitech K375s style; channel LEDs are momentary):
 *   - Channel key FN+1/2/3 TAP   = switch / reconnect channel. The channel
 *     LED SLOW blinks (~1 Hz) for ~30s while trying to reconnect, then
 *     turns off; connecting lights it solid for 3s.
 *   - Channel key FN+1/2/3 HOLD (>=1s) = pairing mode: clears that
 *     channel's bond and opens it for a new host. The LED FAST blinks
 *     (~2.5 Hz, no timeout) until a host pairs, then solid 3s and off.
 *   - USB selected (FN+4, or cable inserted): USB LED solid for 3s, then
 *     off. Channel LEDs are BLE-transport only and stay dark in USB mode
 *     (suppressed during the green 3s window so two LEDs never overlap).
 *   - CapsLock / ScrollLock (white) independent: solid while the host
 *     reports the respective lock state.
 *
 * All three BLE channels advertise under the SAME name "czm_zmk87";
 * switching channels does NOT rename the advertiser.
 *
 * Implementation: a single 250ms tick RECOMPUTES the desired on/off state
 * of all 6 LEDs from the current transport/profile/connection/HID-lock
 * state and force-writes every pin. Events only kick an immediate
 * refresh; they carry no incremental state, so a missed/late event or a
 * stuck pin can never leave an LED on for more than one tick.
 *
 * Pins span BOTH GPIO ports: CAPS/SL on gpio0, BLE1-3/USB on gpio1. The
 * led (gpio-leds) API drives all indices transparently; the raw port/pin
 * table below is only needed to park pins before deep sleep, and it
 * selects the correct gpio device per pin.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/events/activity_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zephyr/settings/settings.h>

#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/hid_indicators.h>
#include <zmk/keymap.h>
#include <zmk/usb.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define TICK_MS        250   /* full resync interval                 */
#define BOOT_GRACE_MS  900   /* all channel LEDs off after boot: the
                             * USB driver reports VBUS only a few
                             * hundred ms after power-up, so without
                             * this a USB-powered boot would blink the
                             * active BLE LED before the green LED.   */
#define BLINK_HALF_MS  500   /* reconnect blink toggle (~1 Hz) */
#define PAIR_HALF_MS   200   /* pairing fast blink toggle (~2.5 Hz) */
#define PAIR_HOLD_MS   1000  /* hold a channel key this long to enter
                             * pairing mode (matches hold-tap term). */
#define CONFIRM_MS     3000  /* solid confirmation / cue window (3s) */
#define BLINK_TIMEOUT_MS 30000 /* unconnected/advertising: blink ~30s,
                                * then LED off until a state change. */
#define VBUS_GRACE_MS  1500  /* USB-enumeration grace on cable insert */

#define LED_CAPS   0   /* P0.20 */
#define LED_SL     1   /* P0.22 */
#define LED_BLE0   2   /* P1.00 */
#define LED_BLE1   3   /* P1.02 */
#define LED_BLE2   4   /* P1.04 */
#define LED_USB    5   /* P1.06 */
#define LED_COUNT  6
#define BLE_COUNT  3

/* Physical GPIO port/pin of every LED (used only for deep-sleep parking).
 * Active-low: led_off() drives the pin HIGH; reconfiguring it as a
 * pulled-up input then holds the OFF state in SYSTEM OFF. */
struct led_pin {
    const struct device *(*get_port)(void);
    uint8_t pin;
};

static const struct device *port_gpio0(void) {
    return DEVICE_DT_GET(DT_NODELABEL(gpio0));
}
static const struct device *port_gpio1(void) {
    return DEVICE_DT_GET(DT_NODELABEL(gpio1));
}

static const struct led_pin led_pins[LED_COUNT] = {
    { .get_port = port_gpio0, .pin = 20 },  /* CAPS P0.20 */
    { .get_port = port_gpio0, .pin = 22 },  /* SL   P0.22 */
    { .get_port = port_gpio1, .pin =  0 },  /* BLE1 P1.00 */
    { .get_port = port_gpio1, .pin =  2 },  /* BLE2 P1.02 */
    { .get_port = port_gpio1, .pin =  4 },  /* BLE3 P1.04 */
    { .get_port = port_gpio1, .pin =  6 },  /* USB  P1.06 */
};

/* FN layer + physical key positions of the BLE profile / output keys.
 * Position = row * 16 + col.
 *   FN+1 = (1,1) = 17, FN+2 = (1,2) = 18, FN+3 = (1,3) = 19,
 *   FN+4 = (1,4) = 20 (USB output). */
#define FN_LAYER    1
#define POS_OUT_USB (1 * 16 + 4)   /* 20 */
#define POS_BT_SEL0 (1 * 16 + 1)   /* 17 */
#define POS_BT_SEL1 (1 * 16 + 2)   /* 18 */
#define POS_BT_SEL2 (1 * 16 + 3)   /* 19 */

/* USB HID LED report bitmasks (USB HID Usage Tables, LED report byte) */
#define HID_LED_NUM_LOCK    0x01
#define HID_LED_CAPS_LOCK   0x02
#define HID_LED_SCROLL_LOCK 0x04

static const struct device *const led_dev = DEVICE_DT_GET(DT_INST(0, gpio_leds));

static struct k_work_delayable tick_work;
static struct k_work_delayable out_usb_work;

/* Edge-detection state (initialised to impossible values so the first
 * tick always sees the current state as a fresh edge). */
static int last_usb       = -1;   /* 1 = USB transport, 0 = BLE */
static int last_profile   = -1;
static int last_connected = -1;   /* -1 unknown, 0/1 */

/* Green USB confirmation window: USB LED stays on until deadline. */
static int64_t usb_confirm_deadline = 0;

/* Blue BLE confirmation window: the connected channel's blue LED is solid
 * until deadline (transport independent). */
static int blue_confirm_led = -1;
static int64_t blue_confirm_deadline = 0;

/* Pairing mode (Logitech-style): holding a channel key >= PAIR_HOLD_MS
 * clears that channel's bond. The target LED FAST blinks until a host
 * bonds; a successful connect ends pairing and lights it solid 3s. */
static int pairing_led = -1;

/* Channel key hold-timer for pairing detection. */
static int pair_pending_pos = -1;
static int64_t pair_pending_deadline = 0;

/* Advertising/disconnected blinking runs ~30s then stops. */
static int64_t blink_deadline = 0;

/* First tick after boot grace: force USB transport if VBUS present. */
static int force_usb_pending = 1;
static int64_t boot_grace_end = 0;

/* VBUS plug-in grace window. */
static int64_t vbus_grace_end = 0;
static int vbus_was_powered = 0;

/* Index carried by the last profile-changed event (real-switch detect). */
static int last_evt_profile = -1;

static void led_set(int idx, bool on) {
    if (on) {
        led_on(led_dev, idx);
    } else {
        led_off(led_dev, idx);
    }
}

static void arm_usb_confirm(void) {
    usb_confirm_deadline = k_uptime_get() + CONFIRM_MS;
}

/* Full recompute + force-write of every LED. Safe from any context. */
static void refresh_all(void) {
    int64_t now = k_uptime_get();

    if (now < boot_grace_end) {
        vbus_was_powered = zmk_usb_is_powered() ? 1 : 0;
        for (int i = 0; i < LED_COUNT; i++) {
            led_set(i, false);
        }
        return;
    }

    if (force_usb_pending) {
        force_usb_pending = 0;
        if (zmk_usb_is_powered()) {
            zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);
        }
    }

    int vbus_powered = zmk_usb_is_powered() ? 1 : 0;
    if (vbus_powered && !vbus_was_powered) {
        vbus_grace_end = now + VBUS_GRACE_MS;
        zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);
    }
    vbus_was_powered = vbus_powered;

    struct zmk_endpoint_instance ep = zmk_endpoints_selected();
    int usb = (ep.transport == ZMK_TRANSPORT_USB) ? 1 : 0;
    if (!usb && vbus_powered && now < vbus_grace_end) {
        usb = 1;
    }

    int cur = zmk_ble_active_profile_index();
    int connected = (cur >= 0 && cur < BLE_COUNT &&
                     zmk_ble_profile_is_connected((uint8_t)cur)) ? 1 : 0;

    /* ---- pairing hold-timer (armed by channel-key down event) ---- */
    if (pair_pending_pos >= 0 && now >= pair_pending_deadline) {
        int idx = pair_pending_pos - POS_BT_SEL0;
        if (idx >= 0 && idx < BLE_COUNT) {
            pairing_led = LED_BLE0 + idx;
            blink_deadline = 0;   /* pairing waits indefinitely for a host */
        }
        pair_pending_pos = -1;
    }

    /* ---- edges: (re)arm confirmation / blink windows ---- */
    if (usb != last_usb) {
        if (usb) {
            arm_usb_confirm();
        } else {
            last_profile = -1;
            last_connected = -1;
            if (!connected) {
                blink_deadline = now + BLINK_TIMEOUT_MS;
            }
        }
        last_usb = usb;
    }

    if (cur != last_profile) {
        if (pairing_led >= 0 && (cur < 0 || pairing_led != LED_BLE0 + cur)) {
            pairing_led = -1;
        }
        last_profile = cur;
        last_connected = connected;
        if (!connected) {
            if (pairing_led < 0 || pairing_led != (cur >= 0 ? LED_BLE0 + cur : -1)) {
                blink_deadline = now + BLINK_TIMEOUT_MS;
            }
            blue_confirm_led = -1;
        } else if (cur >= 0) {
            blue_confirm_led = LED_BLE0 + cur;
            blue_confirm_deadline = now + CONFIRM_MS;
            pairing_led = -1;
        }
    } else if (connected != last_connected) {
        last_connected = connected;
        if (connected && cur >= 0) {
            blue_confirm_led = LED_BLE0 + cur;
            blue_confirm_deadline = now + CONFIRM_MS;
            pairing_led = -1;
        } else {
            blue_confirm_led = -1;
            if (pairing_led < 0) {
                blink_deadline = now + BLINK_TIMEOUT_MS;
            }
        }
    }

    /* ---- desired state of every LED ---- */
    bool on[LED_COUNT] = {0};

    /* CapsLock / ScrollLock: read straight from the current HID report. */
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    on[LED_CAPS] = (ind & HID_LED_CAPS_LOCK) != 0;
    on[LED_SL]   = (ind & HID_LED_SCROLL_LOCK) != 0;

    /* USB LED: solid for the 3s confirmation window. */
    bool usb_green = usb && (now < usb_confirm_deadline);
    if (usb_green) {
        on[LED_USB] = true;
    }

    /* Channel LEDs are BLE-transport ONLY (dark in USB mode). */
    if (cur >= 0 && cur < BLE_COUNT && !usb) {
        int led = LED_BLE0 + cur;
        if (!connected) {
            if (pairing_led == led) {
                on[led] = ((now / PAIR_HALF_MS) % 2) == 0;
            } else if (now < blink_deadline) {
                on[led] = ((now / BLINK_HALF_MS) % 2) == 0;
            }
        } else if (blue_confirm_led == led && now < blue_confirm_deadline) {
            on[led] = true;
        }
    }

    /* ---- force-write all pins (idempotent; fixes any drift) ---- */
    for (int i = 0; i < LED_COUNT; i++) {
        led_set(i, on[i]);
    }

    if (blue_confirm_led >= 0 && now >= blue_confirm_deadline) {
        blue_confirm_led = -1;
    }
}

static void tick_handler(struct k_work *w) {
    ARG_UNUSED(w);
    refresh_all();
    k_work_reschedule(&tick_work, K_MSEC(TICK_MS));
}

/* ---- events: kick an immediate refresh (no state carried) ---- */
static int ble_profile_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);
    if (!ev || ev->index >= BLE_COUNT) {
        return 0;
    }

    /* Real switch = index actually changes (ZMK also fires this event on
     * connect/disconnect/pairing with the same index). */
    bool real_switch = (last_evt_profile >= 0) && (last_evt_profile != ev->index);
    last_evt_profile = ev->index;

    /* All three channels share the SAME advertised name "czm_zmk87";
     * channel switching intentionally does NOT rename the advertiser.
     * Persist the active profile immediately on a real switch so a quick
     * power-off does not revert to the old channel at next boot. */
#if defined(CONFIG_SETTINGS)
    if (real_switch) {
        uint8_t prof_idx = (uint8_t)ev->index;
        settings_save_one("ble/active_profile", &prof_idx, sizeof(prof_idx));
    }
#endif

    LOG_INF("BLE profile %d (connected=%d)", ev->index,
            zmk_ble_profile_is_connected(ev->index));
    refresh_all();
    return 0;
}
ZMK_LISTENER(zmk87_ble_ind, ble_profile_listener);
ZMK_SUBSCRIPTION(zmk87_ble_ind, zmk_ble_active_profile_changed);

static int endpoint_listener(const zmk_event_t *eh) {
    const struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (!ev) {
        return 0;
    }
    LOG_INF("Output endpoint: %s",
            ev->endpoint.transport == ZMK_TRANSPORT_USB ? "USB" : "BLE");

    if (k_uptime_get() >= boot_grace_end &&
        ev->endpoint.transport == ZMK_TRANSPORT_USB && zmk_usb_is_powered()) {
        arm_usb_confirm();
    }

    refresh_all();
    return 0;
}
ZMK_LISTENER(zmk87_ep_ind, endpoint_listener);
ZMK_SUBSCRIPTION(zmk87_ep_ind, zmk_endpoint_changed);

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
ZMK_LISTENER(zmk87_hid_ind, hid_indicators_listener);
ZMK_SUBSCRIPTION(zmk87_hid_ind, zmk_hid_indicators_changed);

/* Runs 50ms after FN+4 (OUT_USB): read the final transport and arm the
 * matching feedback directly. */
static void out_usb_handler(struct k_work *w) {
    ARG_UNUSED(w);
    if (zmk_endpoints_selected().transport == ZMK_TRANSPORT_USB &&
        zmk_usb_is_powered()) {
        arm_usb_confirm();
    }
    last_usb = -1;
    last_profile = -1;
    last_connected = -1;
    refresh_all();
}

/* Key-position listener (FN layer only): channel pairing/blink cues and
 * the FN+4 USB output key. */
static int position_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev =
        as_zmk_position_state_changed(eh);
    if (!ev) {
        return 0;
    }
    if (!zmk_keymap_layer_active(FN_LAYER)) {
        return 0;
    }

    if (ev->position == POS_BT_SEL0 || ev->position == POS_BT_SEL1 ||
        ev->position == POS_BT_SEL2) {
        int64_t now = k_uptime_get();
        if (ev->state) {
            if (pairing_led < 0) {
                blink_deadline = now + BLINK_TIMEOUT_MS;
            }
            pair_pending_pos = ev->position;
            pair_pending_deadline = now + PAIR_HOLD_MS;
            if (pairing_led < 0) {
                last_profile = -1;
            }
        } else {
            if (pair_pending_pos == ev->position) {
                pair_pending_pos = -1;
            }
        }
        refresh_all();
        return 0;
    }

    if (ev->position == POS_OUT_USB && ev->state) {
        k_work_reschedule(&out_usb_work, K_MSEC(50));
        return 0;
    }

    return 0;
}
ZMK_LISTENER(zmk87_pos_ind, position_listener);
ZMK_SUBSCRIPTION(zmk87_pos_ind, zmk_position_state_changed);

/* Deep sleep (System OFF) parking.
 *
 * The nRF52 GPIO block keeps its pin configuration and output value in
 * SYSTEM OFF. An active-low LED left driven LOW would stay ON through the
 * whole sleep. On entering ZMK_ACTIVITY_SLEEP: stop periodic work, drive
 * every LED HIGH (off) via the led driver, then reconfigure each pin as a
 * pulled-up input on its OWN gpio port (no SENSE - LED pins must not be
 * wake sources). Wake-up from System OFF is a full reset, so init below
 * restores the outputs. */
static int activity_state_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev =
        as_zmk_activity_state_changed(eh);
    if (!ev || ev->state != ZMK_ACTIVITY_SLEEP) {
        return 0;
    }

    k_work_cancel_delayable(&tick_work);
    k_work_cancel_delayable(&out_usb_work);

    for (int i = 0; i < LED_COUNT; i++) {
        led_off(led_dev, i);
        const struct device *gpio = led_pins[i].get_port();
        if (device_is_ready(gpio)) {
            gpio_pin_configure(gpio, led_pins[i].pin,
                               GPIO_INPUT | GPIO_PULL_UP);
        }
    }

    LOG_INF("CZM ZMK87 LEDs parked for System OFF (deep sleep)");
    return 0;
}
ZMK_LISTENER(zmk87_activity_ind, activity_state_listener);
ZMK_SUBSCRIPTION(zmk87_activity_ind, zmk_activity_state_changed);

/* ---- init ---- */
static int zmk87_led_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("gpio-leds device not ready");
        return -ENODEV;
    }

    for (int i = 0; i < LED_COUNT; i++) {
        led_off(led_dev, i);
    }
    boot_grace_end = k_uptime_get() + BOOT_GRACE_MS;
    k_work_init_delayable(&tick_work, tick_handler);
    k_work_init_delayable(&out_usb_work, out_usb_handler);

    refresh_all();

    last_evt_profile = zmk_ble_active_profile_index();

    k_work_schedule(&tick_work, K_MSEC(TICK_MS));

    LOG_INF("CZM ZMK87 LEDs ready: CAPS=P0.20 SL=P0.22 "
            "BLE1=P1.00 BLE2=P1.02 BLE3=P1.04 USB=P1.06");
    return 0;
}
SYS_INIT(zmk87_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
