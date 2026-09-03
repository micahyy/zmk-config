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
 * Behavior (Logitech K375s style; channel LEDs are momentary):
 *   - Channel key FN+1/2/3 TAP   = switch / reconnect channel. The channel
 *     LED SLOW blinks (~1 Hz) for ~30s while trying to reconnect, then
 *     turns off; connecting lights it solid for 3s.
 *   - Channel key FN+1/2/3 HOLD (>=1s) = pairing mode: clears that
 *     channel's bond and opens it for a new host. The LED FAST blinks
 *     (~2.5 Hz, no timeout) until a host pairs, then solid 3s and off.
 *   - USB selected: USB LED solid for 3s, then off. Blue channel LEDs
 *     keep showing BLE state (advertising/pairing continues over USB) but
 *     are suppressed during the green 3s window so two LEDs never overlap.
 *   - Inserting the USB cable in ANY state forces the transport to USB.
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
#define BLINK_HALF_MS  500   /* reconnect blink toggle interval (~1 Hz) */
#define PAIR_HALF_MS   200   /* pairing-mode fast blink toggle (~2.5 Hz) */
#define PAIR_HOLD_MS   1000  /* hold a channel key this long to enter
                             * pairing mode (matches the hold-tap term in
                             * the keymap). */
#define CONFIRM_MS     3000  /* solid confirmation / cue window (3s)  */
#define BLINK_TIMEOUT_MS 30000 /* unconnected/advertising: blink ~30s,
                                * then LED off until a state change
                                * (connect / profile switch / BLE re-entry). */

#define LED_NUM   0   /* P0.22 */
#define LED_BLE0  1   /* P0.12 */
#define LED_BLE1  2   /* P0.04 */
#define LED_BLE2  3   /* P0.26 */
#define LED_USB   4   /* P0.08 */
#define LED_COUNT 5
#define BLE_COUNT 3

/* FN layer + physical key positions of the BLE profile / output keys.
 * FN+N1/N2/N3 = BT_SEL 0/1/2 -> positions 12/13/14; FN+N4 = OUT_TOG -> 8. */
#define FN_LAYER    1
#define POS_OUT_TOG 8
#define POS_BT_SEL0 12
#define POS_BT_SEL1 13
#define POS_BT_SEL2 14

/* VBUS plug-in grace: after power (or VBUS) is detected while the
 * endpoint is still enumerating over BLE, assume USB for this long so
 * the LEDs follow the cable immediately instead of blinking blue for
 * the ~500ms enumeration window. */
#define VBUS_GRACE_MS 1500

/* HID LED report bitmasks (USB HID Usage Tables, LED report) */
#define HID_LED_NUM_LOCK 0x01

static const struct device *const led_dev = DEVICE_DT_GET(DT_INST(0, gpio_leds));

static struct k_work_delayable tick_work;
static struct k_work_delayable out_tog_work;

/* Edge-detection state (initialised to impossible values so the first
 * tick always sees the current state as a fresh edge). */
static int last_usb      = -1;   /* 1 = USB transport, 0 = BLE */
static int last_profile  = -1;
static int last_connected = -1;  /* -1 unknown, 0/1 */

/* Green USB confirmation window: green LED stays on until deadline. */
static int64_t usb_confirm_deadline = 0;

/* Blue BLE confirmation window: the connected channel's blue LED is solid
 * until deadline (transport independent). */
static int blue_confirm_led = -1;
static int64_t blue_confirm_deadline = 0;

/* Pairing mode (Logitech-style): holding a channel key >= PAIR_HOLD_MS
 * clears that channel's bond and opens it for a new host. The target LED
 * FAST blinks for as long as it stays unconnected; a successful connect
 * ends pairing mode and lights that LED solid for 3s. -1 = not pairing. */
static int pairing_led = -1;

/* Channel key hold-timer: position armed by the down event, fired by the
 * tick when the key is still down PAIR_HOLD_MS later (a tap releases
 * sooner and cancels it). -1 = no channel key currently held. */
static int pair_pending_pos = -1;
static int64_t pair_pending_deadline = 0;

/* Advertising/disconnected blinking runs for ~30s then stops; the
 * deadline is (re)armed whenever the BLE link drops, a different
 * profile is selected or the keyboard leaves USB mode. */
static int64_t blink_deadline = 0;

/* First tick after the boot grace window: if the cable is present,
 * force the transport to USB (a cold boot on USB must end in USB mode). */
static int force_usb_pending = 1;

/* End of the boot grace window (set in init). */
static int64_t boot_grace_end = 0;

/* VBUS plug-in grace: while now < vbus_grace_end a VBUS-present board is
 * treated as USB mode even before enumeration finishes. */
static int64_t vbus_grace_end = 0;
static int vbus_was_powered = 0;

/* Index carried by the last profile-changed event. ZMK raises that event
 * not only on a real key-driven profile switch but also on BLE connect,
 * disconnect and pairing completion (all with the SAME active index); a
 * real switch is the only case where the index actually changes. */
static int last_evt_profile = -1;

static void led_set(int idx, bool on) {
    if (on) {
        led_on(led_dev, idx);
    } else {
        led_off(led_dev, idx);
    }
}

/* Arm the green USB confirmation (3s). */
static void arm_usb_confirm(void) {
    usb_confirm_deadline = k_uptime_get() + CONFIRM_MS;
}

/* Full recompute + force-write of every LED. Safe to call from any context. */
static void refresh_all(void) {
    int64_t now = k_uptime_get();

    /* Boot grace: the USB driver reports VBUS only a few hundred ms after
     * power-up, so early ticks would mistake USB power for battery BLE and
     * blink a blue LED. Keep every LED dark and arm no state until the
     * window ends; the edge trackers (still -1) then treat the real state
     * as a fresh edge and light the correct LED (green on USB, blue on
     * battery). Events during grace also end up here. */
    if (now < boot_grace_end) {
        /* Keep the VBUS baseline current during grace too, otherwise the
         * first post-grace tick would see a false VBUS rising edge and
         * arm the 1.5s plug-in grace, forcing USB mode and swallowing
         * early BLE/USB toggle edges. */
        vbus_was_powered = zmk_usb_is_powered() ? 1 : 0;
        for (int i = 0; i < LED_COUNT; i++) {
            led_set(i, false);
        }
        return;
    }

    /* Cold boot on cable: the board may restore a BLE preference and stay
     * there while plugged in. Right after the boot grace, force the
     * transport to USB if VBUS is present (a hot-plug rising edge below
     * handles the same after boot). */
    if (force_usb_pending) {
        force_usb_pending = 0;
        if (zmk_usb_is_powered()) {
            zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);
        }
    }

    /* Mode follows the ACTUAL endpoint (so FN+N4 OUT_TOG switches the LEDs
     * instantly) - EXCEPT for a short window after VBUS appears: USB
     * enumeration takes a few hundred ms during which the endpoint is
     * still BLE and the blue LED would blink for a beat. While VBUS is
     * present and within that grace window, assume USB. */
    int vbus_powered = zmk_usb_is_powered() ? 1 : 0;
    if (vbus_powered && !vbus_was_powered) {
        vbus_grace_end = now + VBUS_GRACE_MS;
        /* Cable insertion in ANY state forces the transport back to
         * USB (enumeration-ready); the endpoint event/grace then lights
         * the green confirmation. */
        zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);
    }
    vbus_was_powered = vbus_powered;

    struct zmk_endpoint_instance ep = zmk_endpoints_selected();
    int usb = (ep.transport == ZMK_TRANSPORT_USB) ? 1 : 0;
    if (!usb && vbus_powered && now < vbus_grace_end) {
        usb = 1;
    }

    int cur = zmk_ble_active_profile_index();
    /* BLE connection state is tracked regardless of transport: ZMK keeps
     * the BLE link/advertising alive in USB mode too, so pairing can be
     * done with the cable plugged in - the blue LED must then show the
     * same 30s blink / 3s solid feedback as wireless. */
    int connected = (cur >= 0 && cur < BLE_COUNT &&
                     zmk_ble_profile_is_connected((uint8_t)cur)) ? 1 : 0;

    /* ---- pairing hold-timer (armed by channel-key down event) ---- */
    if (pair_pending_pos >= 0 && now >= pair_pending_deadline) {
        /* Held PAIR_HOLD_MS: the keymap hold-tap is about to run BT_SEL +
         * BT_CLR on this channel. Enter pairing mode (the profile-changed
         * event from BT_SEL refreshes the LED immediately after). */
        int idx = pair_pending_pos - POS_BT_SEL0;
        if (idx >= 0 && idx < BLE_COUNT) {
            pairing_led = LED_BLE0 + idx;
            blink_deadline = 0;   /* pairing waits indefinitely for a host */
        }
        pair_pending_pos = -1;
    }

    /* ---- edges: (re)arm the confirmation / blink windows ---- */
    if (usb != last_usb) {
        if (usb) {
            /* Entering USB: green confirmation. Pairing mode is not
             * cancelled - pairing over BLE with the cable in keeps the
             * fast blink (green suppresses it for its 3s window). */
            arm_usb_confirm();
        } else {
            /* Just entered BLE: force profile/connected edges below to
             * re-arm the confirmation / 30s blink for the active profile. */
            last_profile = -1;
            last_connected = -1;
            if (!connected) {
                blink_deadline = now + BLINK_TIMEOUT_MS;
            }
        }
        last_usb = usb;
    }

    /* Profile / connection edges drive the blue channel feedback:
     * switching channels or losing the link (re)starts the 30s reconnect
     * blink; an established connection lights the channel solid for 3s. */
    if (cur != last_profile) {
        /* Switched profile (or first valid profile). Switching AWAY from a
         * pairing channel cancels pairing mode; a fresh 30s reconnect
         * blink starts for the new channel (solid 3s first if it is
         * already connected). */
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
            /* Connected after being on this channel: if it was the pairing
             * channel, the new host bonded successfully - show solid 3s. */
            blue_confirm_led = LED_BLE0 + cur;
            blue_confirm_deadline = now + CONFIRM_MS;
            pairing_led = -1;
        }
    } else if (connected != last_connected) {
        last_connected = connected;
        if (connected && cur >= 0) {
            blue_confirm_led = LED_BLE0 + cur;
            blue_confirm_deadline = now + CONFIRM_MS;
            pairing_led = -1;   /* pair/reconnect succeeded */
        } else {
            blue_confirm_led = -1;   /* dropped -> blink, no solid window */
            if (pairing_led < 0) {
                blink_deadline = now + BLINK_TIMEOUT_MS;
            }
        }
    }

    /* ---- desired state of every LED ---- */
    bool on[LED_COUNT] = {0};

    /* NumLock: read straight from the current HID report each tick. */
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    on[LED_NUM] = (ind & HID_LED_NUM_LOCK) != 0;

    /* Green USB LED: solid for the 3s confirmation window. While it is on,
     * the blue LEDs are suppressed (below) so the two never overlap. */
    bool usb_green = usb && (now < usb_confirm_deadline);
    if (usb_green) {
        on[LED_USB] = true;
    }

    /* Blue channel LEDs (visible in BOTH transports - BLE keeps
     * advertising over USB too), suppressed while the green confirmation
     * is on:
     *   - pairing mode (long-press): FAST blink, no 30s timeout, until a
     *     host bonds; then solid 3s;
     *   - otherwise unconnected: SLOW blink for ~30s, then off;
     *   - connected over BLE: solid 3s confirmation, then off. */
    if (cur >= 0 && cur < BLE_COUNT && !usb_green) {
        int led = LED_BLE0 + cur;
        if (!connected) {
            if (pairing_led == led) {
                on[led] = ((now / PAIR_HALF_MS) % 2) == 0;
            } else if (now < blink_deadline) {
                on[led] = ((now / BLINK_HALF_MS) % 2) == 0;
            }
        } else if (!usb && blue_confirm_led == led &&
                   now < blue_confirm_deadline) {
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

/* ---- events: just kick an immediate refresh (no state carried) ---- */
static int ble_profile_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);
    if (!ev || ev->index >= BLE_COUNT) {
        return 0;
    }

    /* ZMK raises this event on connect/disconnect/pairing too, always with
     * the SAME index; only a real key-driven switch changes the index.
     * The first event after boot (index sync, e.g. a host reconnecting)
     * is treated as a state sync, never as a switch. */
    bool real_switch = (last_evt_profile >= 0) && (last_evt_profile != ev->index);
    last_evt_profile = ev->index;

    /* Rename the advertiser so hosts show czm_dz17_ble_1/2/3 per channel. */
    static char name[20];
    snprintf(name, sizeof(name), "czm_dz17_ble_%d", ev->index + 1);
    zmk_ble_set_device_name(name);

#if defined(CONFIG_SETTINGS)
    /* Persist the active profile immediately on a real switch: ZMK's own
     * save is debounced ~60s (CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE), so
     * switching channels and powering off within a minute would otherwise
     * revert to the old channel at next boot. Key/length must match ZMK
     * ble.c (uint8). */
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
ZMK_LISTENER(dz17_ble_ind, ble_profile_listener);
ZMK_SUBSCRIPTION(dz17_ble_ind, zmk_ble_active_profile_changed);

static int endpoint_listener(const zmk_event_t *eh) {
    const struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (!ev) {
        return 0;
    }
    LOG_INF("Output endpoint: %s",
            ev->endpoint.transport == ZMK_TRANSPORT_USB ? "USB" : "BLE");

    /* Arm the green confirmation directly on the event (which carries the
     * transport) rather than relying solely on the tick edge detection -
     * e.g. OUT_TOG back to USB then reliably lights green for 2s even if
     * tick timing or the VBUS plug-in grace interferes. Boot grace swallows
     * events before the window ends. */
    if (k_uptime_get() >= boot_grace_end &&
        ev->endpoint.transport == ZMK_TRANSPORT_USB && zmk_usb_is_powered()) {
        arm_usb_confirm();
    }

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

/* Runs 50ms after FN+N4 (OUT_TOG): the behavior has by then updated the
 * selected endpoint, so read the final transport and arm the matching
 * feedback directly - no dependency on endpoint-changed event ordering. */
static void out_tog_handler(struct k_work *w) {
    ARG_UNUSED(w);
    if (zmk_endpoints_selected().transport == ZMK_TRANSPORT_USB &&
        zmk_usb_is_powered()) {
        arm_usb_confirm();
    }
    /* BLE side: reset edge trackers so refresh_all re-arms the blue
     * blink/solid confirmation for the active profile. */
    last_usb = -1;
    last_profile = -1;
    last_connected = -1;
    refresh_all();
}

/* Key-position listener (FN layer only):
 *   FN+1/2/3 TAP  -> switch/reconnect (the keymap hold-tap runs BT_SEL;
 *                    if it is the already-active profile ZMK fires no
 *                    event, so start the 30s reconnect blink here).
 *   FN+1/2/3 HOLD -> the tick timer (armed on key-down) flips the LED to
 *                    pairing fast-blink at PAIR_HOLD_MS; the keymap
 *                    hold-tap then runs BT_SEL + BT_CLR to open the bond.
 *   FN+4 (OUT_TOG)-> arm the delayed transport check below. */
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
            /* Key down: start the 30s reconnect blink immediately (covers
             * re-pressing the current profile, which fires no event) and
             * arm the long-press pairing timer. */
            if (pairing_led < 0) {
                blink_deadline = now + BLINK_TIMEOUT_MS;
            }
            pair_pending_pos = ev->position;
            pair_pending_deadline = now + PAIR_HOLD_MS;
            /* The hold-tap runs BT_SEL on release; make sure the edge
             * logic re-evaluates the profile even if it is unchanged. */
            if (pairing_led < 0) {
                last_profile = -1;
            }
        } else {
            /* Key up: a tap (released before PAIR_HOLD_MS) cancels
             * pairing; the hold-tap's BT_SEL drives the LED via the
             * profile-changed event / fresh edge. */
            if (pair_pending_pos == ev->position) {
                pair_pending_pos = -1;
            }
        }
        refresh_all();
        return 0;
    }

    if (ev->position == POS_OUT_TOG && ev->state) {
        /* Event fires before the output behavior runs; check after it. */
        k_work_reschedule(&out_tog_work, K_MSEC(50));
        return 0;
    }

    return 0;
}
ZMK_LISTENER(dz17_pos_ind, position_listener);
ZMK_SUBSCRIPTION(dz17_pos_ind, zmk_position_state_changed);

/* ---- init ---- */
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
    k_work_init_delayable(&out_tog_work, out_tog_handler);

    /* First tick runs immediately and arms the correct window/edges. */
    refresh_all();

    /* Baseline for real-switch detection in the profile listener: whatever
     * profile is active at boot (restored from settings) is "state sync",
     * so the first genuine key press to a different channel still cues. */
    last_evt_profile = zmk_ble_active_profile_index();

    k_work_schedule(&tick_work, K_MSEC(TICK_MS));

    LOG_INF("DZ17 mono LEDs ready: Num=P0.22 BLE1=P0.12 BLE2=P0.04 BLE3=P0.26 USB=P0.08");
    return 0;
}
SYS_INIT(dz17_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
