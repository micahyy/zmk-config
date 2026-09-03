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
 *   - USB selected: green LED solid for 2s, then off; blue LEDs stay off
 *     EXCEPT a 2s blink cue on the newly selected BLE channel when
 *     switching profiles (real key press only, so the FN+1/2/3 keys give
 *     visible feedback while plugged in; power-on sync never flashes).
 *   - Switching profile always re-triggers the cue/confirmation/blink.
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
#define BLINK_HALF_MS  500   /* blink toggle interval (~1 Hz)        */
#define CUE_HALF_MS    250   /* cue blink toggle (2 Hz). MUST equal TICK_MS:
                             * the 250ms sampling then toggles every tick and
                             * can never alias to a steady on/off. */
#define CONFIRM_MS     2000  /* solid / cue confirmation window       */

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

/* Momentary solid-confirmation window: confirm_led stays on until deadline. */
static int confirm_led = -1;
static int64_t confirm_deadline = 0;

/* Profile-switch cue: fast-blink the target blue LED for CUE_MS regardless
 * of transport, so the BLE1/2/3 keys always give visible feedback. If the
 * green USB confirmation is showing when the cue is armed, the cue is
 * QUEUED to start when the green window ends (cue_start) instead of
 * overlapping it - the <=2s queue delay is imperceptible in use. */
static int cue_led = -1;
static int64_t cue_start = 0;
static int64_t cue_deadline = 0;

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

/* Arm the green USB confirmation (2s). Any in-flight/queued blue cue is
 * pushed to start after the green window so the two LEDs can never light
 * at the same time. */
static void arm_usb_confirm(void) {
    int64_t now = k_uptime_get();
    confirm_led = LED_USB;
    confirm_deadline = now + CONFIRM_MS;
    if (cue_led >= 0) {
        cue_start = confirm_deadline;
        cue_deadline = cue_start + CONFIRM_MS;
    }
}

/* Arm a blue profile-switch cue (2s fast blink). If the green USB
 * confirmation is still showing, queue the cue to begin when it ends
 * rather than overlapping it; the feedback is never dropped. */
static void arm_blue_cue(int led) {
    int64_t now = k_uptime_get();
    cue_led = led;
    cue_start = now;
    if (confirm_led == LED_USB && now < confirm_deadline) {
        cue_start = confirm_deadline;
    }
    cue_deadline = cue_start + CONFIRM_MS;
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

    /* Mode follows the ACTUAL endpoint (so FN+N4 OUT_TOG switches the LEDs
     * instantly) - EXCEPT for a short window after VBUS appears: USB
     * enumeration takes a few hundred ms during which the endpoint is
     * still BLE and the blue LED would blink for a beat. While VBUS is
     * present and within that grace window, assume USB. */
    int vbus_powered = zmk_usb_is_powered() ? 1 : 0;
    if (vbus_powered && !vbus_was_powered) {
        vbus_grace_end = now + VBUS_GRACE_MS;
    }
    vbus_was_powered = vbus_powered;

    struct zmk_endpoint_instance ep = zmk_endpoints_selected();
    int usb = (ep.transport == ZMK_TRANSPORT_USB) ? 1 : 0;
    if (!usb && vbus_powered && now < vbus_grace_end) {
        usb = 1;
    }

    int cur = zmk_ble_active_profile_index();
    int connected = (!usb && cur >= 0 && cur < BLE_COUNT &&
                     zmk_ble_profile_is_connected((uint8_t)cur)) ? 1 : 0;

    /* ---- edges: (re)arm the 2s confirmation / cue windows ---- */
    if (usb != last_usb) {
        if (usb) {
            confirm_led = LED_USB;
            confirm_deadline = now + CONFIRM_MS;
            /* Green and the blue profile cue are mutually exclusive:
             * arriving in USB kills any in-flight blue cue so the two
             * can never light at the same time. */
            cue_led = -1;
        } else {
            /* Just entered BLE: force profile/connected edges below to
             * re-arm the confirmation for the active BLE profile. */
            confirm_led = -1;
            last_profile = -1;
            last_connected = -1;
        }
        last_usb = usb;
    }

    /* Profile / connection edges drive only the BLE-mode solid
     * confirmation. The USB cue is armed by the profile-changed listener
     * (real key presses), never here, so the initial boot state sync
     * (e.g. profile 0 at power-on) does not flash a blue LED. */
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
    } else {
        /* Keep edge trackers current while in USB so the next BLE session
         * starts without a stale edge. */
        last_profile = cur;
        last_connected = connected;
    }

    /* ---- desired state of every LED ---- */
    bool on[LED_COUNT] = {0};

    /* NumLock: read straight from the current HID report each tick. */
    zmk_hid_indicators_t ind = zmk_hid_indicators_get_current_profile();
    on[LED_NUM] = (ind & HID_LED_NUM_LOCK) != 0;

    if (usb) {
        /* USB mode (actual endpoint or VBUS grace after plug-in): green
         * for the 2s confirmation window. Blue LEDs stay off except the
         * 2s cue on a profile switch. */
        if (confirm_led == LED_USB && now < confirm_deadline) {
            on[LED_USB] = true;
        }
    } else if (cur >= 0 && cur < BLE_COUNT) {
        int led = LED_BLE0 + cur;
        if (!connected) {
            /* Advertising / disconnected: slow blink. */
            on[led] = ((now / BLINK_HALF_MS) % 2) == 0;
        } else if (confirm_led == led && now < confirm_deadline) {
            /* Connected: solid 2s confirmation, then off. */
            on[led] = true;
        }
    }

    /* Profile-switch cue: fast-blink overrides the steady off state
     * (applies in USB too; in BLE it's harmless while already blinking).
     * A cue armed while the green confirmation is showing stays queued
     * (now < cue_start) and never overlaps it. */
    if (cue_led >= LED_BLE0 && cue_led <= LED_BLE2 &&
        now >= cue_start && now < cue_deadline) {
        on[cue_led] = ((now / CUE_HALF_MS) % 2) == 0;
    }

    /* ---- force-write all pins (idempotent; fixes any drift) ---- */
    for (int i = 0; i < LED_COUNT; i++) {
        led_set(i, on[i]);
    }

    if (confirm_led >= 0 && now >= confirm_deadline) {
        confirm_led = -1;
    }
    if (cue_led >= 0 && now >= cue_deadline) {
        cue_led = -1;
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

    /* Rename the advertiser so hosts show czm_ble_1/2/3 per channel. */
    static char name[16];
    snprintf(name, sizeof(name), "czm_ble_%d", ev->index + 1);
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

    /* USB cue: a real profile switch while plugged in blinks the target
     * blue LED for 2s (FN+1/2/3 feedback). Connect/disconnect events must
     * NOT fire it - e.g. plugging USB in while a BLE host is still
     * connected raises a profile-changed event with the same index, which
     * previously flashed BLE1 at USB insert. In BLE mode the blink/solid
     * logic in refresh_all already covers switches, so cue is USB-only. */
    if (real_switch) {
        struct zmk_endpoint_instance cur_ep = zmk_endpoints_selected();
        /* Cue in USB mode only (real USB transport with VBUS present;
         * endpoint==USB without VBUS only happens during enumeration). */
        if (cur_ep.transport == ZMK_TRANSPORT_USB && zmk_usb_is_powered()) {
            arm_blue_cue(LED_BLE0 + ev->index);
        }
    }

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

/* Key-position listener: ZMK's &bt BT_SEL <already-active-profile> fires
 * NO profile-changed event (it bails out as "no change"), so pressing
 * FN+N1 while BLE1 is already selected gave zero feedback. Give the
 * cue directly on the physical key press (FN layer held), regardless of
 * whether the profile actually changed. Re-arming the same cue is
 * harmless. FN+N4 (OUT_TOG) arms the delayed transport check above. */
static int position_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev =
        as_zmk_position_state_changed(eh);
    if (!ev || !ev->state) {
        return 0;
    }
    if (!zmk_keymap_layer_active(FN_LAYER)) {
        return 0;
    }

    switch (ev->position) {
    case POS_BT_SEL0:
        arm_blue_cue(LED_BLE0);
        break;
    case POS_BT_SEL1:
        arm_blue_cue(LED_BLE1);
        break;
    case POS_BT_SEL2:
        arm_blue_cue(LED_BLE2);
        break;
    case POS_OUT_TOG:
        /* Event fires before the output behavior runs; check after it. */
        k_work_reschedule(&out_tog_work, K_MSEC(50));
        return 0;
    default:
        return 0;
    }

    refresh_all();
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
