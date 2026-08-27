#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/ble.h>
#include <zmk/rgb_underglow.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define STRIP_NODE      DT_NODELABEL(led_strip)
#define STRIP_LENGTH    DT_PROP(STRIP_NODE, chain_length)
#define PROFILE_COUNT   3
#define INDICATOR_BASE  0    /* LED 0/1/2 correspond to keys 1/2/3 (BT_SEL) */
#define BLINK_PERIOD_MS 300
#define SOLID_HOLD_MS   2000

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

static struct led_rgb pixels[STRIP_LENGTH];

/* BLE indicator state per profile */
enum ind_state {
    IND_OFF = 0,
    IND_BLINK,
    IND_SOLID,
};

static enum ind_state profile_state[PROFILE_COUNT] = { IND_OFF, IND_OFF, IND_OFF };
static bool blink_on;
static bool ug_was_on;
static bool indicator_active;
static struct k_work_delayable ind_work;

/* Bright blue for indicator */
static const struct led_rgb blue_on  = { .r = 0x00, .g = 0x00, .b = 0x80 };
static const struct led_rgb blue_dim = { .r = 0x00, .g = 0x00, .b = 0x20 };
static const struct led_rgb black    = { .r = 0x00, .g = 0x00, .b = 0x00 };

static int strip_update(void) {
    return led_strip_update_rgb(strip, pixels, STRIP_LENGTH);
}

static void fill_all(const struct led_rgb *c) {
    for (int i = 0; i < STRIP_LENGTH; i++) {
        pixels[i] = *c;
    }
}

static void indicator_render(void) {
    if (!indicator_active) return;

    /* Start all black */
    fill_all(&black);

    for (int p = 0; p < PROFILE_COUNT; p++) {
        int idx = INDICATOR_BASE + p;
        if (idx >= STRIP_LENGTH) continue;
        switch (profile_state[p]) {
        case IND_BLINK:
            pixels[idx] = blink_on ? blue_on : blue_dim;
            break;
        case IND_SOLID:
            pixels[idx] = blue_on;
            break;
        case IND_OFF:
            break;
        }
    }
    strip_update();
}

static void restore_underglow(void) {
    if (!indicator_active) return;
    indicator_active = false;
    fill_all(&black);
    strip_update();
    if (ug_was_on) {
        zmk_rgb_underglow_on();
    }
}

static void ind_work_handler(struct k_work *work) {
    bool any_active = false;

    for (int p = 0; p < PROFILE_COUNT; p++) {
        if (profile_state[p] == IND_SOLID) {
            profile_state[p] = IND_OFF;
        }
        if (profile_state[p] != IND_OFF) {
            any_active = true;
        }
    }

    if (!any_active) {
        restore_underglow();
        return;
    }

    blink_on = !blink_on;
    indicator_render();
    k_work_reschedule(&ind_work, K_MSEC(BLINK_PERIOD_MS));
}

static int enter_indicator_mode(void) {
    if (indicator_active) return 0;

    bool ug_state = false;
    zmk_rgb_underglow_get_state(&ug_state);
    ug_was_on = ug_state;
    zmk_rgb_underglow_off();

    indicator_active = true;
    blink_on = true;

    fill_all(&black);
    return strip_update();
}

static int ble_profile_changed_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);

    if (!ev || ev->index >= PROFILE_COUNT) return 0;
    if (!device_is_ready(strip)) return 0;

    if (zmk_ble_profile_is_connected(ev->index)) {
        profile_state[ev->index] = IND_SOLID;
    } else {
        profile_state[ev->index] = IND_BLINK;
    }

    enter_indicator_mode();
    indicator_render();

    if (profile_state[ev->index] == IND_SOLID) {
        k_work_reschedule(&ind_work, K_MSEC(SOLID_HOLD_MS));
    } else {
        k_work_reschedule(&ind_work, K_MSEC(BLINK_PERIOD_MS));
    }

    return 0;
}

ZMK_LISTENER(ble_indicator, ble_profile_changed_listener);
ZMK_SUBSCRIPTION(ble_indicator, zmk_ble_active_profile_changed);

static int ble_indicator_init(void) {
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip not ready");
        return -ENODEV;
    }
    k_work_init_delayable(&ind_work, ind_work_handler);
    fill_all(&black);
    return 0;
}

SYS_INIT(ble_indicator_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
