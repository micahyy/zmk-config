/*
 * DZ17 LED strip proxy — multi-position BLE/USB indicators
 *
 * Sits between ZMK underglow and the real WS2812 strip.
 * ZMK writes animation frames to us; we overlay indicator colors
 * on specific pixels, then forward everything to the real strip.
 *
 * RGB animations keep running normally while indicators show independently.
 */
#define DT_DRV_COMPAT czmao_dz17_indicators

#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define BLINK_PERIOD_MS 500
#define SOLID_HOLD_MS   2000
#define RENAME_DELAY_MS 100
#define USB_SLOT        3
#define NUM_INDICATORS  4

enum phase {
    PHASE_OFF = 0,
    PHASE_BLINK,
    PHASE_SOLID,
};

struct indicator_state {
    uint8_t index;
    uint32_t color;
    enum phase phase;
    bool blink_on;
};

struct proxy_config {
    const struct device *target;
    uint16_t length;
};

/* ---- runtime ---- */
static struct indicator_state ind_states[NUM_INDICATORS];
static struct k_work_delayable blink_work;
static struct k_work_delayable solid_off_work;
static struct k_work_delayable rename_work;
static uint8_t pending_rename_profile = 0xFF;
static int solid_off_slot = -1;

/* ---- helpers ---- */
static inline struct led_rgb rgb_from_u32(uint32_t v) {
    return (struct led_rgb){
        .r = (v >> 16) & 0xFF,
        .g = (v >> 8) & 0xFF,
        .b = v & 0xFF,
    };
}

static void set_indicator(uint8_t slot, enum phase phase) {
    if (slot >= NUM_INDICATORS) return;
    ind_states[slot].phase = phase;
    ind_states[slot].blink_on = true;

    if (solid_off_slot == slot) {
        k_work_cancel_delayable(&solid_off_work);
        solid_off_slot = -1;
    }
    if (phase == PHASE_SOLID) {
        solid_off_slot = slot;
        k_work_reschedule(&solid_off_work, K_MSEC(SOLID_HOLD_MS));
    }
}

/* ---- work handlers ---- */
static void blink_handler(struct k_work *w) {
    bool any = false;
    for (int i = 0; i < NUM_INDICATORS; i++) {
        if (ind_states[i].phase == PHASE_BLINK) {
            ind_states[i].blink_on = !ind_states[i].blink_on;
            any = true;
        }
    }
    if (any) k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
}

static void solid_off_handler(struct k_work *w) {
    if (solid_off_slot >= 0 && solid_off_slot < NUM_INDICATORS) {
        ind_states[solid_off_slot].phase = PHASE_OFF;
    }
    solid_off_slot = -1;
}

static void rename_handler(struct k_work *w) {
    if (pending_rename_profile > 2) return;
    static char name[16];
    snprintf(name, sizeof(name), "czm_ble_%d", pending_rename_profile + 1);
    int ret = zmk_ble_set_device_name(name);
    LOG_INF("BLE rename -> '%s' ret=%d", name, ret);
    pending_rename_profile = 0xFF;
}

/* ---- led_strip API ---- */
static int proxy_update_rgb(const struct device *dev, struct led_rgb *pixels, size_t num) {
    const struct proxy_config *cfg = dev->config;

    for (int i = 0; i < NUM_INDICATORS; i++) {
        struct indicator_state *is = &ind_states[i];
        if (is->index >= num || is->phase == PHASE_OFF) continue;

        if (is->phase == PHASE_SOLID || is->blink_on) {
            pixels[is->index] = rgb_from_u32(is->color);
        } else {
            pixels[is->index] = (struct led_rgb){0};
        }
    }
    return led_strip_update_rgb(cfg->target, pixels, num);
}

static int proxy_init(const struct device *dev) {
    const struct proxy_config *cfg = dev->config;

    /* Wait for target LED strip to be ready */
    if (!device_is_ready(cfg->target)) {
        LOG_ERR("target LED strip not ready, retrying...");
        return -ENODEV;
    }

    /* Read indicator positions/colors from DTS using explicit indices */
    ind_states[0].index = DT_INST_PROP_BY_IDX(0, indicator_indices, 0);
    ind_states[1].index = DT_INST_PROP_BY_IDX(0, indicator_indices, 1);
    ind_states[2].index = DT_INST_PROP_BY_IDX(0, indicator_indices, 2);
    ind_states[3].index = DT_INST_PROP_BY_IDX(0, indicator_indices, 3);

    ind_states[0].color = DT_INST_PROP_BY_IDX(0, indicator_colors, 0);
    ind_states[1].color = DT_INST_PROP_BY_IDX(0, indicator_colors, 1);
    ind_states[2].color = DT_INST_PROP_BY_IDX(0, indicator_colors, 2);
    ind_states[3].color = DT_INST_PROP_BY_IDX(0, indicator_colors, 3);

    for (int i = 0; i < NUM_INDICATORS; i++) {
        ind_states[i].phase = PHASE_OFF;
        ind_states[i].blink_on = false;
    }

    k_work_init_delayable(&blink_work, blink_handler);
    k_work_init_delayable(&solid_off_work, solid_off_handler);
    k_work_init_delayable(&rename_work, rename_handler);

    LOG_INF("DZ17 indicators: idx=[%d,%d,%d,%d] colors=[0x%06X,0x%06X,0x%06X,0x%06X]",
            ind_states[0].index, ind_states[1].index,
            ind_states[2].index, ind_states[3].index,
            ind_states[0].color, ind_states[1].color,
            ind_states[2].color, ind_states[3].color);
    return 0;
}

static const struct led_strip_driver_api proxy_api = {
    .update_rgb = proxy_update_rgb,
};

static struct proxy_config cfg0 = {
    .target = DEVICE_DT_GET(DT_INST_PHANDLE(0, target)),
    .length = DT_INST_PROP(0, chain_length),
};

/* APPLICATION level runs after all POST_KERNEL devices (including WS2812 strip at 35) */
DEVICE_DT_INST_DEFINE(0, proxy_init, NULL, NULL, &cfg0,
                      APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &proxy_api);

/* ---- events ---- */
static int ble_profile_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev =
        as_zmk_ble_active_profile_changed(eh);
    if (!ev || ev->index > 2) return 0;

    LOG_INF("BLE profile %d", ev->index);

    /* Defer rename to avoid BLE stack lock contention */
    pending_rename_profile = ev->index;
    k_work_reschedule(&rename_work, K_MSEC(RENAME_DELAY_MS));

    /* Clear all BLE indicators (slots 0..2); USB is slot 3. */
    for (int i = 0; i < 3; i++) set_indicator(i, PHASE_OFF);

    if (zmk_ble_profile_is_connected(ev->index)) {
        set_indicator(ev->index, PHASE_SOLID);
    } else {
        set_indicator(ev->index, PHASE_BLINK);
        k_work_reschedule(&blink_work, K_MSEC(BLINK_PERIOD_MS));
    }
    return 0;
}
ZMK_LISTENER(dz17_ble_ind, ble_profile_listener);
ZMK_SUBSCRIPTION(dz17_ble_ind, zmk_ble_active_profile_changed);

static int endpoint_listener(const zmk_event_t *eh) {
    const struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (!ev) return 0;

    if (ev->endpoint.transport == ZMK_TRANSPORT_USB) {
        LOG_INF("USB selected");
        set_indicator(USB_SLOT, PHASE_SOLID);
        for (int i = 0; i < 3; i++) set_indicator(i, PHASE_OFF);
    } else {
        set_indicator(USB_SLOT, PHASE_OFF);
    }
    return 0;
}
ZMK_LISTENER(dz17_ep_ind, endpoint_listener);
ZMK_SUBSCRIPTION(dz17_ep_ind, zmk_endpoint_changed);
