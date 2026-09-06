/*
 * DIAGNOSTIC ONLY:
 *  - Scans all 17 direct-key GPIO pins once a second. Any pin reading LOW
 *    with no key pressed is a "ghost" (short/leak/noise) and would wake the
 *    chip instantly out of System OFF.
 *  - Ghost positions are shown as a 5-bit binary code on the 5 LEDs:
 *      bit0 = NumLock LED (P0.22), bit1 = ch1 (P0.12), bit2 = ch2 (P0.04),
 *      bit3 = ch3 (P0.26), bit4 = USB LED (P0.08). lit=1.
 *    Multiple ghosts cycle every 2s. All LEDs off = no ghost.
 *  - After 30s, enters System OFF unconditionally (all wakeup sources are
 *    disabled in the overlay for this build). Wake by power-cycling.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(diag_off, LOG_LEVEL_INF);

#define KEY_COUNT 17

static const struct {
    const struct device *port;
    uint8_t pin;
} keys[KEY_COUNT] = {
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 10},  /* 0  NUM   */
    {DEVICE_DT_GET(DT_NODELABEL(gpio1)),  9},  /* 1  /     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 17},  /* 2  *     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 13},  /* 3  -     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 24},  /* 4  7     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)),  5},  /* 5  8     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 15},  /* 6  9     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)),  9},  /* 7  +     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)),  7},  /* 8  4     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)),  2},  /* 9  5     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 30},  /* 10 6     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 11},  /* 11 Ent   */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 28},  /* 12 1     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 13},  /* 13 2     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 29},  /* 14 3     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)),  3},  /* 15 0     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 10},  /* 16 .     */
};

/* LEDs: index = bit number, active-low (drive 0 = on) */
static const struct {
    const struct device *port;
    uint8_t pin;
} leds[5] = {
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 22},  /* bit0 NumLock */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 12},  /* bit1 ch1     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)),  4},  /* bit2 ch2     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 26},  /* bit3 ch3     */
    {DEVICE_DT_GET(DT_NODELABEL(gpio0)),  8},  /* bit4 USB     */
};

static uint8_t ghost_list[KEY_COUNT];
static uint8_t ghost_count;
static uint8_t ghost_show_idx;

static void leds_show(uint8_t value) {
    for (int b = 0; b < 5; b++) {
        /* active-low: bit set -> drive low (on) */
        gpio_pin_set(leds[b].port, leds[b].pin, (value & (1 << b)) ? 0 : 1);
    }
}

static void scan_work_cb(struct k_work *w) {
    ARG_UNUSED(w);
    ghost_count = 0;
    for (int i = 0; i < KEY_COUNT; i++) {
        int v = gpio_pin_get(keys[i].port, keys[i].pin);
        if (v == 0) {  /* active-low: 0 = pressed / ghost */
            ghost_list[ghost_count++] = i;
            LOG_INF("DIAG: ghost key on position %d", i);
        }
    }
    if (ghost_count == 0) {
        leds_show(0);
    } else {
        leds_show(ghost_list[ghost_show_idx % ghost_count]);
        ghost_show_idx++;
    }
}

K_WORK_DELAYABLE_DEFINE(scan_work, scan_work_cb);

static void scan_timer_cb(struct k_timer *t) {
    ARG_UNUSED(t);
    k_work_reschedule(&scan_work, K_NO_WAIT);
}

static void off_timer_cb(struct k_timer *t) {
    ARG_UNUSED(t);
    LOG_INF("DIAG: forcing sys_poweroff()");
    sys_poweroff();
}

K_TIMER_DEFINE(scan_timer, scan_timer_cb, NULL);
K_TIMER_DEFINE(off_timer, off_timer_cb, NULL);

static int diag_init(void) {
    for (int b = 0; b < 5; b++) {
        gpio_pin_configure(leds[b].port, leds[b].pin, GPIO_OUTPUT_HIGH);
    }
    leds_show(0);

    k_timer_start(&scan_timer, K_SECONDS(2), K_SECONDS(2));
    k_timer_start(&off_timer, K_SECONDS(30), K_NO_WAIT);
    LOG_INF("DIAG: ghost scan + force poweroff armed");
    return 0;
}

SYS_INIT(diag_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
