/*
 * DIAGNOSTIC ONLY: unconditionally enters System OFF 30 seconds after boot,
 * bypassing activity timers and USB state. Used to verify the poweroff path
 * itself works on this board. Wakeup via keys may not be configured here;
 * reset by power-cycle / bootloader double-tap.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(diag_force_off, LOG_LEVEL_INF);

static void force_off_timer_cb(struct k_timer *timer) {
    ARG_UNUSED(timer);
    LOG_INF("DIAG: forcing sys_poweroff() now");
    sys_poweroff();
    LOG_INF("DIAG: sys_poweroff() returned - unexpected, continuing");
}

K_TIMER_DEFINE(force_off_timer, force_off_timer_cb, NULL);

static int diag_force_off_init(void) {
    k_timer_start(&force_off_timer, K_SECONDS(30), K_NO_WAIT);
    LOG_INF("DIAG: force poweroff armed for 30s after boot");
    return 0;
}

SYS_INIT(diag_force_off_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
