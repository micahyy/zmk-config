# Custom BLE indicator: on profile switch, blink LED 0/1/2 (keys 1/2/3)
# while advertising, solid blue for 2s on connect, then restores underglow.
# Also renames BLE device to czmao_dz17_1/2/3 per active profile.
target_sources(app PRIVATE src/ble_indicator.c)
