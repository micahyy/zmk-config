# Custom BLE indicator: blinks LED 12/13/14 during advertising,
# solid blue for 2s on connect, then restores underglow.
target_sources(app PRIVATE src/ble_indicator.c)
