#if !defined(ZIGBEE_ZED_H)
#define ZIGBEE_ZED_H

/** Start the Zigbee End Device stack and begin TX after joining. */
void zigbee_zed_init(void);

/**
 * Block until the 1000-packet Zigbee test batch completes.
 * Call before starting BLE so both radios never run simultaneously.
 */
void zigbee_zed_wait_batch_done(void);

#endif // ZIGBEE_ZED_H
