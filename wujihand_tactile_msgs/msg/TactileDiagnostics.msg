std_msgs/Header header

# Snapshot of firmware-side counters (spec §3.2.1, GET_DIAGNOSTICS).
uint32 uptime_ms          # Device uptime in ms (~49 day wrap)
uint32 frame_count        # Total data frames completed since boot/reset
uint32 crc_err_count      # Command-channel CRC errors observed by firmware
uint32 dropout_count      # Data frames the firmware failed to push to USB
uint16 usb_reset_count    # USB resets observed by the device
