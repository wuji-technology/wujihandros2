std_msgs/Header header

# Host-side connection health. False after the SDK fires the once-only
# disconnect callback (USB cable yanked, device dropped off the bus,
# firmware reset). When false, the firmware-side counters below are
# stale and zero: the driver no longer polls, but keeps publishing this
# topic so external supervisors (launch respawn, systemd, health probes)
# can detect the zombie state. The driver does NOT auto-reconnect;
# external orchestration is expected to restart the node.
bool connected

# Snapshot of firmware-side counters (spec §3.2.1, GET_DIAGNOSTICS).
# All zero when `connected` is false.
uint32 uptime_ms          # Device uptime in ms (~49 day wrap)
uint32 frame_count        # Total data frames completed since boot/reset
uint32 crc_err_count      # Command-channel CRC errors observed by firmware
uint32 dropout_count      # Data frames the firmware failed to push to USB
uint16 usb_reset_count    # USB resets observed by the device
