std_msgs/Header header

# Hand identification
uint8 hand              # 0=left, 1=right

# Frame metadata
uint16 sequence         # Frame counter, wraps at 65535
uint32 device_timestamp_ms  # Milliseconds since device boot

# Pressure matrix (24 rows x 32 columns, row-major, flattened).
# Value range [0.0, 1.0]: 0.0 = no contact, 1.0 = maximum contact.
# NaN marks an invalid cell (zone 0 — not part of any finger / palm region);
# subscribers must skip such cells via std::isnan / numpy.isnan.
float32[768] pressure
