std_msgs/Header header

# Hand identification
uint8 hand              # 0=left, 1=right

# Frame metadata
uint16 sequence         # Frame counter, wraps at 65535
uint32 device_timestamp # Milliseconds since device boot

# Pressure matrix (24 rows x 32 columns, row-major, flattened)
# Value range: 0-2135. Lower value = higher pressure. 2135 = no pressure.
int16[768] pressure
