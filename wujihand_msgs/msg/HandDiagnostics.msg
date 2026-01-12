std_msgs/Header header

# Hand identification
string handedness  # "left" or "right"

# System status
float32 system_temperature
float32 input_voltage

# Joint diagnostics (20 joints: 5 fingers x 4 joints)
# Order: thumb(0-3), index(4-7), middle(8-11), ring(12-15), little(16-19)
# Each finger: mcp_abd, mcp_flex, pip, dip
float32[20] joint_temperatures
uint32[20] error_codes
bool[20] enabled
