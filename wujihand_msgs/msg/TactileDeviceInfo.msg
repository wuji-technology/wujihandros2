std_msgs/Header header

# Identity from device-resident TBIM and firmware build (spec §3.1).
string serial             # Up to 24 ASCII chars from TBIM, e.g. "WT1KA00260428001"
uint8[4] hw_revision      # major, minor, patch, variant
uint8[4] fw_version       # major, minor, patch, pre (last byte is ASCII)
uint8 handedness          # 0=left, 1=right
string git_short_sha      # Up to 8 ASCII chars, e.g. "efb07c8"
