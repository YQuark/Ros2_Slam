# Command authority

`chassis/host_motion_command` carries one Host candidate. `host_subsource` is diagnostic metadata and never maps to a firmware physical source. Bridge always encodes Upper-v3 mode `2`, which firmware maps to `COMMAND_SOURCE_HOST`. An upper disable withdraws Host only; it cannot stop a higher-priority or independently selected firmware source. Physical stop evidence comes from firmware STATUS.
