# Legacy v2 test fixtures

`bridge_node.py`, `protocol_v2.py`, `command_guard.py`, `diagnostics.py` and their historical tests are retained in the source tree for the v0.4.x migration window and beta4 golden-vector regression only. CMake does not install or execute them as package runtime. The sole installed executable is `bridge_node_v3.py` renamed to `bridge_node`.

These fixtures must not be imported by production code and are scheduled for removal in v0.5.0.
