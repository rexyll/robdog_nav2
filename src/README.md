# ROS 2 source workspace

The first ROS 2 package has been scaffolded:

- robodog_lite3_bridge: read-only state topic mock publisher and topic freshness diagnostics. It does not connect to the vendor SDK and never publishes cmd_vel.

Additional packages planned after hardware interface and safety contracts are confirmed:
- robodog_description
- robodog_bringup
- robodog_localization
- robodog_stereo_perception
- robodog_obstacle_perception
- robodog_nav2_bringup
- robodog_cmd_adapter
- robodog_diagnostics
- robodog_interfaces (only if standard ROS messages are insufficient)

Build and test details for the initial package are in src/robodog_lite3_bridge/README.md.

Do not place vendor SDK binaries, undocumented firmware, or model weights with unclear redistribution terms into this public source tree.
