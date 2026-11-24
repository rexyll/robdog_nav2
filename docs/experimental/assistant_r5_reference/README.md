# R5 experimental reference bundle

This document records the disposition of the assistant-generated R3–R5 prototype artifacts and prevents them from being mistaken for the canonical project workspace.

## Stored artifact

- Archive: **r5_reference.tar.gz**
- SHA-256: **86bcae9b45d4d0cc0a2fc6eb13baa4e0359758314e5c7b0527587987656bd68a**
- Contents: selected pure-Python algorithm cores and unit tests for terrain mapping, RTK quality gating, obstacle tracking, and dynamic-costmap geometry, plus the prototype's test report and integration-status notes.
- The archive preserves the reported 102/102 offline checks from that prototype.

## Integration status and limitations

The bundle is **reference material only**. It is not merged into src/, is not a replacement for this repository's source tree, and has not been built against the repository's pinned ROS 2 / Nav2 distributions.

The R3–R5 workspace used generic package names, provisional topics and message types, and generated ROS wrappers / C++ Costmap plugins. It did not match the repository's current robodog_lite3_bridge, Lite3 vendor-interface boundary, Humble-first reproduction path, Point-LIO pinned source workflow, or hardware-admission gates. The prototype's 102/102 result represents offline Python/configuration/source-contract checks only; it does not establish ROS compilation, pluginlib loading, Nav2 runtime behavior, hardware-in-the-loop, or robot safety.

## Reuse rules

1. Treat the root TaskPlan.md as the only progress source of truth.
2. Reuse algorithm ideas only after mapping them to the repository's actual message/topic/frame contracts and pinned ROS/Nav2 versions.
3. Do not copy the prototype's provisional EKF, velocity limits, sensor rates, costmap parameters, or extrinsics into a real deployment without measurements and supplier confirmation.
4. Keep all mock topics isolated; never connect prototype command outputs to a physical Lite3.
5. Promote any prototype component into src/ only with source review, Humble/Jazzy CI, plugin/build validation where applicable, and explicit safety review.
