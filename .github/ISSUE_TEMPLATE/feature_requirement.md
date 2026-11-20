---
name: Feature Requirement
about: Track a new functional capability
title: "Feature Name"
labels: ["enhancement"]
---

### Functional Requirement ("The system shall...")
*What capability, behavior, or user command does this implement?*
> Example: The system shall rotate the rover chassis to a specified absolute heading (0°–360° True North) within a tolerance of ±2.0° upon receiving a `[Face Bearing]` command.

### Non-Functional Constraints (The "How Well")
*Describe the operational limits, quality targets, and safety boundaries for this feature:*
* **Performance / Execution Limits:** (e.g., Target update rate, maximum latency, CPU/memory or bandwidth budget)
* **Safety & Edge-Case Handling:** (e.g., Failsafe state on communication or sensor failure, recovery behavior)
* **Configuration & Modularity:** (e.g., Parameters that should be tunable without recompiling)
* **Testing & Verification Standard:** (e.g., How this must be demonstrated before merging—unit tests, Gazebo sim, or hardware bench test)

### Context & Competition Link
* **Target CIRC Task:** (e.g., Automated Demolition / Mining / Meteorite)
* **Related ROS 2 Nodes / Topics:** (e.g., `/cmd_vel`, `/imu/data`)