# HackUSF TODO

We are building a smart wheelchair that helps users navigate daily life more safely and independently through onboard sensing, mapping, voice control, and a polished hackathon-ready demo.

## Device Notes

- `/dev/ttyUSB0` -> CP210x converter -> LiDAR
- `/dev/ttyUSB1` -> CH34x converter -> Arduino / motor controller

## Adrian - ADK / Voice

- [ ] Finalize the Google ADK voice assistant flow for wheelchair commands.
- [ ] Define the supported voice command list for navigation, stop, waypoint, and help actions so it matches Michael's autonomous navigation flow.
- [ ] Connect STT output to the command parser so spoken input maps cleanly to navigation and stop actions.
- [ ] Add confirmation and error responses so users know whether a command was accepted or rejected.
- [ ] Test voice-triggered navigation actions with the mic and document any command failures, latency issues, or edge cases.

## Carlos - Hardware / Controls

- [ ] Verify Jetson, LiDAR, Arduino, motors, and power wiring are connected cleanly and safely.
- [ ] Confirm serial-port consistency so LiDAR stays on `/dev/ttyUSB0` and Arduino stays on `/dev/ttyUSB1`.
- [ ] Integrate the motor controller with Arduino command handling for forward, reverse, turn, and stop.
- [ ] Hook up MPU6050 gyro data and confirm it can be read consistently for motion feedback.
- [ ] Expose a reliable motion-control interface Michael can use for autonomous navigation testing.

## Michael - Autonomous Navigation

- [ ] Bring up SLAM end to end and confirm the LiDAR map can be generated reliably on the wheelchair.
- [ ] Set up the autonomous navigation stack so the chair can plan and follow routes safely.
- [ ] Tune obstacle detection and collision avoidance behavior for indoor demo conditions.
- [ ] Implement or validate waypoint save/load flow for repeatable navigation targets.
- [ ] Verify LiDAR, TF, and Nav2-related pieces are aligned and stable during autonomous movement.

## Jason - Integration / Demo / QA

- [ ] Create an end-to-end demo checklist covering startup, mapping, voice control, movement, and shutdown.
- [ ] Run full-system testing across hardware, autonomous navigation, and voice features and log blockers as they appear.
- [ ] Track bugs, ownership, and remaining hackathon blockers so the team has a single integration view.
- [ ] Prepare the judge-facing demo flow with a short script for what features to show and in what order.
- [ ] Clean up README and demo notes so setup, launch steps, and expected behavior are easy to follow.
