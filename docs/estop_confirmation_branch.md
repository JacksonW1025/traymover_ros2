# Depth EStop confirmation changes

This branch builds on `feature/nav-localization-20260917`; review its EStop
commit separately from the navigation and dependency changes.

The detector now requires at least two near tiles with an adjacent pair,
0.5% near pixels in the ROI, and two consecutive obstacle frames. A single
near tile, isolated tiles, or a one-frame obstacle no longer triggers a stop.
Moderately invalid depth needs two frames; validity below 20% still stops
immediately. The depth timeout changes from 0.50 to 0.80 seconds. These changes
reduce sensitivity to sparse noise and also change the detection envelope and
reaction latency. Unit tests establish the implemented rules, not their
adequacy for the moving robot.

Menu 17 starts the RealSense/automatic EStop launch for stationary observation.
It does not start a chassis or navigation process, but its normal EStop topic
can still be consumed by already-running nodes.

Vehicle stopping distance, thin-obstacle detection, intermittent depth failure,
and behavior under load remain unverified. Keep these deployment limits with
any review or merge of this branch.
