# Kalibr AprilTag detector

Unmodified library sources and headers from the `ethz_apriltag2` package in
https://github.com/ethz-asl/kalibr at commit
`1f60227442d25e36365ef5f72cd80b9666d73467`.

The calibration preview uses Kalibr's tag36h11 detector with a two-bit black
border and a four-pixel image boundary check. Using the same detector avoids
rejecting raw camera images that Kalibr can calibrate successfully. Examples
and the ROS 1 package build files are omitted; the library is built by the ROS 2
package's CMake configuration. The upstream licence is preserved in `LICENSE`.

This product includes software developed by the Autonomous Systems Lab and
Skybotix AG.
