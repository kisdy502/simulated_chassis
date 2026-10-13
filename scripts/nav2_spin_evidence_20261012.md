# 2026-10-12 Nav2 spin failure evidence

Read-only inspection; no running robot nodes were stopped or restarted.

## Failure
behavior_server_1266419_1791795942889.log:
- wall 1791796812.442978151: requested 579.351, earliest 821.161, base_link -> odom.
- wall 1791796842.879800147: requested 579.351, earliest 850.511, same transform.
- Current footprint not available precedes Collision Ahead; collision is not established.

## 12-second live capture
- clock: 1091.354 -> 1102.906, 11548 samples.
- map -> odom: 1091.342 -> 1102.887, 602 samples.
- odom -> base_footprint: 1091.371 -> 1102.901, 1154 samples.
- tracked_pose: 1091.351 -> 1102.887, 601 samples.
- local published_footprint: 180 samples, every stamp 579.351.
- global published_footprint: 11 samples, every stamp 763.841.
- local footprint sole publisher is /local_costmap/local_costmap.
- base_footprint -> base_link is static, stamp 0; no active dynamic publisher for that edge observed.

## Independent fresh TF buffer
At sim1252.015:
- odom -> base_link stamp1252.011, xyz[8.16733045,-4.34449629,0.15].
- map -> base_link stamp1251.994, xyz[7.36162010,-5.22724675,0.06569351].

## Runtime
Nav2 costmap installed package 1.1.20-1jammy.20260908.002855.
RMW rmw_fastrtps_cpp; FASTRTPS_DEFAULT_PROFILES_FILE=/home/kisdy/fastdds_udp.xml (UDP-only).
Behavior footprint_topic=local_costmap/published_footprint, global_frame=odom, base=base_link, tolerance=1.5.

## Source and interpretation
https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp
updateMap copies the robot pose header into published footprint.
https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/footprint_subscriber.cpp
getFootprintInRobotFrame queries TF at footprint header stamp.

The stale timestamp originates upstream of behavior_server, in the costmap pose/TF path. Both costmaps continue publishing stale-stamped polygons while the shared TF topic and a fresh independent TF buffer are healthy. This does not prove a DDS defect or a specific internal buffer bug. The existing costmap buffers must be instrumented (received TF stamps and lookup result stamps) to distinguish reception stalls from lookup/cache behavior. Relabeling the old polygon with now() or increasing tolerance does not establish a correct current footprint.
