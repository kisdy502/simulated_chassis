# wheel-3d Docker deployment

Two containers are used: robot-simulation runs Gazebo and the chassis; agv-bridge owns localization, online mapping and Nav2. Do not start separate navigation or Cartographer containers alongside these. This branch supports 3D lidar and saves pbstream/PGM/YAML; map-3d frontend and HTTP features are outside this deployment.

Run from a Linux or WSL shell at the repository root:

```bash
cp .env.example .env
# Optional initial map in .env:
# PBSTREAM_FILE=/ros2_ws/maps/map1008Pro/map1008Pro.pbstream
bash deploy.sh build
bash deploy.sh up
bash deploy.sh logs
```

When migrating from the old Compose profiles, first stop the old project's containers with `bash deploy.sh down`; otherwise old standalone Nav2/Cartographer processes may remain. `restart` recreates the two services to apply changed environment settings.

With PBSTREAM_FILE empty, startup has no localization or Nav2. Start mapping from the frontend or run:

```bash
bash deploy.sh start-mapping
# Drive with the frontend, then:
bash deploy.sh save-map map1008Pro
```

Mapping and saving are asynchronous. Success means accepted, not completed. Observe /agv/status, bridge logs and generated files. After localization TF becomes ready, bridge starts a fresh Nav2 stack. Set PBSTREAM_FILE in .env to the saved map for the next startup. Service commands wait up to 60 seconds for their response, including discovery.

## Data and logs

- maps/: shared writable maps, including remembered poses.
- runtime/bridge/ros/: ROS node logs.
- runtime/bridge/process/: agv_localization.log, agv_navigation.log and agv_slam.log; each subprocess launch overwrites its log. ROS logs retain per-run files.
- runtime/simulation/ros/: simulation ROS logs.

Stopping with deploy.sh down preserves bind-mounted data and the simulation volume.

## DDS and graphics

Both containers default to Cyclone DDS. The network 172.28.0.0/16 and fixed addresses .10/.11 match docker/cyclonedds.xml. If the subnet conflicts, change Compose addresses and XML peers together. See the [Cyclone DDS configuration reference](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html). Host-native ROS nodes are outside this deployment's DDS network; inspect from deploy.sh shell. Frontend access is ws://HOST:9090.

GAZEBO_GUI=false and START_RVIZ=false are defaults. Xvfb supplies a display for headless 3D lidar rendering. GUI use needs a reachable, authorized X11 display; flags alone do not configure forwarding. Keyboard and gamepad nodes are disabled in containers. Frontend manual control uses bridge's persistent velocity relay.

## Verification

```bash
docker compose config
bash deploy.sh status
bash deploy.sh shell
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic hz /clock
ros2 topic hz /points2_1
ros2 topic hz /tracked_pose
ros2 topic echo /global_costmap/published_footprint --once
```

Compare footprint stamps with /clock and exercise navigation -> mapping -> save -> localization -> navigation. DDS selection and Nav2 restarts alone do not prove the frozen-footprint issue is fixed.

## Offline delivery

```bash
bash deploy.sh export
# Destination machine:
gunzip -c wheel-3d.docker-images.tar.gz | docker load
```

Deliver docker-compose.yml, docker/, deploy.sh, .env.example and maps separately. After loading images, deploy.sh up needs no source compilation. Dockerfiles normalize entrypoint line endings for Windows checkouts.

Source files must be plain text as seen by Linux. Windows transparent-encryption software can make a file readable in Windows but unreadable to WSL/Docker; verify Linux can read modified launch/C++ files before building.

The old independent navigation/SLAM profiles and automatic motion exploration command are removed. up-nav aliases up; up-slam starts the two containers and calls bridge's mapping service. Build failures are not ignored.
