# XGC2 ROS1 Adapter

This repository is a Catkin workspace for the first-party
`xgc_ros1_adapter` package. It bridges ROS1 robot state into the XGC2 Go Core
adapter contract over gRPC UDS.

## Source Packages

- `xgc_ros1_adapter`: XGC2 ROS1 semantic adapter.

## Development Dependencies

Install ROS1 Noetic and the C++ dependencies required to build the adapter:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-mavros-msgs \
  libprotobuf-dev \
  protobuf-compiler \
  protobuf-compiler-grpc \
  libgrpc++-dev \
  libre2-dev \
  pkg-config
```

The adapter protobuf contract is resolved from `XGC2_CONTRACTS_DIR` when set.
For standalone development, place the contracts tree at `contracts/` in this
workspace root or export the path explicitly:

```bash
export XGC2_CONTRACTS_DIR=/path/to/xgc2/contracts
```

## Build

```bash
cd xgc2-ros1-adapter
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
source devel/setup.bash
```

After sourcing `devel/setup.bash`, ROS package resolution should find this
workspace package:

```bash
rospack find xgc_ros1_adapter
```
