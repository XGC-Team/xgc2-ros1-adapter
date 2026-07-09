# XGC2 ROS1 Adapter

This repository is a Catkin workspace for the first-party
`xgc_ros1_adapter` package. It bridges ROS1 robot state into the XGC2 Go Core
adapter contract over gRPC UDS.

## Product

- Product id: `xgc2-ros1-adapter`
- APT package: `ros-noetic-xgc2-ros1-adapter`
- ROS package: `xgc_ros1_adapter`
- Runtime process id in XGC Go Core: `ros1-adapter`

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
For standalone development, the product carries the current adapter contract at
`contracts/adapter/v1/adapter.proto`. To test a different Go Core contract,
export the path explicitly:

```bash
export XGC2_CONTRACTS_DIR=/path/to/xgc2/contracts
```

The planned protocol evolution toward stable gRPC envelopes, packed typed
arrays, and a centralized versioned schema registry is documented in
[`docs/protocol-evolution-plan.md`](docs/protocol-evolution-plan.md). This is a
design plan; the current implementation continues to use the v1 contract.

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

## Runtime

The Go Core process-control manifest starts the adapter with:

```bash
rosrun xgc_ros1_adapter xgc_ros1_adapter_node \
  _adapter_id:=ros1-adapter \
  _experiment_id:=exp-ros1-sss-four-ugv \
  _socket_path:=/tmp/xgc2/adapter-ingress.sock
```

The node registers with `AdapterIngress`, pushes semantic robot state frames,
and consumes streamed robot commands from Go Core.
