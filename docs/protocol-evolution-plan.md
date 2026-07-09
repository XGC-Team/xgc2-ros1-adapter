# ROS1 Adapter 协议演进计划

## 1. 背景

当前 `adapter.proto` 直接定义了 `RobotState`、`RemoteControlCommand`、
`TopicSignalCommand` 和 `ServiceCommand` 等业务字段。它已经能够覆盖现有 UGV、
Tello、PX4 多旋翼和固定翼，但继续增加机器人时容易出现两个问题：

- 公共 proto 随机器人专有字段反复修改，Core 和所有 Adapter 都需要重新生成代码。
- ROS topic、service 和 MAVROS 消息类型逐渐泄漏到跨系统协议中。

下一版协议继续使用 gRPC 和 Protobuf，不改为 JSON，也不放弃强类型传输。演进目标是
保持少量稳定的传输信封，把机器人差异放入集中管理、可版本化的 schema 定义和 Adapter
映射中。

## 2. 设计目标

- gRPC 服务、注册、心跳和命令流接口保持稳定。
- 新增机器人通常只增加 schema 定义、能力声明和 Adapter 解析器。
- 公共 proto 不为每种机器人增加固定字段或新的 `oneof` 分支。
- 同类型数值使用 Protobuf packed repeated 编码，满足高频状态和多机器人传输。
- Core 能根据 `schema_id` 做权限、长度、范围和版本检查。
- ROS topic、service、消息类型和 CAN 细节只存在于 ROS Adapter 内部。

## 3. 建议的稳定信封

以下结构用于说明方向，字段编号和命名在实施前还需要与 Go Core 一起冻结：

```protobuf
message CommandEnvelope {
  string command_id = 1;
  string robot_id = 2;
  uint32 schema_id = 3;
  uint32 schema_version = 4;

  repeated double numbers = 5;
  repeated sint64 integers = 6;
  bytes flags = 7;
  repeated string texts = 8;

  int64 issued_unix_nanos = 9;
  uint32 timeout_ms = 10;
}

message StateEnvelope {
  string robot_id = 1;
  uint32 schema_id = 2;
  uint32 schema_version = 3;

  repeated uint32 numeric_field_ids = 4;
  repeated double numeric_values = 5;
  repeated uint32 integer_field_ids = 6;
  repeated sint64 integer_values = 7;
  bytes flags = 8;

  int64 observed_unix_nanos = 9;
}
```

`repeated double`、`repeated sint64` 和 `repeated uint32` 在 proto3 中按 packed 形式编码。
信封不解释数组位置；`schema_id + schema_version` 唯一决定数据布局和语义。

## 4. 集中 Schema 注册表

schema 不定义成公共 proto 的 enum，否则新增 schema 仍会触发公共 proto 修改。建议在仓库中
维护机器可读的 YAML 注册表，并生成 C++、Go 常量、校验器和文档。

示例：

```yaml
id: 1001
name: motion.velocity
version: 1
numbers:
  - { index: 0, name: linear_x, unit: m/s, default: 0.0 }
  - { index: 1, name: linear_y, unit: m/s, default: 0.0 }
  - { index: 2, name: linear_z, unit: m/s, default: 0.0 }
  - { index: 3, name: angular_x, unit: rad/s, default: 0.0 }
  - { index: 4, name: angular_y, unit: rad/s, default: 0.0 }
  - { index: 5, name: angular_z, unit: rad/s, default: 0.0 }
flags:
  - { bit: 0, name: enabled }
  - { bit: 1, name: stop }
```

Scout 解析 `numbers[0]` 和 `numbers[5]`，全向底盘可额外解析 `numbers[1]`，飞行器可以解析
全部六个方向。机器人不支持的维度不能静默执行，应在能力协商或命令校验阶段明确拒绝。

建议按范围分配 schema ID：

| 范围 | 用途 | 示例 |
| --- | --- | --- |
| 1000-1999 | 跨机器人公共控制 | 速度、停止、位姿目标 |
| 2000-2999 | 公共状态和健康 | 位姿、电池、在线状态 |
| 10000-19999 | AgileX 专有能力 | 电机复位、底盘诊断 |
| 20000-29999 | PX4 专有能力 | 飞行模式、飞控命令 |

具体范围在首次落地时冻结；已经发布的 ID 不再复用。

## 5. 能力协商

Adapter 注册机器人时应声明每个机器人支持的 schema 和版本，而不是只上报
`robot_type`：

```protobuf
message SchemaCapability {
  uint32 schema_id = 1;
  repeated uint32 versions = 2;
}
```

Core 下发命令前检查机器人是否声明对应能力。Adapter 收到未知 schema、未知版本、数组长度
错误或字段越界时必须拒绝，并返回可定位的错误；不能按最接近的布局猜测解析。

## 6. 兼容与版本规则

- 同一 `schema_id + schema_version` 的数组位置、类型和单位永久不变。
- 追加可选值也发布新版本，避免旧接收端误判数组长度。
- 公共信封字段只能追加，不能复用或改变已发布字段号。
- 删除的公共字段号使用 `reserved` 保留。
- Core 和 Adapter 在注册阶段协商双方共同支持的最高 schema 版本。
- 安全停车必须有所有移动机器人共同支持的公共 schema，并在 Adapter 本地保留超时停车保护。

## 7. 数据通道边界

该数组协议适用于控制命令和结构化遥测，不用于承载所有 ROS 数据：

- 速度、位姿、电池、电机状态、故障码：使用 schema 数组。
- 图像、点云、地图和大块二进制数据：使用独立数据通道或对象存储引用。
- ROS 原始序列化消息：仅用于诊断或桥接，不作为 Core 的默认业务协议。

这样可以避免为高带宽传感器数据扩大控制协议，也避免 Core 依赖 ROS 消息定义。

## 8. 实施步骤

1. 盘点当前 `adapter.proto` 字段及 Go Core 使用点，形成旧字段到 schema 的映射表。
2. 冻结第一批公共 schema：`motion.velocity`、`motion.stop`、`state.pose`、
   `state.velocity`、`state.battery` 和 `state.health`。
3. 定义 YAML 格式、ID 分配规则和生成器，生成 C++/Go 常量及长度、范围校验代码。
4. 在不删除旧 RPC 的前提下增加 v2 信封和能力协商，ROS1 Adapter 同时支持 v1/v2。
5. 先用 UGV/Scout 做端到端验证，再验证 PX4 六自由度命令和固定翼状态。
6. 增加兼容性测试、错误 schema 测试、超时停车测试和高频遥测基准测试。
7. Go Core 与所有已部署 Adapter 完成升级后，再制定 v1 停用周期。

## 9. 验收条件

- 新增一种机器人时不修改稳定信封和 gRPC Service。
- Scout 与 PX4 能复用同一个 `motion.velocity` schema，并只执行各自声明支持的维度。
- 未知 schema、版本不匹配和数组长度错误均被稳定识别并拒绝。
- 旧版 Adapter 在迁移期仍可通过 v1 接口工作。
- 高频状态测试证明编码、CPU 和带宽满足目标机器人数量及发布频率。

