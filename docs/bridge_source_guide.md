# Bridge 业务、源码与 C++ / ROS2 导读

核对日期：2026-10-28。面向了解基本变量、if、for，但不熟悉现代 C++ 和 ROS2 的维护者。本文按当前仓库实现说明；标为“简化”的代码用于解释，不能直接替换生产实现。请按函数名定位源码，行号会随开发改变。

上位机参数和报文见 [主协议](rosbridge_integration.md)、[自由导航](free_navigation_api.md)、[电量充电](battery_api.md)。本文解释这些接口背后的调用关系、状态与语言用法。

## 1. 先分清进程与普通 C++ 对象

日常说的“bridge”经常指整套启动组合，并不是一个进程完成全部功能。

| 角色 | 职责 | 与其他角色的边界 |
|---|---|---|
| rosbridge_server | WebSocket JSON 与 ROS 通信转换 | 不生成导航路线 |
| rosapi | 查询 ROS 图、消息定义、允许访问的参数 | 不执行导航 |
| agv_nav_server | 业务入口、任务归属、状态上报、地图定位操作和进程托管 | 不直接驱动舵轮 |
| NavigationManager | 节点内的 C++ 对象，组织路径和调度 Nav2 | 不是独立 ROS 进程 |
| LocalizationMonitor | 节点内的对象，检查 TF 并提供位姿 | 不运行 SLAM 匹配算法 |
| BatteryGateway | 节点内的对象，检查电池、桥接充电服务 | 不计算模拟耗电 |
| Nav2 | 规划、路径跟踪、旋转行为、代价地图和速度平滑 | 不负责 RMS 任务调度 |
| Lightning / Cartographer | 建图或定位 | 不处理 WebSocket |
| 底盘控制器 | 执行底盘速度、发布里程计和相应 TF | 不选择业务地图 |
| battery_simulator | 独立 C++ 节点，模拟 SOC 与充电连接 | 不导航到充电点 |

主调用链：

~~~text
上位机 → rosbridge_server → /agv/follow_edge
  → AgvNavServerNode：检查业务条件、登记任务
  → NavigationManager：选择路径和底层动作
  → Nav2：NavigateToPose / FollowPath / Spin
  → 速度链路 → 底盘控制器 → 机器人运动

Nav2 反馈/结果 → NavigationManager → AgvNavServerNode → 上位机
电池节点 → /battery_state → BatteryGateway → /agv/status → 上位机
~~~

速度具体经过哪些 relay、smoother 和话题重映射，取决于实际启动配置，不能只看某处 /cmd_vel 的名字。

## 2. 源码地图与阅读顺序

以下路径相对于项目根目录，链接可直接打开。

| 顺序 | 文件 | 先搜索什么 |
|---|---|---|
| 1 | [main](../src/agv_bridge_v2/src/agv_nav_server_main.cpp) | main、MultiThreadedExecutor |
| 2 | [节点声明](../src/agv_bridge_v2/include/agv_bridge_v2/AgvNavServerNode.hpp) | 模式、成员和接口 |
| 3 | [业务入口](../src/agv_bridge_v2/src/AgvNavServerNode.cpp) | 构造函数、handle_accepted、publish_status |
| 4 | [输入校验](../src/agv_bridge_v2/include/agv_bridge_v2/NavigationValidation.hpp) | validateNavigationMode、navigationAdmissionError |
| 5 | [导航实现](../src/agv_bridge_v2/src/NavigationManager.cpp) | startNavigation、followPathNavigation、sendSpinCommand、finishTask |
| 6 | [导航声明与模板](../src/agv_bridge_v2/include/agv_bridge_v2/NavigationManager.hpp) | NavigationState、dispatchGoal |
| 7 | [定位监控](../src/agv_bridge_v2/src/LocalizationMonitor.cpp) | checkTFStability、getCurrentPose、reset |
| 8 | [电池网关](../src/agv_bridge_v2/include/agv_bridge_v2/BatteryGateway.hpp) | prepareMotion、control |
| 9 | [地图管理](../src/agv_bridge_v2/src/MapFileManager.cpp) | 目录与栅格文件加载 |
| 10 | [坐标数学](../src/agv_bridge_v2/src/TransformUtils.cpp) | 四元数、yaw 和路径计算 |
| 11 | [模拟电池节点](../src/battery_simulator/src/battery_simulator_node.cpp) | tick、publish、save |
| 12 | [电量模型](../src/battery_simulator/include/battery_simulator/model.hpp) | rate、advance |

配套文件：

- [FollowEdge.action](../src/agv_bridge_v2_interfaces/action/FollowEdge.action)：导航请求、结果、反馈字段的源头。
- [通用 bridge launch](../src/agv_bridge_v2/launch/agv_rosbridge.launch.py)：参数、白名单、rosbridge、地图保存和 3D 服务等。
- [UR5e 组合 bridge launch](../src/simulated_chassis_ur5e/launch/bridge.launch.py)：机械臂组合入口，不能与通用入口简单互换。
- [managed_ros_launch.py](../src/agv_bridge_v2/scripts/managed_ros_launch.py)：托管 launch 的生命周期辅助程序。

.hpp 通常声明“有什么类型和函数”，.cpp 定义“具体怎么做”。但模板通常需要让使用处看到定义，所以 dispatchGoal 在头文件；BatteryGateway 也把方法实现放在头文件里。不能认为头文件都没有业务逻辑。

## 3. 从 main 开始：程序如何保持运行

main 的顺序是：

1. rclcpp::init 初始化 ROS2，处理 ROS 参数与重映射。
2. Linux prctl 配置子进程收养，帮助回收托管 launch 的后代进程。
3. make_shared 创建 AgvNavServerNode。
4. 创建三个工作线程的 MultiThreadedExecutor，加入节点。
5. executor.spin 持续等待并执行消息、服务、action 和定时器回调。
6. 退出事件循环后，停止托管进程、释放节点，再 shutdown。

这里的 spin 是“运行事件循环”，不是机器人旋转；机器人旋转使用 Nav2 Spin action。

节点构造函数依次执行：

~~~cpp
initialize_parameters();
create_status_publishers();
initialize_components();
create_interfaces();
create_timers();
~~~

先创建状态发布器，让外部尽早发现话题；不意味着所有初始化已经完成。定时心跳仍需要执行器运行。组件初始化可能等待底层 action 服务。

带地图启动时，构造函数安排 transition_thread_ 异步启动定位，尝试读该地图的 last_pose.yaml。不能在执行器尚未运行时无限等待必须由 ROS 回调完成的服务响应。

## 4. Topic、Service、Action 的区别

| 类型 | 适合什么 | 本项目例子 | 关键特点 |
|---|---|---|---|
| Topic 话题 | 持续数据 | /agv/status、/odom、/battery_state | 发布/订阅，不逐条建立业务应答 |
| Service 服务 | 一次请求与响应 | /agv/set_charging、/agv/load_map | 响应中还要检查业务成功字段 |
| Action 动作 | 有过程、可取消的任务 | /agv/follow_edge、Nav2 Spin | 接收、反馈、最终结果相互分开 |

FollowEdge.action 用两个 --- 分隔 Goal、Result、Feedback，构建后生成对应的 C++ 类型。

- ACCEPTED 只表示目标已接收，不能当成开始移动或任务成功。
- action status=6 是 ABORTED，不直接说明碰撞、超时还是参数错误。
- result.success 和 message 是本项目的业务结论和原因。
- goal_handle 是某条动作的操作句柄，可用于反馈、取消和结束；不是机器人位姿。
- 函数返回 true 常只表示请求已经提交，必须查看其具体约定，不能统一理解成已到达。

## 5. 一条导航请求怎样执行

### 5.1 入口和检查顺序

在 AgvNavServerNode.cpp 搜索 handle_goal 和 handle_accepted。

当前 handle_goal 返回 ACCEPT_AND_EXECUTE；业务错误在 handle_accepted 中转换为带原因的 ABORTED。简单 REJECT 不能携带本项目完整业务说明。

handle_accepted 主要依次做：

1. 读取管理器阶段、已有任务 ID、stop 标记和业务模式。
2. 忙碌、停止或非 NAVIGATION 时，只结束新请求，保留原任务。
3. 登记新任务句柄和 command_id / node_id。
4. 校验模式、edge_id、控制点和数值。
5. 校验定位初始化、Nav2 恢复状态、生命周期及动作服务。
6. 将 ROS Goal 转成内部 MoveToMessage。
7. BatteryGateway::prepareMotion 检查电池，需要时断开充电。
8. 再检查取消状态，防止等待后端服务期间收到取消后仍移动。
9. 按模式调用 NavigationManager。

第 2、3 步不能随意交换：先覆盖句柄再发现忙碌，会破坏原任务归属。

### 5.2 不同类型调用什么

| 上层类型 | 管理器入口 | 底层主要动作 | 路线来源 |
|---|---|---|---|
| FREE | startNavigation | NavigateToPose | Nav2 自主规划 |
| STRAIGHT | followPathNavigation | FollowPath，按需 Spin | bridge 采样直线 |
| CURVE | followBezierPathNavigation | FollowPath，按需 Spin | bridge 采样贝塞尔曲线 |
| ELEVATION | 当前走 followPathNavigation 分支 | 按当前沿边实现处理 | 不能由名字推断已支持通用三维坡道规划 |
| 适用任务的 back_up=true | 倒车分支 | FollowPath | 保持当前车体朝向的后退路径 |

输入规则在 NavigationValidation.hpp：

- edge_type 必须明确，区分大小写，不再根据控制点推断。
- FREE 不校验 edge_id；其他三种类型必须有非空、非纯空白 edge_id。
- CURVE 必须有 1 或 2 个控制点，分别对应二阶、三阶曲线；不会退化为直线。
- 其他模式控制点必须为空；FREE 不接受 back_up 和 reverse。
- 目标及控制点等数值必须有效，不能使用 NaN / Infinity。

FollowPath 跟踪固定路径，不等于 NavigateToPose 自主全局规划。局部避障与重新选择全局路线是不同能力，不能只换接口名字就保证所有环境都能绕障。

### 5.3 两套状态不能混在一起

bridge 业务模式：MAPPING（建图）、RELOCALIZING（定位切换/等待）、NAVIGATION（允许继续检查导航条件）。

NavigationState 是导航任务阶段：

| 阶段 | 含义 |
|---|---|
| IDLE | 无当前执行阶段 |
| PRE_ROTATING | 沿边移动前摆正方向 |
| EXECUTING | 正在执行移动 |
| BACKING_UP | 保持朝向倒车 |
| END_ROTATING | 到点后调整最终朝向 |
| COMPLETED / FAILED / CANCELLED | 枚举定义的结果状态，具体通知与回到 IDLE 看结束路径 |

常见沿边链路：检查 → 按需预旋转 → 移动 → 按需最终旋转 → 回传结果 → IDLE。

不是每种任务都经过全部阶段。FREE 直接交给 NavigateToPose；沿边任务 end_point 影响最终旋转。NAVIGATION 模式不代表 Nav2 已完全 active，更不证明地图坐标物理正确。

### 5.4 三类回调与收尾

- goal response：底层接受或拒绝；接受后保存句柄，供取消使用。
- feedback：过程信息，例如旋转累计角度、剩余距离。
- result：底层最终 SUCCEEDED / ABORTED / CANCELED。

NavigationManager 通过 NavigationCallbacks 接口把结果交回节点，节点对上层句柄调用 succeed / abort / canceled，并清理任务归属。finishTask 是统一收尾的重要入口。

“预旋转,旋转被终止”只表明 Spin 最终 ABORTED，不能凭这句话断言碰撞或超时。具体原因还需要对应时间的 Nav2 日志，例如 Collision Ahead - Exiting Spin。

## 6. 为什么超时后不能直接清掉 busy

### 6.1 旋转 watchdog

| 参数 | 默认值 | 作用 |
|---|---:|---|
| spin_timeout_sec | 60 秒 | 单次旋转总时限，也传给 Spin time_allowance |
| spin_progress_timeout_sec | 10 秒 | 连续没有有效转动的时限 |
| spin_progress_min_angle_rad | 0.01 弧度 | 最大已转角度继续推进达到此量，才刷新进展时间 |
| spin_cancel_timeout_sec | 5 秒 | 超时取消后等待底层终态的时间 |

重复反馈和小幅抖动不能无限续期；总时限与无进展时限分别判断，不是相加。

~~~text
发现旋转超时/无进展 → 保存原因 → 发取消
  → 收到底层终态：带原失败原因结束
  → 等不到终态：标记 unsafe，保留任务占用
      → 托管模式停止本 bridge 拥有的 Nav2 进程组
      → 确认停止后 resetAfterServerShutdown，再恢复接入
~~~

取消请求发出、取消应答、动作终态是不同事件，不能把应答当作停车确认。普通取消和目标响应超时还有各自分支，不能认为与旋转 watchdog 完全相同：部分分支会释放逻辑任务，但保留 unsafe 阻止新动作。非托管模式也不能仅清 busy 假装底层已经停止。

### 6.2 generation 防止旧回调污染新任务

任务 A 失效后，它的结果可能很晚才回来，不能拿来结束任务 B。dispatchGoal 给每条底层动作编号，回调保存当时编号，并与当前编号比较：

~~~cpp
// 简化：解释编号隔离，不含完整锁和取消逻辑
const auto generation = ++action_generation_;
options.result_callback = [this, generation](const auto &result) {
    if (generation != action_generation_) return;
    // 只处理当前这代动作
};
~~~

generation 是进程内代号，不是上位机 command_id 或 ROS action UUID。旧目标延迟被接受时，代码还会尝试取消它。生命周期查询也有 request_generation，防止旧响应覆盖新状态。

## 7. 定位、地图和导航进程

LocalizationMonitor 检查 map 到底盘的 TF 是否可用、时间与连续稳定情况，提供 getCurrentPose；reset 清掉旧检查状态。它不运行定位匹配算法，TF 新鲜不等于物理位置正确。

排查时分别确认：

1. /clock 与各节点时间源一致。
2. TF、定位输出、代价地图数据新鲜。
3. 加载地图、机器人实际位置、上位机拓扑坐标一致。

### 7.1 最后位置保存

搜索 load_remembered_pose、save_remembered_pose、update_localization_monitor：

- 按地图目录保存 last_pose.yaml，含 x、y、yaw。
- 定位可用、处于 NAVIGATION、地图名非空时，约每 5 秒尝试保存。
- 启动、加载地图和定位恢复等路径尝试读对应地图的记忆位置。
- 重定位和保存地图流程也会保存相关初始位姿。
- 写入失败会记录日志；5 秒是尝试节奏，不是任何负载下的落盘保证。
- 记忆位置只是定位初值，机器人被搬动、仿真重置、地图改变后仍需要正确重定位。

先写 .tmp 再 rename，可减少读到半截文件的风险，但不能等同于断电级 fsync 保证。

### 7.2 谁启动与恢复 Nav2

显式切图/重定位新增了 [LocalizationOperationTracker](../src/agv_bridge_v2/include/agv_bridge_v2/LocalizationOperationTracker.hpp)，由工作线程按操作 ID 报告阶段，独立心跳驱动超时和定位完成检查。终态不能被迟到回调改写，最近 128 条记录可查询；失败不再回切为可导航模式。接口见 [操作结果文档](localization_operation_api.md)。它只追踪显式 load_map / relocalize，不为启动或自动恢复创建新的 ID。

ensure_managed_navigation 在 manage_navigation=true 时管理导航：检查模式、地图、定位、切换状态，再启动；进程活着还要检查生命周期。启动有宽限，持续不健康后再恢复；unsafe 动作优先处理。

恢复只处理本 bridge 拥有的进程组，不应扫描并杀掉所有 ROS 进程，也不会自动重放旧移动任务。

“托管导航 launch 已退出”是监督器看到的结果，底层原因应继续查 /tmp/agv_navigation.log；定位查 /tmp/agv_localization.log，路径以实际启动日志为准。

stop_child_process 使用信号、waitpid 和进程组检查。launch 父进程结束不代表全部 Nav2 子进程都已退出。

## 8. 用电量模块练习分层阅读

Model 是纯计算；BatterySimulator 包装 ROS 通信与文件；BatteryGateway 处理业务检查与后端适配。

~~~text
未连接 + 静止：每仿真小时减少 12 个百分点
未连接 + 移动：每仿真小时减少 30 个百分点
已连接 + 未充满：每仿真小时增加 60 个百分点
已连接 + 充满：维持 100%
~~~

BatteryState.percentage 是 0..1，/agv/status.battery 是 0..100。FULL 时 charging=false，但 charge_connected 仍可为 true；不是所有“没在充电”的状态都应该扣电。

上位机导航到充电点并停车后调用 /agv/set_charging，网关转给 /battery/set_charging。不会仅因到达某坐标自动充电。移动前 prepareMotion 检查并按条件断开；模拟端还对运动和非零速度指令做防护。

tick 用 ROS 时间差积分，仿真暂停就不继续扣电。advance 忽略非正、非有限及大于 30 秒的 dt，这是异常跳时策略，不是离线耗电补算。发布和保存使用 wall timer，不代表电量积分也用墙钟。

SOC 默认存在 ~/.ros/battery_simulator/soc.state，约每 10 秒及析构时尝试保存；恢复 SOC，不自动恢复充电连接。完整契约见 [电量文档](battery_api.md)。

## 9. C++ 声明和符号怎么读

### 9.1 命名空间、类、成员

~~~cpp
namespace agv_bridge {
class NavigationManager {
public:
    bool isReady() const;
private:
    bool initialized_ = false;
};
}
~~~

- namespace 避免同名冲突；agv_bridge::NavigationManager 指定类型所属范围。
- class 把数据和行为放在一起；public 是外部入口，private 是内部实现。
- :: 限定作用域；NavigationManager::isReady 表示类的方法。
- 成员尾部 _ 是命名习惯，没有额外语言效果。
- . 访问对象成员；-> 通过指针访问对象成员。
- 成员函数末尾 const 限制通过该函数修改普通成员，不表示函数一定没有日志、锁或外部作用。

### 9.2 const、引用与指针

~~~cpp
bool startNavigation(const MoveToMessage &task);
bool load_remembered_pose(const std::string &name,
                          geometry_msgs::msg::Pose &pose) const;
~~~

| 写法 | 含义 |
|---|---|
| const T& | 借用对象只读访问，避免复制整个任务 |
| T& | 引用，可修改调用者对象；pose 是输出参数 |
| T* | 指针，可能为空；要确认生命周期 |
| const T* | 不可通过该指针修改所指对象 |
| &value | 表达式中取地址，不同于声明中的引用 |
| *pointer | 表达式中解引用；在声明中表示指针 |
| nullptr | 空指针 |

bool 返回值的意义由函数决定；std::string &error 常用来输出失败说明。不要只看函数签名猜“导航已经成功”。

### 9.3 构造、析构与接口继承

~~~cpp
AgvNavServerNode::AgvNavServerNode(const rclcpp::NodeOptions &options)
    : Node("agv_nav_server", options) {
    initialize_parameters();
}
~~~

冒号后是初始化列表，先构造基类 Node，再执行函数体。~ClassName 是析构函数。virtual 允许通过基类接口调用实际实现，NavigationCallbacks 就是例子；override 让编译器检查是否正确覆盖。

noexcept 声明不允许异常向外传播，不代表里面永远不会发生错误；关闭函数必须在内部处理可能的异常。

### 9.4 auto、using、枚举

~~~cpp
using FollowPath = nav2_msgs::action::FollowPath;
const auto goal = goal_handle->get_goal();
enum class NavigationState { IDLE, PRE_ROTATING, EXECUTING };
~~~

using 创建类型别名，不创建对象。auto 由右侧推导编译期类型，不是动态类型。const auto 如果推导出指针，不能自动理解为所指对象也只读，需要看完整类型。

enum class 是有作用域的枚举，用 NavigationState::IDLE 访问，避免散落数字。

### 9.5 智能指针和生命周期

~~~cpp
auto node = std::make_shared<agv_bridge::AgvNavServerNode>();
~~~

shared_ptr 管理共享所有权，最后一个所有者释放后通常销毁对象；make_shared 构造对象并返回智能指针，无需手工 delete。

ROS 的 SharedPtr 常是消息智能指针别名，ConstSharedPtr 通常指向只读消息。reset 释放当前指针的所有权；其他所有者存在时对象不会马上销毁。它们不自动保证对象数据线程安全。

enable_shared_from_this 是让被共享管理的对象在合适场景获取自身共享指针的辅助类。仅继承它不会让所有 [this] 回调自动延长对象寿命，也不能随便在构造期间调用 shared_from_this。

### 9.6 optional 表示“可能没有值”

getCurrentPose 返回 std::optional<PoseStamped>，不是保证一直有位姿：

~~~cpp
// 简化：正确检查顺序
const auto pose = pose_provider_->getCurrentPose();
if (!pose.has_value()) return false;
const auto &position = pose.value().pose.position;
~~~

空 optional 上 value() 会抛异常。只打印警告后继续 value() 并不安全；遇到这类旧分支需复核，不要照搬。current_task_ 也是可选值，!current_task_ 表示没有任务。

{} 根据上下文可表示默认初始化、空容器或空返回值。校验函数 return {} 返回空字符串，表示没有错误。

## 10. 回调、lambda、bind、模板

### 10.1 注册函数不等于当场执行

~~~cpp
// 简化
timer_ = node_->create_wall_timer(1s, [this]() {
    publish_status();
});
~~~

lambda 是匿名函数：[] 写捕获内容，() 写参数，{} 写函数体。

- [this] 保存当前对象指针，允许访问成员，但不延长对象寿命。
- [this, generation] 另外复制当时的编号。
- [&] 按引用捕获局部变量；异步执行时局部变量可能早已销毁，应谨慎使用。
- auto 参数是泛型 lambda 参数，由编译器推导。

订阅回调和定时器回调由执行器以后调用，不能假设注册完就已收到数据。

### 10.2 std::bind 的读法

~~~cpp
std::bind(&NavigationManager::resultCallback, this,
          std::placeholders::_1);
~~~

意思是“以后收到第一个参数时，在 this 对象上调用 resultCallback”，近似等于：

~~~cpp
[this](const auto &result) {
    this->resultCallback(result);
};
~~~

_1、_2 是以后传入参数的位置，不是数值。成员函数前 & 表示取地址。std::function 保存符合指定签名的可调用对象，可以容纳 lambda 或 bind 的结果。

### 10.3 template 和 typename

~~~cpp
template<typename Action>
bool dispatchGoal(
    const typename rclcpp_action::Client<Action>::SharedPtr &client,
    const typename Action::Goal &goal,
    typename rclcpp_action::Client<Action>::SendGoalOptions options);
~~~

读作“为各种 Action 复用发送与保护逻辑”。dispatchGoal<Spin> 将 Action 替换为 Spin；FollowPath 同理。模板是编译期机制，不是在运行时解析字符串。

typename 在依赖模板参数的嵌套名字前说明“这是类型”。friend struct NavigationManagerTestPeer 允许指定测试辅助结构访问私有成员，不是对所有调用者开放。

### 10.4 常见标准库和表达式

| 写法 | 用法 |
|---|---|
| std::vector<T> | 可变长顺序容器，保存路径点 |
| std::deque<T> | 两端可增删的顺序容器，操作跟踪器用它保存最近 128 条记录 |
| rbegin() / rend() | 反向遍历，从最近一条记录向前查询；迭代器用 -> 访问记录成员 |
| push_back / size / empty | 添加元素、取数量、判断为空 |
| const auto &item : items | 遍历集合，只读借用每项 |
| std::make_pair(a,b) | 组合为 first / second |
| std::move(value) | 允许转移资源，不保证必然清空或必然发生移动 |
| std::isfinite(x) | 排除 NaN、正负无穷 |
| std::hypot(dx,dy) | 计算平面距离 |
| std::clamp(x,0.0,100.0) | 限制到给定区间 |
| static_cast<double>(i) | 显式转类型，可避免整数除法 |
| condition ? a : b | 条件为真取 a，否则 b |
| std::string::npos | 字符串没有找到目标位置 |
| text.c_str() | 给 C 接口或日志传字符串 |

std::move 后对象仍存在，但不要依赖原内容保持不变。vector 扩容可能让旧引用失效；跨异步边界保存引用需要特别注意。

## 11. 多线程、锁、future 与时间

### 11.1 执行器和回调组是两层

MultiThreadedExecutor 提供工作线程；CallbackGroup 决定哪些回调允许并行：同一个 MutuallyExclusive 组内互斥，不同组可以并行；Reentrant 允许组内重入，但要求业务本身安全。

心跳单独分组可以减少被慢业务堵住的机会。BatteryGateway 的后端客户端和订阅也使用独立分组。回调组对象需要保留所有权。

若回调阻塞等 future，而完成 future 的回调也需要同一个互斥组，增加线程仍可能死锁。不能简单改成单线程，也不能到处加无限等待。

### 11.2 lock_guard 与 RAII

~~~cpp
{
    std::lock_guard<std::mutex> lock(goal_mutex_);
    busy = bool(current_goal_handle_);
} // 自动解锁
~~~

RAII 将资源管理与对象生命周期绑定：lock_guard 构造加锁，析构解锁，return 或异常离开作用域也会清理。

recursive_mutex 允许同一线程重复获取同一把锁，但不解决不同锁的锁顺序死锁。节点特意避免持有 goal_mutex_ 时再获取管理器锁，因为结果回调可能按相反顺序访问。

atomic<bool> 的 load / store 用于停止、恢复等原子标记，只保证该对象的操作，不保证多个状态一起变化就自动一致。

### 11.3 时间源与时长

| 时间 | 项目用途 |
|---|---|
| node->now() / ROS clock | 消息时间、TF、电量积分，use_sim_time 时跟随 /clock |
| steady_clock | 单调超时，动作期限、进展 watchdog 等 |
| 日历时间 | 人读的日志日期、文件命名 |

using namespace std::chrono_literals 后，1s、100ms 是有单位的时长。duration<double>(...).count() 常用于换算秒，duration_cast 用于转换时长类型。

wall timer 在仿真暂停时仍可调度回调，适合心跳和超时。不同时间源的数值不能直接相减判断 TF 是否过期。

### 11.4 future 代表未来响应

~~~cpp
auto future = client->async_send_request(request);
~~~

- async_send_request 先返回，响应以后才到。
- wait_for(timeout) 有界等待，检查 ready / timeout。
- get() 取结果，没就绪时可能阻塞，也可能抛异常。
- wait_for_service 只确认服务可发现，仍需检查业务响应。
- remove_pending_request 清理客户端等待，不等于撤销远端操作。

项目 waitFuture 分段检查关闭标记，使长等待可被退出请求打断。std::thread 创建独立线程，joinable 判断可否 join，join 等待退出；销毁仍 joinable 的线程会导致程序终止，因此必须安排收尾。

## 12. ROS2 API 速查

| API | 项目用途与注意点 |
|---|---|
| declare_parameter<T> | 声明默认值，启动参数可以覆盖 |
| get_parameter | 读取实际参数，别只看默认值 |
| create_publisher<Msg> | 创建发布器，之后还要 publish |
| create_subscription<Msg> | 创建订阅并注册回调，需要保存订阅对象 |
| create_service<Srv> | 注册服务处理，类型由 .srv 生成 |
| create_client<Srv> | 创建客户端，不保证远端存在 |
| create_wall_timer | 注册周期回调，不跟随仿真推进 |
| rclcpp_action::create_server | 对上层提供 action，区分 goal/cancel/accepted |
| rclcpp_action::create_client | 调用底层动作 |
| async_send_goal | 提交目标，通过 options 注册回调 |
| async_cancel_goal | 请求取消指定目标，不等于停车确认 |
| action_server_is_ready | 动作服务可用，不代替生命周期检查 |
| lifecycle GetState | 查询 Nav2 状态，本项目以 ID 3 判断 active |
| RCLCPP_INFO/WARN/ERROR | 日志，格式与参数类型必须匹配 |
| RCLCPP_INFO_THROTTLE | 按指定时钟限频日志，不是数据更新频率 |

QoS 决定消息传输行为：队列深度 10 不是每秒 10 条；SensorDataQoS 常用于传感器流；transient_local 用于保留样本并供晚加入者获取，需匹配相应策略。发布订阅 QoS 不兼容可能收不到数据。

收到消息不能证明 header.stamp 新鲜。Pose 不带时间和坐标系，PoseStamped 带 header；map 坐标的点不能直接与 odom 坐标边界比较，必须使用 TF 转换。

四元数表示姿态，yaw 是平面偏航角，接口 theta 用弧度。TransformUtils 做转换；angles::shortest_angular_distance 处理跨 ±π 的最短角差。

## 13. 文件、异常与 Linux API

| 写法 | 作用 |
|---|---|
| ifstream / ofstream | 文件输入输出，离开作用域释放流资源 |
| setprecision(17) | 保存 double 时保留较高精度 |
| filesystem | 目录、路径、文件处理 |
| rename | 提交临时文件，必须检查返回值 |
| throw invalid_argument | 参数非法时抛出异常 |
| catch (const std::exception &e) | 捕获标准异常，e.what() 取说明 |
| catch (...) | 其他异常兜底 |
| call_once | 保证某段关闭逻辑只执行一次 |
| ::kill(-pid, signal) | 向进程组发信号，负 PID 有特定含义 |
| ::waitpid(..., WNOHANG) | 非阻塞检查或回收子进程 |
| errno / ESRCH / ECHILD | 系统调用失败详情 |

前缀 :: 指全局作用域。Linux 进程 API 面向 WSL / Linux 执行环境，不能当作通用 Windows API。

catch 后打印日志不一定完成业务收尾，还应核对任务、锁、线程、子进程和上层结果。

## 14. 源码、安装文件、运行进程不是同一份状态

~~~text
src → colcon / CMake → build 中间产物 → install 执行文件和配置
  → ros2 launch → 进程启动时读取的参数与状态
~~~

- 修改 .cpp / 参与编译的 .hpp：构建对应包，再重启使用它的进程。
- 修改 msg / srv / action：重新生成接口并构建使用方，上位机也需对齐。
- 修改 launch / YAML：是否反映到 install 取决于安装方式，不能假设软链接；已启动进程不会自动重读。
- 修改 docs：不需要构建或重启。
- 改 bridge 不等于必须重启整个仿真，按所属进程判断范围。
- CLOSED_LOOP / OPEN_LOOP 等配置应核对运行参数，而非只看某份源 YAML。

公司加密软件可能造成 Windows 能读、WSL 读到加密内容或空字节。出现 source code string cannot contain null bytes 时先核查文件，不要反复全量清理。按包编译清理见根 README，禁止全量删除 build / install / log。

## 15. 从故障倒查函数

| 现象 | 先查什么 | 源码入口 |
|---|---|---|
| 已有任务占用 | 任务 ID、阶段、旧目标终态 | handle_accepted、navigationAdmissionError |
| 上条动作未确认停止 | unsafe、取消终态、进程回收 | dispatchGoal、checkActionProgress、resetAfterServerShutdown |
| controller_server 未激活 | Nav2 日志、参数、生命周期 | readinessError、ensure_managed_navigation |
| 预旋转立刻终止 | 同时段 Spin 日志与碰撞数据 | sendSpinCommand 及结果回调 |
| 旋转长期不动 | 指令、odom、角度反馈 | observeSpinProgress、checkActionProgress |
| TF 新鲜但位置偏移 | 地图、初值、拓扑坐标 | start_managed_localization、last_pose、LocalizationMonitor |
| 电量始终 100% | FULL、连接状态、时钟、发布者 | BatteryGateway、Model::rate / advance |
| 状态不更新 | 进程、回调组、执行器、QoS | create_timers、publish_status |
| 机械臂模型不显示 | 启动包、URDF、mesh HTTP | 组合 launch、urdf_mesh_server |

这是排查入口，不是仅凭症状确定根因。优先只读采样，不用不停重启替代分析。

## 16. 建议学习路线和现有测试

1. 先从 main 到 create_interfaces，看对象如何连接，不急着读数学。
2. 跟一条 STRAIGHT：入口检查 → 路径 → Spin / FollowPath → finishTask。
3. 再看 FREE、取消、generation、watchdog，理解异步边界。
4. 从较短的电池 Model 读到节点和网关，练习区分纯计算、通信和业务策略。

修改前先回答：输入是什么、改变哪个状态、谁持有任务、谁回传结果、取消时底层怎么办。

现有测试可当作业务例子阅读：

- [导航检查与保护](../src/agv_bridge_v2/test/test_navigation_readiness.cpp)：模拟底层状态、旋转进度和停止确认。
- [任务接入](../src/agv_bridge_v2/test/test_navigation_admission.py)：忙碌原因、旧任务不被覆盖。
- [电池网关](../src/agv_bridge_v2/test/test_battery_gateway.cpp)：充电后端和移动前检查。
- [电量模型](../src/battery_simulator/test/test_battery_model.cpp)：纯计算。
- [电量节点](../src/battery_simulator/test/test_battery_node.py)：话题、服务和保存。

集成测试使用隔离 ROS domain，不要去掉隔离后在作业机器人 domain 运行。本次文档更新不表示重新执行了这些测试。

## 17. 维护约定

本文是核对日期的源码导读，不是运行时健康证明或全部分支无缺陷的声明。旧注释或日志可能落后于控制流，例如初始化日志说“跳过预旋转”，不能代替当前 readiness 条件。

新增能力时同时维护接口定义、专题报文和失败原因、本导读的职责及状态变化、关键业务测试。ROS2 后续可按官方 Humble 文档中的 Executors、Using callback groups、Writing an action server and client 主题学习；标准库按本文完整类型名逐项查，不必先学完所有模板语法才读业务。
