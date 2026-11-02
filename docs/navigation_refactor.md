# NavigationManager 分阶段重构记录

## 第一阶段：路径生成独立（2026-10-30）

本轮只迁移几何路径生成；主节点接口、任务状态、action 客户端、回调、取消、超时和停止确认继续由原 NavigationManager 管理。

| 文件 | 职责 |
|---|---|
| [StraightPathBuilder.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/navigation/StraightPathBuilder.hpp) / [cpp](../src/agv_bridge_v2/src/navigation/StraightPathBuilder.cpp) | 直线采样、沿线朝向、精确终点 |
| [BezierPathBuilder.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/navigation/BezierPathBuilder.hpp) / [cpp](../src/agv_bridge_v2/src/navigation/BezierPathBuilder.cpp) | 二阶/三阶曲线、长度估计、控制点顺序与切线 |
| [BackUpPathBuilder.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/navigation/BackUpPathBuilder.hpp) / [cpp](../src/agv_bridge_v2/src/navigation/BackUpPathBuilder.cpp) | 后退位置采样，全程保持输入机器人朝向 |

三个类无执行状态，build 输入机器人位姿、内部任务及 logger，输出 PoseStamped 数组。不持有 Node、动作句柄、当前任务或主节点回调；无需实例化和销毁执行器。static 表示函数属于类而不依赖对象成员，const 引用避免复制输入。日志仍使用调用方原来的 logger。

调用链变为：

```text
NavigationManager
  → 相应 PathBuilder::build(pose, task, logger)
  → 获得路径
  → 原来的 Spin / FollowPath 调度与结果流程
  → 原 NavigationCallbacks → AgvNavServerNode
```

贝塞尔起点预旋转仍由 Manager 决定，只调用生成器提供的 calculateBezierTangent 取得切线；point 和 length 是生成器私有辅助方法。输入校验和速度/步长默认处理仍沿用原调用边界。

### 保持的行为

- 路径点 frame=map，时间戳为零；整个 Path 的时间由原调度设置。
- 直线和倒车至少 10 点，曲线至少 2 点；长度估计仍使用原采样方式。
- 终点位置精确覆盖为目标坐标。
- 倒车不因目标方向改变输入机器人朝向。
- 非法曲线不退化为直线。
- 移除的只有未使用的到达时间计算、未使用的时钟读取及注释掉的实验代码。

### 单独留待复核的历史行为

这轮不顺带调整以下算法细节，避免把结构重构与导航行为变化混为一谈：

1. 曲线路径采样依据 nodeId 与 sourceId 的相等关系反转控制点；预旋转切线准备处使用 reverse 标记。这两处原有依据不同，后续应单独核对业务契约。
2. 原曲线只给部分点设置切线朝向，其余点维持消息默认姿态；本轮保留并用测试标记。是否为所有采样点按曲线切线设置朝向，应作为独立行为调整验证。
3. 回归测试复现了跨线程锁次序风险：测试线程持有 Manager 锁直接调用 watchdog 发取消，同时反馈线程持有 action 客户端锁等待 Manager，形成互等。测试改为只调整时间基准，由真实 watchdog 定时器在 ROS 回调组内执行，仍验证反馈冻结与取消终态。生产取消代码本轮没有修改；后续动作会话拆分必须复核跨回调组调用的锁顺序，不能仅因测试通过便认定所有并发场景安全。

### 检查入口

- [NavigationManager.cpp](../src/agv_bridge_v2/src/NavigationManager.cpp)：检查原调用处替换，任务执行与收尾不迁移。
- [NavigationManager.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/NavigationManager.hpp)：删除路径生成和曲线数学的私有声明，公共接口保留。
- [test_path_builders.cpp](../src/agv_bridge_v2/test/test_path_builders.cpp)：直线/倒车、二阶/三阶、反向控制点、时间戳和非法曲线。
- 原导航接入与就绪测试：回归阶段、取消及超时保护。

## 第二阶段：执行流程独立（2026-10-30）

第一阶段审查通过后，增加两个执行器，保留一个任务所有者：

| 文件 | 职责 |
|---|---|
| [FreeNavigationExecutor.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/navigation/FreeNavigationExecutor.hpp) / [cpp](../src/agv_bridge_v2/src/navigation/FreeNavigationExecutor.cpp) | FREE 参数检查、目标构建、反馈转换、最终业务结果 |
| [PathNavigationExecutor.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/navigation/PathNavigationExecutor.hpp) / [cpp](../src/agv_bridge_v2/src/navigation/PathNavigationExecutor.cpp) | 直线/曲线预旋转、调用生成器、FollowPath、最终旋转及倒车流程 |
| [NavigationExecutionContext.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/navigation/NavigationExecutionContext.hpp) | 执行器调用 Manager 能力的窄接口，不暴露当前任务和动作句柄 |
| [NavigationState.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/navigation/NavigationState.hpp) | 原有枚举和字符串转换；名称和枚举值不变，避免执行器依赖 Manager 完整头文件 |

```text
AgvNavServerNode → NavigationManager：保留原来的三个导航入口
                    ├─ FreeNavigationExecutor
                    └─ PathNavigationExecutor → 三个 PathBuilder

执行器 → NavigationExecutionContext → Manager
  ├─ sendNavigateGoal / sendPathGoal / sendSpinCommand → 原 dispatchGoal / Nav2
  ├─ setState → 原 onNavigationStateChanged
  ├─ reportAccepted / reportFeedback → 原主节点回调
  └─ finishTask / failTask → 原统一任务收尾 → 主节点 action 终态
```

### 谁持有任务和动作

- Manager 独占 current_task_、当前阶段、所有 Nav2 客户端与 goal handle。
- 执行器只持有 context 引用，不保存任务副本为长期状态，不创建线程、定时器、ROS 节点或 action 客户端。
- ROS 回调仍先经过 Manager 的 generation / cancel_pending 检查，再转交执行器。取消期间和过期回调不能绕过原保护。
- Spin 结果中的未知终态与 watchdog 仍由 Manager 处理；旋转成功后的业务衔接才委托给 PathNavigationExecutor。
- 执行器提出“进入阶段”和“结束任务”，真正改变状态、清句柄及向主节点返回结果的仍是 Manager。
- Manager 在持锁状态同步调用执行器；执行器不得把 context 保存到后台线程或注册自己的异步回调。引用不延长 Manager 寿命。

三个导航入口统一先检查 ready 与已有任务。原路径入口缺少独立 busy 检查，本次补齐，防止直接调用时覆盖另一模式正在执行的任务；主节点已有的拒绝并发语义保持不变。

### C++ 接口怎么理解

NavigationExecutionContext 是抽象类：方法后面的 `= 0` 表示具体实现由使用方提供。NavigationManager 私有继承它并使用 `override` 实现，执行器只认识接口，不依赖 Manager 的私有字段，也不需要 friend 授权。

例如执行器调用 `context_.sendPathGoal(goal)`，动态调用 Manager 的实现；Manager 再通过原 dispatchGoal 安装接受、反馈、结果回调和编号保护。调用 `context_.finishTask(...)` 也只进入 Manager 原来的统一收尾，不直接对主节点的 ROS goal handle 操作。

Manager 使用 unique_ptr 持有两个执行器：唯一所有权，随 Manager 销毁；执行器用前向声明，完整类型放在 cpp，避免头文件相互包含。NavigationState 独立出来，也是为了打断头文件依赖环。

### 本阶段明确保留与修正

- 保留原预旋转 20°、最终旋转约 4.5° 阈值，以及 end_point、倒车朝向限制、速度符号和路径时间戳规则。
- 第一阶段标记的控制点顺序与部分点默认姿态问题未混入调整。
- 修正一个明确的边界错误：路径完成时定位为空，原来只打印警告后继续 optional.value()，现在返回清楚的 FAILED，避免异常中断收尾；正常有定位时流程不变。
- 路径入口没有位姿时也通过 Manager 回传失败并释放任务，避免统一 busy 检查后留下“没有发出动作但任务一直占用”的状态。
- 原 FollowPath 未知结果分支的停止确认策略留待动作会话阶段，不在执行器里擅自清除安全状态。
- 第一阶段发现的跨线程取消锁顺序风险仍需在动作会话阶段处理，本次没有复制或改写那套机制。

### 验证入口

[test_navigation_executors.cpp](../src/agv_bridge_v2/test/test_navigation_executors.cpp) 使用不连接 ROS 的假 context，验证 FREE 目标/反馈/结果、直线预旋转与最终旋转、曲线衔接、倒车不旋转、朝向错误拒绝、发送失败以及结束时丢失定位。

原导航就绪测试增加跨模式忙碌保护用例：FREE 执行时直线、曲线入口不得覆盖任务，最终结果必须仍属于原任务 ID。原导航接入、旋转 watchdog、取消确认和旧回调隔离继续回归。

## 第三阶段：统一动作会话与锁顺序（2026-10-30）

实现位于 [Nav2ActionSession.hpp](../src/agv_bridge_v2/include/agv_bridge_v2/navigation/Nav2ActionSession.hpp) / [cpp](../src/agv_bridge_v2/src/navigation/Nav2ActionSession.cpp)。前两阶段中“待动作会话阶段处理”的说明是当时的记录，本节说明当前实现。

### 当前职责

| 模块 | 持有或负责 |
|---|---|
| NavigationManager | 业务任务 ID、阶段、主节点回调、生命周期就绪检查、旋转总时限与无进展判据 |
| Nav2ActionSession | 三个 Nav2 客户端、底层句柄的取消能力、generation、响应/取消期限、unsafe 标记、传输事件队列 |
| 两个 Executor | 模式流程与业务目标，仍通过原 context 同步请求能力 |

Session 不认识 MoveToMessage、上位机任务句柄或主节点。Manager 不再保存 Nav2 goal handle，也不再实现 dispatchGoal 模板；目标接受、反馈与终态由 Session 的同一套模板保护。

### 为什么原来的锁互等不再成立

```text
Nav2 客户端回调（独立 transport callback group）
  → 短暂获取 Session 锁 → 入队 → 返回
  → 不调用 Manager、不发新的客户端请求

Manager 的 100ms 检查周期
  → Session.poll() 取出事件（返回前已释放 Session 锁）
  → 检查 generation 和当前任务
  → 执行器处理业务 → Manager 更新阶段/结果

取消 / 发送目标
  → 在 Session 锁内更新或复制状态
  → 释放 Session 锁
  → 调用 ROS 客户端 API
```

因此客户端内部锁不会通过业务回调再去等待 Manager 锁；Manager 请求取消时，Session 也不会拿着自己的锁等待客户端内部锁。业务事件统一在 Manager 侧处理，而不是在客户端持锁的回调栈里处理。

传输回调组与 Manager 定时器分开，使 Manager 暂时持锁时仍能接收底层数据。接受与终态不丢弃，连续反馈合并为最新一条，避免业务处理繁忙时积压反馈。阶段衔接和业务反馈增加至多约一个 100ms 检查周期的调度延迟，Nav2 本身的运动控制循环不经过这个队列。

### 取消、迟到响应与未知终态

- 取消应答不等于底层已停止，只有已知终态或外部确认托管进程组停止，才可清除相应 unsafe。
- 目标尚未接受就取消时，保留原来的逻辑结束策略，但禁止新动作；迟到的接受响应会排队定向取消旧 UUID，旧终态不再回传成新任务结果。
- 即使目标恰好在“检测响应超时”和“清理任务”之间被接受，也会保留它的定向取消能力，不会丢掉句柄后任其继续。
- 普通取消等待超时仍可逻辑结束，但 unsafe 继续阻止新动作；旋转 watchdog 失败或未知终态则保留当前任务，等待终态或托管组停止确认。
- FREE、FollowPath、Spin 未知终态统一按不确定停止处理，修正了此前 FollowPath 可能既不收尾也不触发安全恢复的分支。
- resetAfterServerShutdown 只能在外部确认本 bridge 拥有的 Nav2 进程组停止后调用，不是强行清 busy 的接口。不会自动重放旧任务。

### 生命周期与 C++ 用法

Manager 用 shared_ptr 持有 Session，客户端回调捕获 weak_ptr；回调先 lock，发现对象已关闭或不存在就不再处理。Manager 的周期回调与生命周期响应也改用 weak_ptr，避免迟到回调直接使用已析构的 Manager。

这里 `weak.lock()` 是尝试取得 shared_ptr、延长对象在本次调用中的生命期，不是获取互斥锁；保护状态的互斥锁是另一个概念，使用 std::lock_guard。两者分别解决“对象是否还活着”和“多个线程能否同时改数据”。

事件中的 std::function 是延迟执行的业务回调，不代表后台线程；poll 只返回事件，真正调用发生在 Manager 中。Snapshot 是在 Session 锁内复制的状态快照，查询方不直接读写其私有状态。

shutdown 关闭事件投递并对已知句柄请求取消，不能替代底层停止确认；节点拥有者仍需按现有流程停止托管进程后再销毁节点。Session 没有创建自己的线程或 ROS 节点，独立回调组由现有执行器调度。

销毁顺序应先结束执行器运行、移除节点，再销毁 Manager 和它的回调组；生产 main 已按此顺序执行，测试清理也同步调整，避免 Humble 执行器在移除节点时遇到已失效的回调组。

### 回归关注点

- 恢复第一阶段原来会互等的测试调用顺序，不再通过串行化测试来回避问题。
- 新增“Manager 持锁时传输反馈仍可入队”、迟到接受自动取消、停止确认后的旧结果隔离、超时清理窗口中恰好接受目标等用例。
- 三种动作的未知终态均验证不得提前释放安全保护。
- test_action_session_lock_order 将两个锁顺序用例连续运行 10 次，使用独立 ROS domain 95；其他集成测试继续隔离，不操作正在运行的机器人。

无需上位机协议改动；构建仅涉及 agv_bridge_v2，运行程序需重新启动才加载新二进制。
