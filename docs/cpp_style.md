# C++ 代码约定

本约定适用于仓库自研 C++17 代码。`src/lightning-lm` 及其他 vendor/third-party
目录保持上游风格，不由仓库格式化或静态检查自动改写。

## 基本规则

- 语言标准固定为 C++17；ROS 2 节点使用 `rclcpp`，依赖在 `CMakeLists.txt` 和
  `package.xml` 同时声明。
- 类型和函数使用 `UpperCamelCase`；变量和函数参数使用 `lower_snake_case`；成员变量
  使用尾部下划线；常量使用 `kUpperCamelCase` 或遵从所在模块既有约定。
- 头文件使用 `#pragma once`，按本项目头文件、外部依赖、标准库分组；头文件只包含
  自身声明实际需要的依赖，能前置声明时避免引入重依赖。
- 使用 `const`、引用和 `std::` RAII 类型表达所有权；不使用裸 `new/delete`。
  非拥有指针必须说明生命周期由谁保证。
- 错误沿边界显式传播：库代码抛出有意义的异常或返回明确结果，ROS/HTTP/CLI 边界
  负责转换为对应协议错误；不得吞掉错误后伪报成功。
- 对共享状态说明同步策略；不在锁内执行网络写、ROS 等待、磁盘大文件操作或子进程等待。
- HTTP 响应使用 Beast 类型和流式文件 body；上传需设置请求体上限、超时和路径/归档
  校验。新增 endpoint 必须覆盖方法、状态码、响应头和错误体的契约验证。
- CMake target 显式声明依赖和警告；禁止把全局隐式 include/link 依赖作为包契约。

## 格式与静态检查

仓库根目录 `.clang-format` 是格式事实源。新增或修改的自研 C++ 文件必须通过
`clang-format --dry-run --Werror`。Clang-Tidy 采用目标包的 `.clang-tidy` 配置；初期
启用 bugprone、clang-analyzer、performance 和少量安全的 modernize 检查，不将历史
告警升级为全仓构建错误。

工具为开发依赖，不是运行或普通交叉编译的强制依赖。Ubuntu/WSL 可安装：

```bash
sudo apt-get install clang-format clang-tidy
```

运行本次统一 HTTP 服务覆盖到的自研 C++ 格式检查：

```bash
./scripts/check_cpp_style.sh
```

第三方 vendor、生成文件、build/install/log 不纳入检查。后续扩大覆盖范围时，在脚本
中逐个添加自研包，不要对整个 `src` 做无差别格式化。
