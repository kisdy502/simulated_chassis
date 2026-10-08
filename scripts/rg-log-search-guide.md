# rg 日志搜索速查手册

> 适用环境：Ubuntu 22.04 / WSL2 / ROS2 Humble。用于快速定位 ROS2、Cartographer、Spring Boot 等项目的异常日志。

## 1. 安装

```bash
sudo apt update
sudo apt install -y ripgrep
rg --version
```

`rg` 是 ripgrep 命令，支持正则表达式、递归搜索和文件过滤。

## 2. 最常用命令（从实际排障命令出发）

```bash
rg -n -C 5 \
'save_map|map1008Pro|轨迹状态|start_trajectory|定位进程启动失败|定位轨迹已启动|停止.*定位' \
/home/kisdy/.ros/log/parameter_bridge_218408_1791444783170.log
```

**含义：** 在指定日志中匹配任意一个关键词，打印匹配行号及前后各 5 行。`|` 表示“或”；`停止.*定位` 表示“停止”与“定位”之间可以有任意字符。

> 注意：`parameter_bridge` 主要负责 Gazebo 与 ROS2 消息桥接；定位启动、轨迹管理等日志更可能在 `agv_nav_server`、`cartographer_node` 或 launch 日志中。搜不到时应扩大搜索目录。

## 3. 常用参数

| 参数 | 作用 | 示例 |
| --- | --- | --- |
| `-n` | 显示行号 | `rg -n 'ERROR' app.log` |
| `-C 5` | 显示匹配行前后各 5 行 | `rg -n -C 5 'ERROR' app.log` |
| `-B 10` | 显示匹配行之前 10 行 | `rg -B 10 'ERROR' app.log` |
| `-A 10` | 显示匹配行之后 10 行 | `rg -A 10 'ERROR' app.log` |
| `-i` | 忽略大小写 | `rg -i 'error' app.log` |
| `-F` | 按普通字符串搜索，不解析正则 | `rg -F '[RMF][MAP]' app.log` |
| `-l` | 只显示包含匹配内容的文件名 | `rg -l 'start_trajectory' ~/.ros/log` |
| `-c` | 统计每个文件的匹配行数 | `rg -c 'ERROR' *.log` |
| `-v` | 显示不匹配的行 | `rg -v 'heartbeat' app.log` |
| `-g '*.log'` | 只搜索 `.log` 文件 | `rg 'ERROR' ~/.ros/log -g '*.log'` |
| `--hidden` | 搜索隐藏文件/目录 | `rg --hidden 'ERROR' ~` |
| `--line-buffered` | 及时输出管道匹配结果 | `tail -F app.log \| rg --line-buffered 'ERROR'` |

## 4. 常用组合

### 4.1 查单个日志中的异常及上下文

```bash
rg -n -C 10 'ERROR|WARN|FATAL|Exception' app.log
```

### 4.2 搜索整个 ROS2 日志目录

```bash
rg -n -C 5 \
'start_trajectory|定位失败|Topics are already used' \
~/.ros/log -g '*.log'
```

### 4.3 先找到包含异常的文件

```bash
rg -l 'Topics are already used' ~/.ros/log -g '*.log'
```

### 4.4 只查看最近 20 条匹配行

```bash
rg -n 'ERROR|WARN' app.log | tail -20
```

### 4.5 实时过滤正在增长的日志

```bash
tail -F app.log | rg --line-buffered 'ERROR|WARN|FATAL'
```

注意：`tail -F` 跟踪的是指定文件；如果 ROS2 重启后生成了**另一个文件名**，需要重新选择日志文件。

### 4.6 排除高频干扰日志

```bash
rg 'ERROR|WARN' app.log | rg -v 'heartbeat|HealthCheck|io.netty'
```

### 4.7 按日志时间文本筛选（依赖日志格式）

```bash
rg '^2026-10-08 15:30:' app.log
rg '^2026-10-08 15:.*ERROR' app.log
```

## 5. ROS2 / Cartographer 专用排障命令

### 定位进程及轨迹启动

```bash
rg -n -C 8 \
'start_trajectory|trajectory_id|轨迹状态|定位进程启动失败|定位轨迹已启动|Topics are already used' \
~/.ros/log -g '*.log'
```

### TF 超时及变换错误

```bash
rg -n -C 5 \
'TF 时间戳过期|Extrapolation|Lookup would require|TransformException' \
~/.ros/log -g '*.log'
```

### 保存地图 / pbstream

```bash
rg -n -C 8 \
'save_map|write_state|pbstream|map1008Pro' \
~/.ros/log -g '*.log'
```

### 停止和重启定位

```bash
rg -n -C 8 \
'停止.*定位|SIGKILL|SIGTERM|finish_trajectory|start_trajectory' \
~/.ros/log -g '*.log'
```

### 搜索 Gazebo 与 ROS2 桥接日志

```bash
rg -n -i -C 5 \
'error|warn|bridge|topic|failed' \
~/.ros/log -g 'parameter_bridge_*.log'
```

## 6. 先找到最新日志文件

```bash
# 按文件修改时间，列出最近 10 个桥接日志
ls -lt ~/.ros/log/parameter_bridge_*.log | head -10

# 查看最近修改的桥接日志
latest=$(ls -t ~/.ros/log/parameter_bridge_*.log | head -1)
rg -n -C 5 'ERROR|WARN|failed' "$latest"

# 实时跟踪该文件
tail -F "$latest"
```

`ls` 默认按文件名排序，**不是**按修改时间排序；使用 `ls -t` 才是按修改时间从新到旧。

## 7. 小技巧

```bash
# 搜索精确字面量（适合含 []、.、* 等特殊字符的文本）
rg -n -F '[RMF][MAP]' app.log

# 多个 -e，等价于用 | 连接多个模式
rg -n -e 'start_trajectory' -e '定位失败' app.log

# 查看匹配次数（每个文件按匹配行计数）
rg -c 'ERROR' ~/.ros/log -g '*.log'

# 可选：设置常用别名
printf "\nalias rlog='rg -n -C 5'\n" >> ~/.bashrc
source ~/.bashrc
rlog 'start_trajectory|定位失败' ~/.ros/log
```

## 8. 排障推荐流程

1. **不知道在哪个文件：** `rg -l '关键错误' ~/.ros/log -g '*.log'`。
2. **知道文件后看上下文：** `rg -n -C 10 '关键错误' 文件.log`。
3. **噪声太多：** 增加 `-g` 文件过滤，或用 `rg -v` 排除高频日志。
4. **正在复现：** `tail -F 文件.log | rg --line-buffered 'ERROR|WARN|关键错误'`。
5. **没搜到：** 检查关键词、日志文件、时间范围；也可以去掉 `-g '*.log'` 扩大搜索。

**记住一个核心模板：**

```bash
rg -n -C 5 '关键词1|关键词2|关键词3' /path/to/logs -g '*.log'
```
