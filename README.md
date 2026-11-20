# IFR 彩色方块识别（ROS 2 Humble · 话题通信 + 服务通信 + Foxglove 可视化）

用**话题通信**持续发布图像与识别结果，用**服务通信**按控制台输入的颜色返回指定方块在画面中的位置，并在 **Foxglove** 中把识别框叠加显示在图像上。

- 图像来源：包内自带的样图 `images/blocks.png`（1706×1279），因为没有接真实摄像头，所以用图片模拟相机，以 10 Hz 发布为 `sensor_msgs/msg/CompressedImage`。
- 识别方式：OpenCV 把 JPEG 解回 BGR → 高斯模糊 → HSV 阈值分割 → 形态学开闭 → 取最大轮廓的外接矩形。
- 可视化方式：`foxglove_msgs/msg/ImageAnnotations`（现代 Foxglove 主推）+ `visualization_msgs/msg/ImageMarker`（课程教材的老接口）双通道发送，内容一致。

## 1. 目录结构

```
ros2_ws/
├── src/
│   ├── ifr_interfaces/              # 接口包（必须 ament_cmake）
│   │   ├── msg/BlockResult.msg
│   │   ├── srv/DetectBlock.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── ifr_block_detection/         # 功能包
│       ├── src/image_publisher.cpp      # 节点 1：样图 → /camera/compressed
│       ├── src/block_detector.cpp       # 节点 2：识别 → 三路结果
│       ├── src/detect_block_server.cpp  # 节点 3：/detect_block 服务端
│       ├── images/blocks.png            # 样图
│       ├── CMakeLists.txt
│       └── package.xml
├── resource/                        # 实操图（文件名前缀 <你的简写>_ = 提交者姓名缩写）
│   ├── <你的简写>_01_red.png
│   ├── <你的简写>_02_blue.png
│   └── <你的简写>_03_green.png
└── README.md
```

## 2. 依赖

```bash
sudo apt install -y \
  ros-humble-rosbridge-server \
  ros-humble-foxglove-bridge \
  ros-humble-foxglove-msgs \
  ros-humble-cv-bridge \
  libopencv-dev
```

- 系统：Ubuntu 22.04 + ROS 2 Humble
- 已实测：OpenCV 4.5.4、foxglove_msgs 3.6.3、rosbridge_server 2.0.8

## 3. 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash          # 建议写进 ~/.bashrc
```

## 4. 运行（4 个终端，都要先 `source install/setup.bash`）

```bash
# 终端 1：图像话题（10 Hz）
ros2 run ifr_block_detection image_publisher

# 终端 2：识别节点（订阅图像，发识别结果 + 可视化注解）
ros2 run ifr_block_detection block_detector

# 终端 3：服务端（/detect_block）
ros2 run ifr_block_detection detect_block_server

# 终端 4：Foxglove 桥（Rosbridge 协议，端口 9090）
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

> 也可以用 `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`（Foxglove WebSocket 协议，端口 8765），两种协议 Foxglove 都支持。

## 5. 接口与话题

### 5.1 话题

| 话题 | 类型 | 方向 | 频率 | 说明 |
| --- | --- | --- | --- | --- |
| `/camera/compressed` | `sensor_msgs/msg/CompressedImage` | 发布 | 10 Hz | JPEG 图像，`frame_id=camera` |
| `/block_result` | `ifr_interfaces/msg/BlockResult` | 发布 | 30 Hz | 每种颜色一条，共 3 条/帧 |
| `/block_annotations` | `foxglove_msgs/msg/ImageAnnotations` | 发布 | 10 Hz | Foxglove 图像注解（框 + 文字） |
| `/block_marker` | `visualization_msgs/msg/ImageMarker` | 发布 | 30 Hz | 教材版图像标记（POLYGON 四角） |
| `/target_color` | `std_msgs/msg/String` | 双向 | 事件 | 只显示某个颜色的框；空串表示全显示 |

### 5.2 服务

| 服务 | 类型 | 请求 | 返回 |
| --- | --- | --- | --- |
| `/detect_block` | `ifr_interfaces/srv/DetectBlock` | `string color` | `bool success, int32 x, int32 y, int32 w, int32 h, string message` |

### 5.3 消息定义

```
# msg/BlockResult.msg
std_msgs/Header header
string color
bool   found
int32  x
int32  y
int32  w
int32  h
```

```
# srv/DetectBlock.srv
string color
---
bool   success
int32  x
int32  y
int32  w
int32  h
string message
```

## 6. 控制台调用

```bash
# 请求红色方块
ros2 service call /detect_block ifr_interfaces/srv/DetectBlock "{color: 'red'}"

# 请求蓝色方块
ros2 service call /detect_block ifr_interfaces/srv/DetectBlock "{color: 'blue'}"

# 请求绿色方块
ros2 service call /detect_block ifr_interfaces/srv/DetectBlock "{color: 'green'}"
```

返回示例（实测）：

```
response:
ifr_interfaces.srv.DetectBlock_Response(
  success=True, x=288, y=114, w=307, h=312,
  message='找到 red 方块 (结果 125 ms 前)')
```

支持的写法：`red` / `RED` / `红` / `红色` 都会归一化成 `red`。
颜色不存在时返回 `success=False, x=y=w=h=-1`，并在 `message` 里列出可用颜色：

```
success=False, x=-1, y=-1, w=-1, h=-1,
message="没有该颜色的识别结果: '黄' (可用: red / blue / green)"
```

恢复显示全部三个框：

```bash
ros2 topic pub --once /target_color std_msgs/msg/String "{data: ''}"
```

## 7. 识别参数（实测标定值）

| 颜色 | HSV 阈值（H 0–179） | 实测外接矩形 (x, y, w, h) |
| --- | --- | --- |
| red | `(0,100,80)–(10,255,255)` + `(170,100,80)–(180,255,255)` | 288, 114, 307, 312 |
| blue | `(90,100,60)–(115,255,255)` | 776, 128, 264, 312 |
| green | `(40,80,60)–(85,255,255)` | 1275, 128, 338, 317 |

- 红色用了两段阈值：HSV 的色相是环形，红色跨 0° 两侧，只用 `0–10` 会丢掉偏紫的那部分。
- 最小面积阈值 `min_area` 默认 3000 像素，滤掉噪声小斑。
- 形态学：`MORPH_OPEN` + `MORPH_CLOSE`，核为椭圆 5×5，先去掉椒盐噪点再补洞。

## 8. Foxglove 配置

1. 打开 Foxglove（网页版 <https://app.foxglove.dev> 或桌面版），**打开连接 → Rosbridge → `ws://<虚拟机IP>:9090`**。
2. 加一个 **Image** 面板，Topic 选 `/camera/compressed`。
3. 在这个 Image 面板的设置里找到 **图像注释 / Annotations**，勾选 `/block_annotations`
   （`/block_marker` 是同内容的旧格式，二选一即可，两个都勾会在同一位置画出重叠的框）。
4. 左侧打开 **Service Call** 面板，Service 选 `/detect_block`，参数填 `{"color": "red"}`，也可以直接点 Call。

## 9. 实操图

统一放在仓库根目录的 `resource/` 下，文件名前缀 `<你的简写>_` 为提交者姓名缩写（与提交身份 `<你的简写>_butter@163.com` 一致）。

| 文件 | 内容 |
| --- | --- |
| `resource/<你的简写>_01_red.png` | 控制台请求 red，Foxglove 上只剩红框 |
| `resource/<你的简写>_02_blue.png` | 控制台请求 blue，Foxglove 上只剩蓝框 |
| `resource/<你的简写>_03_green.png` | 控制台请求 green，Foxglove 上只剩绿框 |

每张图里都能同时看到「Foxglove 画出来的识别框」和「控制台的 service call 命令与返回」。

## 10. 设计说明

**为什么服务端不自己算，而是订阅 `/block_result`？**
识别是连续动作（10 Hz），而服务是离散查询。让 `block_detector` 持续把结果广播到 `/block_result`，服务端只维护一份「最近一次识别结果」的缓存，收到请求时按颜色查表返回。这样职责单一，而且用 `header.stamp` 判新鲜度（默认 2000 ms 过期），识别节点停了也不会返回过期坐标。

**为什么同时发 `ImageAnnotations` 和 `ImageMarker`？**
`visualization_msgs/msg/ImageMarker` 是 ROS 1 时代的老接口，教材要求；`foxglove_msgs/msg/ImageAnnotations` 是 2023 年后 Foxglove 官方主推的替代品。两条都发，无论用哪个版本的 Foxglove 都一定有框显示，代价是每帧多几百字节。
注意 Humble 的 `ImageMarker` **没有** `LINE_LOOP` 常量，所以用 `POLYGON` + 四个角点画闭合框。

**为什么要有 `/target_color`？**
`block_detector` 每帧本来会画三个框，但作业要求每张实操图上是**一个**方块的识别框。所以加了这个话题：服务端收到合法的颜色请求后发布该颜色，识别节点只在 `target_color` 为空或匹配时才发注解，`/block_result` 仍然全发（服务端缓存不能缺色）。

## 11. 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| `/camera/compressed` 频率是 20/40 Hz 而不是 10 Hz | 有残留的 `image_publisher` 进程。`ros2 node list` 看是否出现两个 `/ifr_image_publisher`，`pgrep -af image_publisher` 找到旧进程 kill 掉 |
| `ros2 service list` 看不到 `/detect_block` | ros2 daemon 缓存。`ros2 daemon stop`，再新开终端 `ros2 service list` |
| `Package 'ifr_block_detection' not found` | 当前终端没 source：`source ~/ros2_ws/install/setup.bash`（每次 `colcon build` 后旧终端都要重新 source） |
| 直接敲 `block_detector` 报 command not found | 可执行文件必须用 `ros2 run ifr_block_detection block_detector` |
| Foxglove 连不上 9090 | `ss -ltn \| grep 9090` 确认在监听；确认 `ws://` 地址是**虚拟机**的 IP；网页版页面是 HTTPS，连明文 `ws://` 非 localhost 地址可能被浏览器拦，此时用桌面版或把 Foxglove 装在虚拟机里 |
| 画面上有三个框 | `/target_color` 是空的，属于正常。请求一个颜色就只剩一个框 |
| 画面冻住不刷新 | Foxglove 底部播放控制被暂停了，点一下播放 |

## 12. 提交

```bash
cd ~/ros2_ws
git init
git add .
git commit -m "feat: ROS 2 彩色方块识别（话题 + 服务 + Foxglove 可视化）"
git remote add origin <你的仓库地址>
git push -u origin main
```
