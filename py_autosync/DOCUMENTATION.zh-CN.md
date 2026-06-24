# Autosync-Time — 完整工具文档

一个独立、无界面（UI-free）的 Python 工具，用于求出相机运动与其 IMU/陀螺仪数据流之间的恒定
**时间偏移**（单位：毫秒）。它忠实移植自 Gyroflow 默认的时间偏移求解器
（`essential_matrix::find_offsets` + `filtering::Lowpass`），并额外增加了更高精度的插值模式以及
一个带真值（ground-truth）的精度评估器。

- **新用户**：先读 [`QUICKSTART.md`](QUICKSTART.md)，再回到这里深入了解。
- **本文档**是完整参考手册：背景、原理、每个模式与参数、输入/输出格式、Python API、用法配方、
  故障排查、精度数据，以及局限性。
- 英文版见 [`DOCUMENTATION.md`](DOCUMENTATION.md)（内容一致）。

---

## 目录

1. [它解决什么问题](#1-它解决什么问题)
2. [工作原理（算法）](#2-工作原理算法)
3. [安装](#3-安装)
4. [概念与约定](#4-概念与约定)
5. [命令行参考](#5-命令行参考)
   - [`sync`](#51-sync同步两路真实信号生产用途) ·
     [`selftest`](#52-selftest带真值的精度评估) ·
     [`compare`](#53-compare偏置基线) ·
     [`omega`](#54-omega导出角速度)
   - [全局参数](#55-全局参数)
6. [输入 / 输出格式](#6-输入--输出格式)
7. [精度模式](#7-精度模式nearest-vs-interp-vs-parabolic)
8. [Python API 参考](#8-python-api-参考)
9. [用法配方 / 端到端流程](#9-用法配方--端到端流程)
10. [故障排查与常见问题](#10-故障排查与常见问题)
11. [可变帧率（VFR）](#11-可变帧率vfr)
12. [与 Rust/C++ 引擎的一致性](#12-与-rustc-引擎的一致性)
13. [实测精度](#13-实测精度)
14. [局限性与未来工作](#14-局限性与未来工作)
15. [仓库文件一览](#15-仓库文件一览)

---

## 1. 它解决什么问题

视频防抖需要陀螺仪采样与视频帧共用同一个时钟。但实际中两路数据流往往以略有不同、且未知的时刻
开始——IMU 日志与相机帧之间存在某个恒定延迟（通常为几毫秒到几十毫秒）。如果给防抖器喂入错误的
偏移，校正就会滞后或超前于真实运动，残留可见抖动。

**Autosync-time 就是用来恢复这个延迟的。** 给定以角速度表示的相机旋转运动信号和 IMU 角速度信号，
它会找出能让两者最佳重合的那个时间平移量。因为两路信号测量的是**同一个物理旋转**，正确的平移量
会让运动匹配代价（cost）出现一个尖锐的极小值。

本工具是 Gyroflow 同步流程中的**时间偏移那一半**。它**不解码像素**：你需要把相机侧运动以角速度
形式提供（来自光流 / `estimated_gyro`、来自第二个陀螺仪，或——用于评估时——由 DJI 融合四元数合成）。

---

## 2. 工作原理（算法）

整条流水线与 Rust 引擎逐步对齐：

1. **角速度。** 每路信号是一个 `(N,3)` 的机体系角速度数组（deg/s），并带有逐采样时间戳（ms）。
   对四元数姿态序列，每个区间的 ω 为 `rotvec(q[i]⁻¹·q[i+1]) / Δt`（见
   `quaternions_to_angular_velocity`）；对原始陀螺仪，ω 即采样本身。
2. **零相位低通。** 两路信号都用 20 Hz 前向-反向 Butterworth 双二阶（biquad，`Q = 1/√2`）滤波。
   先正向再反向滤波可抵消相位，因此滤波不会移动代价极小值。当 `2·f_cutoff > sample_rate` 时有
   一个 Nyquist 保护会跳过滤波（例如 24/30 fps 的视频侧不被滤波——与 Gyroflow 完全一致）。
3. **代价（cost）。** 对候选偏移 `o`，每个时刻 `t` 的视频采样与 IMU 在 `t − o` 处的取值比较。
   代价为匹配采样上的加权平方差均值，各轴权重为 **x:70、y:70、z:100**（z=偏航 yaw 权重更高）。
   仅当超过半数视频采样能找到 IMU 匹配时，候选才有效。
4. **粗扫 → 细化。** 在 `[initial ± search]` 上以 **1 ms** 步长粗扫，然后在粗极值附近 **±2 ms**
   范围内以 **0.01 ms** 步长细化。
5. **子网格（可选）。** `parabolic=True` 会对细化极小值附近的三个代价采样拟合一条抛物线，跳到其
   解析顶点，从而消除残余的 0.01 ms 网格步进。
6. **IMU 查找（精度旋钮）。**
   - `nearest`（默认）：把查询时刻向上吸附到下一个 IMU 采样——与 Rust/C++ 移植逐位一致，但带有
     最多一个 IMU 采样间隔的量化偏置。
   - `interp`：在精确查询时刻对 IMU 值做线性插值——消除该偏置。
7. **接受判定。** 仅当极值落在搜索窗口的 90% 以内才接受（位于窗口边缘的弱极值会被判为“未找到”）。

---

## 3. 安装

```sh
cd py_autosync
pip install -r requirements.txt        # numpy, scipy
python3 test_autosync_time.py          # 8 个测试，应全部打印 OK
```

纯 Python，无需编译。`scipy.signal.lfilter`（零初始状态）可将 Rust 双二阶
`DirectForm2Transposed` 输出复现到约 4e-13。

---

## 4. 概念与约定

- **单位。** 代价权重和 3 deg/s 运动门限是按 **deg/s** 调校的。所有加载器内部都返回 deg/s；
  rad/s 输入会被换算。
- **时间戳。** 单位毫秒，并重新基准化（rebase）使每路信号从 0 开始。整条流水线**基于时间戳，
  从不基于帧序号**，因此非均匀间隔可被自然处理。
- **偏移符号。** `offset_ms` 是这样的延迟：**IMU 在 `video_time − offset_ms` 处被采样**。
  等价地说，IMU 数据流对齐到“视频时间戳整体平移 `−offset_ms`”。这与 Gyroflow 报告的约定一致。
- **坐标轴必须可比。** 代价是逐轴计算的。如果两路信号采用不同的轴约定（置换/翻转），需先用
  `--gyro-orientation` / `--video-orientation` 对齐（3 字符映射；大写保留该轴，小写取负：
  `xzY` → `(−x, −z, +y)`），否则可能锁不上。
- **运动门限。** 当激励低于约 3 deg/s 时极小值很浅、偏移不可靠；工具会发出警告。

---

## 5. 命令行参考

```
python3 gyroflow_autosync.py <mode> [flags]
  模式：sync | selftest | compare | omega
```

### 5.1 `sync`——同步两路真实信号（生产用途）

求两路独立测量的角速度信号之间的偏移。

```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video camera_motion.csv --interp-parabolic
```

必填：`--gyro`（IMU 日志：GCSV 或角速度 CSV）与 `--video`（相机运动 CSV）。
可选：`--units`、`--gyro-orientation`、`--video-orientation`、`--search`、`--initial`、`--lpf`、
`--interp` / `--interp-parabolic`。

采样率会根据每路信号时间戳间隔的中位数自动估计。

**输出**（stderr=诊断信息，stdout=结果）：

```
gyro : 1200 samples, ~200.0 Hz, span [0.0,5995.0] ms, max|omega| 73.16 deg/s
video: 360 samples,  ~60.0 Hz, span [0.0,5983.3] ms, max|omega| 70.08 deg/s
offset_ms=8.4940 cost=2453.5929 matched=360/360 mode=interp+parabolic
```

解读：`cost` 越低、且 `matched == 视频采样数`，说明拟合越干净。退出码：成功 `0`，未找到可接受偏移
`1`，用法错误（如缺 `--video`）`2`。

### 5.2 `selftest`——带真值的精度评估

从 DJI 四元数 CSV 合成相机侧信号，注入已知偏移并测量恢复误差——这是量化本工具自身精度的方式。

```sh
python3 gyroflow_autosync.py selftest --quat ../data/dji_quaternions_full.csv \
    --fps 30 --search 120 --inject="-30,-10,0,10,30" --noise 1.5
```

打印 `injected,recovered,error,cost,matched,frames` 表格，以及均值/RMS/最大误差汇总。

> **注意：** 传含负数的偏移列表要用 `=`，例如 `--inject="-30,..."`，否则 argparse 会把开头的 `-`
> 当作参数标志。

### 5.3 `compare`——偏置基线

求“满采样率信号”与“视频采样率重采样信号”之间的偏移，其真值偏移为 0，所以恢复出的值**就是**算法的
偏置基线（bias floor）。

```sh
python3 gyroflow_autosync.py compare --quat ../data/dji_quaternions_full.csv --fps 30 --interp
```

### 5.4 `omega`——导出角速度

从四元数推导 ω 并写成 CSV。该输出本身就是合法的 `--video`/`--gyro` 输入，所以这个模式也可当转换器用。

```sh
python3 gyroflow_autosync.py omega --quat ../data/dji_quaternions_full.csv --range="1000,9000" > omega.csv
# -> timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s
```

### 5.5 全局参数

| 参数 | 默认值 | 适用模式 | 含义 |
|------|--------|----------|------|
| `--quat PATH` | — | selftest/compare/omega | DJI 四元数 CSV（full 或 camera_data 格式） |
| `--gyro PATH` | — | sync | IMU 日志（GCSV 或角速度 CSV） |
| `--video PATH` | — | sync | 相机运动角速度 CSV |
| `--units deg\|rad` | `deg` | sync | 当列名无法推断单位时使用 |
| `--gyro-orientation STR` | 无 | sync | IMU 信号 3 字符轴重映射，如 `xzY` |
| `--video-orientation STR` | 无 | sync | 相机信号 3 字符轴重映射 |
| `--fps FLOAT` | `30` | selftest/compare | 合成视频帧率 |
| `--search FLOAT` | `200` | 所有偏移模式 | 偏移搜索半宽（ms）；必须大于真实偏移 |
| `--initial FLOAT` | `0` | 所有偏移模式 | 若已有先验，给一个粗略起始偏移（ms） |
| `--lpf FLOAT` | `20` | 所有偏移模式 | 低通截止频率（Hz）；20 与 Gyroflow 一致 |
| `--interp` | 关 | 所有偏移模式 | 插值 IMU 查找（消除量化偏置） |
| `--interp-parabolic` | 关 | 所有偏移模式 | `--interp` + 子网格顶点（隐含 `--interp`） |
| `--swap-xy` | 关 | 由四元数推导 ω | 推导 ω 时交换 x/y |
| `--inject LIST` | `-30,-15,-5,0,5,15,30` | selftest | 逗号分隔的注入偏移（ms） |
| `--noise FLOAT` | `0` | selftest | 加到视频信号上的高斯噪声标准差（deg/s） |
| `--range A,B` | 全部 | 所有模式 | 仅分析 `[A,B]` ms 内的时间戳 |
| `--seed INT` | `12345` | selftest | 噪声随机数种子 |

---

## 6. 输入 / 输出格式

### 通用角速度 CSV（`--gyro` / `--video`）

一行表头给出时间戳列名和一组三轴列名，随后是数据行：

```
timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s
0.0,1.23,-0.45,0.10
16.6667,1.40,-0.30,0.22
...
```

- **时间戳列**（按先匹配优先）：`timestamp_ms` / `time_ms` / `t_ms` / `ts_ms`（ms）；
  `timestamp_s` / `t_s` / `seconds`（s）；或裸名 `timestamp` / `time` / `t` / `ts`（按 ms 处理）。
- **三轴列**（按先匹配优先）：`wx_deg_s,wy_deg_s,wz_deg_s` · `wx_rad_s,...` · `wx,wy,wz` ·
  `gx,gy,gz` · `gyro_x,gyro_y,gyro_z` · `omega_x,omega_y,omega_z` · `x,y,z`。
- **单位**由 `_deg`/`_rad` 后缀推断，否则用 `--units`。
- 时间戳重新基准化到 0。`omega` 子命令的输出可直接回灌（round-trip）。

### GCSV IMU 日志（`--gyro`）

telemetry-parser / Gyroflow 输出的格式——先是元数据块，然后是 `t,gx,gy,gz` 数据表头：

```
GYROFLOW IMU LOG
version,1.3
tscale,0.001
gscale,1.0
orientation,xzY
t,gx,gy,gz
0,0.0123,-0.0045,0.0010
1,0.0140,-0.0030,0.0022
...
```

会处理的表头：`tscale`（原始值 → 秒，默认 0.001）、`gscale`（原始值 → rad/s，默认 1.0）、
`orientation`（轴映射；被 `--gyro-orientation` 覆盖）。靠“`t`/`time` 表头前有元数据块”这一特征
与通用 CSV 自动区分。

### DJI 四元数 CSV（`--quat`）

支持 `dji_quaternions_full.csv` 或 `dji_camera_data.csv`。自动识别列：
`quat_timestamp_ms` / `timestamp_ms`，以及 `quat_w,quat_x,quat_y,quat_z`（或 `org_quat_*`）。
四元数为 `(w,x,y,z)`，加载时归一化。

### 输出

`sync`/`compare` 在 stdout 打印一行 `offset_ms=… cost=… matched=…/… mode=…`。
`selftest` 打印 CSV 表格 + 汇总。`omega` 打印 `timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s` CSV。
诊断信息（信号统计、警告）走 stderr，使 stdout 保持适合管道。

---

## 7. 精度模式（`nearest` vs `interp` vs `parabolic`）

| 模式 | 参数 | 一致性 | 精度 | 何时使用 |
|------|------|--------|------|----------|
| nearest | *（默认）* | 与 Rust/C++ 逐位一致 | 上限 ≈ 一个 IMU 采样 | 需要逐字节一致时 |
| interp | `--interp` | 打破一致性 | 消除系统性偏置 | 粗采样 IMU（≤ ~500 Hz）、看重精度时 |
| interp+parabolic | `--interp-parabolic` | 打破一致性 | + 消除 0.01 ms 网格步进 | **默认推荐** |

**为何重要。** `nearest` 把每个查询向上吸附到下一个 IMU 采样，因此在粗采样 IMU 上会带有确定性、
同号的偏置，且**不会**通过平均抵消。`interp` 消除它；`parabolic` 再消除残余的细化网格量化。

**实测示例**（200 Hz IMU、60 fps 相机、真值偏移 8.5 ms）：

```
nearest          -> offset_ms = 10.0000   （约 +1.5 ms 量化偏置）
interp+parabolic -> offset_ms =  8.4940   （约 −0.006 ms 误差）
```

注意：`interp` 针对的是**系统性偏置**。残余的逐次估计**离散性**（在短片段或低运动片段上约 0.1 ms）
受“条件数”限制，并非量化效应——可对多个充分激励的片段做平均把它压下去（约 1/√N）。

---

## 8. Python API 参考

作为库导入：`import autosync_time as at`。

### 四元数辅助函数
- `quat_normalize(q) -> (4,)` —— 归一化 `(w,x,y,z)`。
- `quat_inverse_unit(q) -> (4,)` —— 单位四元数的共轭。
- `quat_mul(a, b) -> (4,)` —— Hamilton 乘积。
- `quat_from_axis_angle(axis, radians) -> (4,)`。

### `Lowpass`
- `Lowpass.filter_forward_backward(freq_hz, sample_rate_hz, x) -> np.ndarray | None` —— 对一维数组
  做零相位 20 Hz 风格 Butterworth；当 `2·freq > sample_rate` 时返回 `None`（Nyquist 保护）。
- `Lowpass.filter_gyro_forward_backward(freq_hz, sample_rate_hz, gyro) -> bool` —— 对 `(N,3)` 数组
  原地处理。

### 信号
- `GyroSeries(t, w)` —— 数据类；`t` 为 `(N,)` ms，`w` 为 `(N,3)` deg/s。`len()` = N。
- `quaternions_to_angular_velocity(t_ms, quats, swap_xy=False, degrees=True) -> GyroSeries`。
- `resample_angular_velocity(series, target_t_ms) -> GyroSeries` —— 逐轴线性插值，端点钳制。
- `max_angle(series) -> float` —— 最大 `|ω|` 分量（运动门限）。
- `estimate_sample_rate_hz(series) -> float` —— 由 Δt 中位数稳健估计采样率。
- `orient_vec(w, orientation) -> (N,3)` —— 应用 3 字符轴映射。

### 偏移搜索
- `find_offset(of, gyro, initial_offset_ms, search_size_ms, of_sample_rate_hz, gyro_sample_rate_hz,
  lpf_hz=20.0, interp=False, parabolic=False) -> OffsetResult`
  其中 `of` 为视频/相机信号，`gyro` 为 IMU 信号。
- `OffsetResult(found: bool, offset_ms: float, cost: float, matched: int)`。

### 加载器
- `load_quaternions(path) -> (t_ms (N,), quats (N,4))`。
- `load_gcsv(path, orientation=None) -> GyroSeries`。
- `load_angular_velocity_csv(path, units="deg", orientation=None) -> GyroSeries`。
- `load_motion(path, units="deg", orientation=None) -> GyroSeries` —— 自动区分 GCSV 与通用 CSV。

**最小库用法示例：**

```python
import autosync_time as at

gyro  = at.load_motion("imu.gcsv")
video = at.load_motion("camera_motion.csv")
r = at.find_offset(video, gyro, 0.0, 200.0,
                   at.estimate_sample_rate_hz(video),
                   at.estimate_sample_rate_hz(gyro),
                   lpf_hz=20.0, interp=True, parabolic=True)
print(r.offset_ms if r.found else "未锁定")
```

---

## 9. 用法配方 / 端到端流程

**A. 将一个真实 IMU 日志与一个光流运动 CSV 同步**
```sh
python3 gyroflow_autosync.py sync --gyro flight.gcsv --video flow_omega.csv --interp-parabolic
```

**B. 将 DJI 四元数片段转为 ω，再同步两个这样的片段**
```sh
python3 gyroflow_autosync.py omega --quat camA.csv > a.csv
python3 gyroflow_autosync.py omega --quat camB.csv > b.csv
python3 gyroflow_autosync.py sync --gyro a.csv --video b.csv --interp-parabolic
```

**C. 同步前修正不匹配的坐标轴**
```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video cam.csv --gyro-orientation xzY --interp
```

**D. 在你自己的数据上测量本工具的精度**
```sh
python3 gyroflow_autosync.py selftest --quat my_clip.csv --fps 30 \
    --inject="-20,-7.3,0,8.5,18" --noise 1.0 --interp-parabolic
```

**E. 限定到一个充分激励的时间窗口**
```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video cam.csv --interp \
    --search 100      # 一旦确定偏移较小，可用更窄的窗口
```

---

## 10. 故障排查与常见问题

| 现象 | 可能原因 / 处理 |
|------|------------------|
| `no acceptable offset found` | 真值偏移超出 `±search` → 增大 `--search`。或信号几乎不重叠 / 单位错误 → 检查 stderr 的 `span` 和 `max|omega|` 行。 |
| `--inject` 报 "expected one argument" | 开头的 `-` 被当成参数 → 用 `--inject="-30,..."`（带 `=`）。 |
| `warning: motion below the 3 deg/s gate` | 旋转不足以锁定；用 `--range` 选更动态的窗口，或接受低置信度。 |
| 在粗采样 IMU 上偏移差约 1–2 ms | 这是 `nearest` 量化偏置 → 加 `--interp-parabolic`。 |
| 运动很强却完全锁不上 | 轴约定不同 → 设置 `--gyro-orientation` / `--video-orientation`。 |
| `could not estimate a sample rate` | 某路信号少于 2 个采样 / 时间戳相同。 |
| 结果与 C++/Rust 工具相差一个 0.01 ms 步进 | `nearest` 下浮点累加打平的预期差异；`interp` 可避免。 |
| `unrecognised CSV header` / `no recognised angular-velocity columns` | 把列名改为受支持的集合（见 §6）或用 `omega` 导出格式。 |

**我该用哪个模式？** 真实数据用 `sync`；评估精度用 `selftest`；零偏移基线用 `compare`；
四元数转 ω 用 `omega`。

**能处理丢帧/可变帧率吗？** 对齐与 ω 幅值可以——见 §11。

---

## 11. 可变帧率（VFR）

流水线基于时间戳，因此非均匀帧间隔在多数环节都能处理：

1. **偏移搜索**在每个采样的*真实*时间戳上匹配（`gyro at of.t[k] − offs`），从不用 `k·dt`。
   只要喂入真实逐帧时间戳，间隔不均也没问题。
2. **ω 幅值**用每个区间的实际 `Δt`、而非恒定 fps 来除。
3. **低通**是唯一假设均匀采样率的环节（双二阶取单一标量 `sample_rate`）。实际上这很少造成问题：
   20 Hz 截止下，当 `2·20 > fps`（24/30 fps）时视频侧被 Nyquist 保护跳过，而 IMU 是稳定的约 1 kHz 流。

对被滤波那一侧的重度 VFR（60/120 fps 视频），可用以下方式缓解：用帧间隔中位数作为名义采样率；
在滤波前把两路重采样到统一时间网格、滤波后再按真实时间戳匹配；或使用时间感知滤波器
（会偏离双二阶一致性）。

---

## 12. 与 Rust/C++ 引擎的一致性

默认（`nearest`）路径逐行忠实：1 ms 粗扫 → ±2 ms 内 0.01 ms 细化、加权最小二乘代价（x,y ×70、
z ×100）、最近上界 IMU 查找（微秒键、向零截断）、90% 接受窗口，以及低通的 `2·f0 > fs` Nyquist 跳过。
在 `data/dji_quaternions_full.csv` 上，凡是代价极小值定义良好之处，恢复的偏移与代价都与
`cpp_core/build/gyroflow_autosync` 匹配到 4 位小数；打平时可能因浮点累加相差至多一个 0.01 ms 细化
步进。`interp`/`parabolic` 为换取精度而有意打破逐位一致性。

---

## 13. 实测精度

在自带 DJI 片段、30 fps 上（见 `README.md` 的 “Precision & accuracy” 以及
`variance_experiment.py` / `segment_*` 研究）：

| 模式 | 偏置（bias） | 标准差（std） | 重复误差 @7.3 ms |
|------|------|------|------|
| `nearest`（默认） | +0.166 ms | 0.210 ms | +0.446 ms |
| `interp` | **−0.005 ms** | **0.020 ms** | **+0.009 ms** |

- 随机抖动下限约 0.05 ms；`nearest` 的系统性偏置（+0.2…0.5 ms，受一个 IMU 采样约束）**不会**通过
  平均抵消——只有 `interp` 才能消除它。
- 对 N 个充分激励的片段做平均，可让随机分量按约 1/√N 缩小。综合：`interp`（消除偏置）+ 片段平均
  （消除方差）可达约 0.02 ms。

---

## 14. 局限性与未来工作

**局限性**
- 不解码视频——需自行以角速度形式提供相机侧运动。
- 估计单个恒定偏移，不估计时钟漂移（偏移 + 斜率 skew）。
- 低通假设均匀采样率（见 §11）。
- 受运动门限约束：激励低于约 3 deg/s 时不可靠。

**未来工作**（优先级路线图见 [`QUICKSTART.md`](QUICKSTART.md) 的 “Future improvements” 与
[`TODO.md`](TODO.md)）
- `--video` 直接接受四元数 CSV（内部推导 ω）。
- 归一化 0–1 置信度评分 + JSON 输出，便于流水线集成。
- 时钟漂移模型：分段偏移 + 线性拟合 → 斜率（ppm）以及偏移+斜率校正。
- GCC-PHAT / 互谱相位斜率估计器；Huber 损失 + 分段运动门限。
- 将 `interp`/`parabolic` 移植到 C++ `findOffset`，实现跨移植一致性。

---

## 15. 仓库文件一览

| 文件 | 作用 |
|------|------|
| `autosync_time.py` | 核心库：四元数辅助、`Lowpass`、ω 推导、`find_offset`、加载器。 |
| `gyroflow_autosync.py` | CLI：`sync` / `selftest` / `compare` / `omega`。 |
| `test_autosync_time.py` | 单元测试（低通、ω、恢复、噪声、interp 偏置、加载器、sync）。 |
| `variance_experiment.py` | 蒙特卡洛精度研究（偏置 / 方差 / 范围随噪声变化）。 |
| `segment_experiment.py` | 固定真值下的逐段偏移（nearest vs interp）→ PNG/CSV。 |
| `segment_stats.py` | 段数扫描 → 偏置 / 重复性 / 1/√N 平均 → PNG/CSV。 |
| `QUICKSTART.md` | 快速上手：sync、格式、精度模式、部署状态。 |
| `DOCUMENTATION.md` | 完整参考文档（英文）。 |
| `DOCUMENTATION.zh-CN.md` | 本文档——完整参考（中文）。 |
| `TODO.md` | 优先级工程路线图。 |
| `README.md` | 概览、一致性细节、VFR 说明、精度研究。 |

相关移植：`../cpp_core/`（C++ `gyroflow_autosync` + `AUTOSYNC_TIME_REPORT.md`）、
`../tools/gcsv_simple_gyro_compare.py`（GCSV 陀螺仪对比）。
