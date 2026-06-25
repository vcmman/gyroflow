# Autosync-Time 使用文档

本文档面向使用者，讲解如何用本工具求出**相机运动与 IMU/陀螺仪之间的时间偏移**（毫秒），并把结果
可视化。算法原理、一致性与精度实验结论见 [`DESIGN.md`](DESIGN.md)；命令行速览见 [`README.md`](README.md)。

---

## 目录

1. [它能做什么](#1-它能做什么)
2. [安装](#2-安装)
3. [五分钟上手](#3-五分钟上手)
4. [四种命令模式](#4-四种命令模式)
5. [可视化](#5-可视化)
6. [输入格式](#6-输入格式)
7. [精度模式怎么选](#7-精度模式怎么选)
8. [参数速查表](#8-参数速查表)
9. [实用配方](#9-实用配方)
10. [故障排查](#10-故障排查)
11. [作为 Python 库调用](#11-作为-python-库调用)

---

## 1. 它能做什么

给定**两路测量同一段旋转运动的角速度信号**——一路来自相机（光流 / `estimated_gyro` / 另一个陀
螺仪），一路来自 IMU——本工具求出让两者最佳对齐的那个**恒定时间平移量**（即偏移 `offset_ms`）。

- **它不解码视频**：相机侧运动需以角速度形式提供。若手头只有相机姿态四元数（如 DJI），可先用
  `omega` 模式转成角速度。
- **偏移符号约定**：`offset_ms` 表示 **IMU 在 `相机时间 − offset_ms` 处采样**，与 Gyroflow 一致。

适用：离线批量求时间同步偏移；评估同步精度；把四元数姿态转角速度。
不适用：估计时钟漂移（offset + 斜率）；处理低于约 3 deg/s 激励的弱运动片段（结果不可靠，会告警）。

## 2. 安装

```sh
cd py_autosync
pip install -r requirements.txt    # numpy、scipy（可视化还需 matplotlib）
python3 test_autosync_time.py      # 8 个测试，应全部输出 OK
```

纯 Python，无需编译。核心工具只依赖 numpy + scipy；matplotlib 仅 `visualize.py` 需要。

## 3. 五分钟上手

```sh
# 1) 用两路真实信号求偏移（生产用途，推荐高精度模式）
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video camera_motion.csv --interp-parabolic

# 2) 把结果画出来（代价曲线 + 对齐叠加图）
python3 visualize.py --gyro imu.gcsv --video camera_motion.csv --interp-parabolic --out sync.png

# 3) 没有真实相机信号？用 DJI 四元数做带真值的精度自测
python3 gyroflow_autosync.py selftest --quat ../data/dji_quaternions_full.csv \
    --fps 30 --inject="-30,-10,0,10,30" --noise 1.5
```

`sync` 的典型输出（stderr 是诊断，stdout 是结果）：

```
gyro : 1200 samples, ~200.0 Hz, span [0.0,5995.0] ms, max|omega| 73.16 deg/s
video: 360 samples,  ~60.0 Hz, span [0.0,5983.3] ms, max|omega| 70.08 deg/s
offset_ms=8.4940 cost=2453.5929 matched=360/360 mode=interp+parabolic
```

读法：`cost` 越小、且 `matched` 等于视频采样数 → 拟合越干净、越可信。

> **小贴士**：含负数的列表参数（如 `--inject`）必须用 `=` 写法，例如 `--inject="-30,..."`，
> 否则 argparse 会把开头的 `-` 当成参数标志。

## 4. 四种命令模式

统一入口：`python3 gyroflow_autosync.py <mode> [参数]`

### `sync`——同步两路真实信号（生产用途）

```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video cam.csv [--interp-parabolic] [--search 200]
```

- 必填：`--gyro`（IMU 日志，GCSV 或角速度 CSV）、`--video`（相机运动角速度 CSV）。
- 采样率由各路时间戳间隔的中位数**自动估计**，无需手填。
- 退出码：成功 `0`；未找到可接受偏移 `1`；用法错误（如缺 `--video`）`2`。便于脚本判断。

### `selftest`——带真值的精度自测

从 DJI 四元数合成相机侧信号，注入已知偏移并测量恢复误差，用来量化工具在你数据上的精度。

```sh
python3 gyroflow_autosync.py selftest --quat dji.csv --fps 30 \
    --inject="-20,-7.3,0,8.5,18" --noise 1.0 --interp-parabolic
```

输出 `injected,recovered,error,cost,matched,frames` 表格 + 均值/RMS/最大误差汇总。

### `compare`——零偏移基线

求“满采样率信号”与“视频采样率重采样信号”之间的偏移（真值为 0），恢复值即算法的偏置基线。

```sh
python3 gyroflow_autosync.py compare --quat dji.csv --fps 30 --interp
```

### `omega`——导出角速度

把四元数推导成角速度并写成 CSV；输出本身就是合法的 `--gyro`/`--video` 输入，因此也可当转换器。

```sh
python3 gyroflow_autosync.py omega --quat dji.csv --range="1000,9000" > omega.csv
# 表头：timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s
```

## 5. 可视化

`visualize.py` 生成一张三联诊断图（PNG）：

- **A**：整个搜索窗口内的“代价 vs 偏移”曲线，标出找到的极小值（红线）；合成模式还会画出注入真值（绿色虚线）。
- **B**：极小值附近 ±3 ms 放大，便于看清亚毫秒级的尖锐谷底（`nearest` 模式下会看到由采样量化造成的“阶梯”）。
- **C**：在峰值运动窗口内叠加角速度幅值 |ω|——蓝色为相机、红色虚线为按找到的偏移对齐后的陀螺、
  灰色为未对齐（偏移 0）的陀螺，直观展示对齐效果。

两种输入方式（与主程序一致）：

```sh
# 真实信号
python3 visualize.py --gyro imu.gcsv --video cam.csv --interp-parabolic --out sync.png

# 合成（用四元数，并把注入偏移作为真值画出）
python3 visualize.py --quat dji.csv --fps 30 --inject 8.5 --out demo.png
```

可视化复用与搜索完全相同的代价函数（共享 `cost_curve`），所以图上的极小值与 `sync` 报告的偏移一致。

## 6. 输入格式

### 通用角速度 CSV（`--gyro` / `--video`）

一行表头给出时间戳列与一组三轴列，随后是数据行：

```
timestamp_ms,wx_deg_s,wy_deg_s,wz_deg_s
0.0,1.23,-0.45,0.10
16.6667,1.40,-0.30,0.22
```

- **时间戳列**（按先匹配优先）：`timestamp_ms`/`time_ms`/`t_ms`/`ts_ms`（毫秒）；
  `timestamp_s`/`t_s`/`seconds`（秒）；或裸名 `timestamp`/`time`/`t`/`ts`（按毫秒处理）。
- **三轴列**（按先匹配优先）：`wx_deg_s,wy_deg_s,wz_deg_s` · `wx_rad_s,...` · `wx,wy,wz` ·
  `gx,gy,gz` · `gyro_x,gyro_y,gyro_z` · `omega_x,omega_y,omega_z` · `x,y,z`。
- **单位**：由 `_deg`/`_rad` 后缀自动推断；推断不出时用 `--units deg|rad`。
- 时间戳会被重新基准化到从 0 开始。`omega` 模式的输出可直接回灌。

### GCSV（`--gyro`）

telemetry-parser / Gyroflow 输出的 IMU 日志：先是元数据块，再是 `t,gx,gy,gz` 数据表头。会处理
`tscale`（原始值→秒，默认 0.001）、`gscale`（原始值→rad/s，默认 1.0）、`orientation`（轴映射，
可被 `--gyro-orientation` 覆盖）。靠“`t`/`time` 表头前有元数据块”自动与通用 CSV 区分。

### DJI 四元数 CSV（`--quat`）

支持 `dji_quaternions_full.csv` 或 `dji_camera_data.csv`，自动识别时间戳与 `quat_w/x/y/z`（或
`org_quat_*`）列；四元数为 `(w,x,y,z)`，加载时归一化。

> **坐标轴必须可比**：代价是逐轴加权计算（x,y ×70，z ×100）。若两路信号的轴约定不同（置换/翻转），
> 先用 `--gyro-orientation` / `--video-orientation`（3 字符映射，大写保留、小写取负，如 `xzY`）对齐，
> 否则可能锁不上。

## 7. 精度模式怎么选

| 模式 | 参数 | 说明 |
|------|------|------|
| nearest | *（默认）* | 与 Rust/C++ 引擎逐位一致；精度上限 ≈ 一个 IMU 采样间隔。需要逐字节一致时用。 |
| interp | `--interp` | 线性插值 IMU 取值，消除“最近采样”量化偏置。粗采样 IMU（≤ ~500 Hz）收益明显。 |
| interp+parabolic | `--interp-parabolic` | 在 interp 基础上再做抛物线子网格顶点，消除 0.01 ms 网格步进。**追求精度时推荐。** |

举例（200 Hz IMU、60 fps、真值 8.5 ms）：`nearest` 因量化吸附给出 10.0 ms（约 +1.5 ms 偏置），
`--interp-parabolic` 给出 8.494 ms（约 −0.006 ms）。详见 `DESIGN.md`。

## 8. 参数速查表

| 参数 | 默认 | 适用 | 含义 |
|------|------|------|------|
| `--gyro PATH` | — | sync/visualize | IMU 日志（GCSV 或角速度 CSV） |
| `--video PATH` | — | sync/visualize | 相机运动角速度 CSV |
| `--quat PATH` | — | selftest/compare/omega/visualize | DJI 四元数 CSV |
| `--units deg\|rad` | `deg` | sync/visualize | 列名推断不出单位时使用 |
| `--gyro-orientation STR` | 无 | sync/visualize | IMU 信号 3 字符轴重映射，如 `xzY` |
| `--video-orientation STR` | 无 | sync/visualize | 相机信号 3 字符轴重映射 |
| `--fps FLOAT` | `30` | selftest/compare/visualize | 合成视频帧率 |
| `--search FLOAT` | `200` | 所有偏移模式 | 偏移搜索半宽（ms），必须大于真实偏移 |
| `--initial FLOAT` | `0` | 所有偏移模式 | 粗略起始偏移（ms），有先验时填 |
| `--lpf FLOAT` | `20` | 所有偏移模式 | 低通截止频率（Hz），20 与 Gyroflow 一致 |
| `--interp` | 关 | 所有偏移模式 | 插值 IMU 取值 |
| `--interp-parabolic` | 关 | 所有偏移模式 | interp + 子网格顶点（隐含 `--interp`） |
| `--inject LIST` | `-30,-15,-5,0,5,15,30` | selftest | 逗号分隔的注入偏移（ms） |
| `--noise FLOAT` | `0` | selftest | 加到视频信号的高斯噪声标准差（deg/s） |
| `--range A,B` | 全部 | 主程序各模式 | 仅分析 `[A,B]` ms 内的时间戳 |
| `--swap-xy` | 关 | 由四元数推导 ω | 推导 ω 时交换 x/y |
| `--seed INT` | `12345` | selftest | 噪声随机数种子 |
| `--out PATH` | `autosync_plot.png` | visualize | 输出 PNG 路径 |

## 9. 实用配方

**把一路真实 IMU 日志与一路光流运动 CSV 同步并出图**
```sh
python3 gyroflow_autosync.py sync --gyro flight.gcsv --video flow_omega.csv --interp-parabolic
python3 visualize.py --gyro flight.gcsv --video flow_omega.csv --interp-parabolic --out flight.png
```

**先把两个 DJI 片段转角速度，再相互同步**
```sh
python3 gyroflow_autosync.py omega --quat camA.csv > a.csv
python3 gyroflow_autosync.py omega --quat camB.csv > b.csv
python3 gyroflow_autosync.py sync --gyro a.csv --video b.csv --interp-parabolic
```

**轴约定不一致时先修正再同步**
```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video cam.csv --gyro-orientation xzY --interp
```

**确定偏移较小后用更窄窗口加速**
```sh
python3 gyroflow_autosync.py sync --gyro imu.gcsv --video cam.csv --interp --initial 8 --search 50
```

**只分析一段充分激励的时间窗口**
```sh
python3 gyroflow_autosync.py selftest --quat dji.csv --fps 30 --range="2000,12000" --interp-parabolic
```

## 10. 故障排查

| 现象 | 可能原因 / 处理 |
|------|------------------|
| `no acceptable offset found` | 真值超出 `±search` → 增大 `--search`；或信号几乎不重叠 / 单位错 → 看 stderr 的 `span` 与 `max|omega|`。 |
| `--inject` 报 “expected one argument” | 开头 `-` 被当成参数 → 用 `--inject="-30,..."`（带 `=`）。 |
| `warning: motion below the 3 deg/s gate` | 旋转太小、锁不稳 → 用 `--range` 选更动态的窗口，或接受低置信度。 |
| 粗采样 IMU 上偏移差约 1–2 ms | `nearest` 量化偏置 → 加 `--interp-parabolic`。 |
| 运动很强却完全锁不上 | 轴约定不同 → 设 `--gyro-orientation` / `--video-orientation`。 |
| `could not estimate a sample rate` | 某路信号少于 2 个采样 / 时间戳全相同。 |
| 与 C++/Rust 工具差一个 0.01 ms 步进 | `nearest` 下浮点打平的预期差异；`interp` 可避免。 |
| `unrecognised CSV header` / `no recognised angular-velocity columns` | 把列名改成受支持集合（见 §6），或改用 `omega` 输出格式。 |

## 11. 作为 Python 库调用

```python
import autosync_time as at

gyro  = at.load_motion("imu.gcsv")          # 自动识别 GCSV / 通用 CSV，返回 deg/s、ms
video = at.load_motion("camera_motion.csv")
r = at.find_offset(video, gyro, 0.0, 200.0,
                   at.estimate_sample_rate_hz(video),
                   at.estimate_sample_rate_hz(gyro),
                   lpf_hz=20.0, interp=True, parabolic=True)
print(r.offset_ms if r.found else "未锁定")

# 画自己的代价曲线
import numpy as np
offs = np.arange(-50, 50, 0.1)
costs = at.cost_curve(video, gyro, offs,
                      at.estimate_sample_rate_hz(video),
                      at.estimate_sample_rate_hz(gyro), interp=True)
```

常用 API：`load_motion` / `load_gcsv` / `load_angular_velocity_csv` / `load_quaternions`、
`quaternions_to_angular_velocity`、`resample_angular_velocity`、`estimate_sample_rate_hz`、
`max_angle`、`find_offset`（返回 `OffsetResult(found, offset_ms, cost, matched)`）、`cost_curve`。
完整签名见 `DESIGN.md` §2 与源码 `autosync_time.py`。
