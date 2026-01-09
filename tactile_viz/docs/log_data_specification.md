# Tactile Visualization Log Data Specification

本文档说明 `tactile_viz` 包产生的日志数据格式及其含义。

## 文件命名规则

日志文件存储在 `tactile_viz/log/` 目录下，命名格式如下：

| 文件类型 | 命名格式 | 说明 |
|---------|---------|------|
| 原始数据 | `raw_YYYYMMDD_HHMMSS.csv` | 每帧触觉数据的原始记录 |
| 统计数据 | `stats_YYYYMMDD_HHMMSS.csv` | 采集期间各触觉点的统计汇总 |

示例：
- `raw_20260108_201353.csv` - 2026年1月8日20:13:53开始记录的原始数据
- `stats_20260108_201353.csv` - 对应时间段的统计数据

---

## Raw 文件格式

### 概述

Raw 文件记录每一帧采样的触觉力向量数据，包含36个触觉点及其合力信息。

### 表头结构

```
sample,p0_x,p0_y,p0_z,p1_x,p1_y,p1_z,...,p35_x,p35_y,p35_z,resultant_x,resultant_y,resultant_z
```

### 字段说明

| 字段 | 类型 | 单位 | 说明 |
|-----|------|-----|------|
| `sample` | int | - | 采样序号（从0开始递增） |
| `pN_x` | float | N (牛顿) | 第N个触觉点的X方向力分量 |
| `pN_y` | float | N (牛顿) | 第N个触觉点的Y方向力分量 |
| `pN_z` | float | N (牛顿) | 第N个触觉点的Z方向力分量 |
| `resultant_x` | float | N (牛顿) | 合力的X方向分量 |
| `resultant_y` | float | N (牛顿) | 合力的Y方向分量 |
| `resultant_z` | float | N (牛顿) | 合力的Z方向分量 |

### 触觉点编号

共36个触觉点，编号从 `p0` 到 `p35`，分布在触觉传感器表面。

### 数据示例

```csv
sample,p0_x,p0_y,p0_z,p1_x,p1_y,p1_z,...,resultant_x,resultant_y,resultant_z
0,20.48,12.8,20.48,20.48,12.8,20.48,...,133.12,168.96,202.25
1,20.48,12.8,20.48,20.48,12.8,20.48,...,135.68,171.52,207.37
```

### 合力计算

合力是所有36个触觉点力向量的矢量和：

$$\vec{F}_{resultant} = \sum_{i=0}^{35} \vec{F}_i$$

其中：
- `resultant_x` = $\sum_{i=0}^{35} p_i\_x$
- `resultant_y` = $\sum_{i=0}^{35} p_i\_y$
- `resultant_z` = $\sum_{i=0}^{35} p_i\_z$

---

## Stats 文件格式

### 概述

Stats 文件对整个采集期间的数据进行统计汇总，便于分析各触觉点的力学特性。

### 表头结构

```
point,x_mean,x_std,x_min,x_max,x_range,y_mean,y_std,y_min,y_max,y_range,z_mean,z_std,z_min,z_max,z_range,mag_mean,mag_std,mag_min,mag_max,mag_range
```

### 字段说明

| 字段 | 类型 | 单位 | 说明 |
|-----|------|-----|------|
| `point` | int/string | - | 触觉点编号(0-35)或 "resultant" 表示合力 |
| `x_mean` | float | N | X分量均值 |
| `x_std` | float | N | X分量标准差 |
| `x_min` | float | N | X分量最小值 |
| `x_max` | float | N | X分量最大值 |
| `x_range` | float | N | X分量范围 (max - min) |
| `y_mean` | float | N | Y分量均值 |
| `y_std` | float | N | Y分量标准差 |
| `y_min` | float | N | Y分量最小值 |
| `y_max` | float | N | Y分量最大值 |
| `y_range` | float | N | Y分量范围 (max - min) |
| `z_mean` | float | N | Z分量均值 |
| `z_std` | float | N | Z分量标准差 |
| `z_min` | float | N | Z分量最小值 |
| `z_max` | float | N | Z分量最大值 |
| `z_range` | float | N | Z分量范围 (max - min) |
| `mag_mean` | float | N | 力幅值均值 |
| `mag_std` | float | N | 力幅值标准差 |
| `mag_min` | float | N | 力幅值最小值 |
| `mag_max` | float | N | 力幅值最大值 |
| `mag_range` | float | N | 力幅值范围 (max - min) |

### 力幅值 (Magnitude) 计算

每个点的力幅值为三维力向量的模：

$$mag = \sqrt{x^2 + y^2 + z^2}$$

### 数据示例

```csv
point,x_mean,x_std,x_min,x_max,x_range,y_mean,y_std,y_min,y_max,y_range,z_mean,z_std,z_min,z_max,z_range,mag_mean,mag_std,mag_min,mag_max,mag_range
0,22.4497,1.08234,20.48,25.6,5.12,12.5441,0.767886,10.24,12.8,2.56,17.7443,0.964362,15.36,20.48,5.12,31.27,1.04068,27.572,33.7687,6.19668
1,22.4497,1.08234,20.48,25.6,5.12,12.5441,0.767886,10.24,12.8,2.56,17.7443,0.964362,15.36,20.48,5.12,31.27,1.04068,27.572,33.7687,6.19668
...
resultant,134.857,3.30727,125.44,145.92,20.48,173.453,6.96796,148.48,194.56,46.08,209.688,4.97794,194.57,230.41,35.84,303.794,5.9156,279.833,323.551,43.7182
```

### 特殊行

最后一行 `point=resultant` 表示合力的统计数据，用于分析整体受力情况。

---

## 统计指标解释

| 指标 | 公式 | 说明 |
|-----|------|------|
| mean (均值) | $\bar{x} = \frac{1}{n}\sum_{i=1}^{n}x_i$ | 数据的中心趋势 |
| std (标准差) | $\sigma = \sqrt{\frac{1}{n}\sum_{i=1}^{n}(x_i-\bar{x})^2}$ | 数据的离散程度 |
| min (最小值) | $\min(x_1, x_2, ..., x_n)$ | 数据的下界 |
| max (最大值) | $\max(x_1, x_2, ..., x_n)$ | 数据的上界 |
| range (范围) | $max - min$ | 数据的波动范围 |

---

## 使用场景

### Raw 数据用途
- 时序分析：观察力随时间的变化趋势
- 频率分析：对力信号进行FFT分析
- 事件检测：识别接触、滑动等事件
- 数据回放：在可视化工具中重现采集场景

### Stats 数据用途
- 传感器标定：评估各触觉点的零点漂移
- 噪声分析：通过标准差评估信号稳定性
- 异常检测：识别数据范围异常的触觉点
- 性能对比：比较不同采集条件下的统计特性

---

## 坐标系定义

- **X轴**：触觉传感器平面的横向（通常为手指左右方向）
- **Y轴**：触觉传感器平面的纵向（通常为手指前后方向）
- **Z轴**：垂直于触觉传感器表面（法向力方向）

正值表示正方向受力，负值表示反方向受力。

---

## 数据分辨率

根据示例数据，力值的分辨率约为 **2.56 N**，这是由硬件ADC精度决定的。

---

## 相关配置

数据采集相关的配置参数位于 `config/point_positions.yaml`：

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `publish_rate` | 100.0 Hz | 数据发布频率 |
| `smoothing_alpha` | 0.1 | 指数平滑因子 |
| `force_threshold_show` | 5.0 N | 显示力箭头的阈值 |
| `max_slope` | 500.0 N/s | 斜率滤波器阈值 |

---

resolved: m-6593946857
