# 比例导引法 (Proportional Navigation, PN)

## 概述

比例导引法是现代导弹制导系统中**最广泛使用**的制导律，自1940年代发明以来，几乎所有的空空导弹、地空导弹和反舰导弹都采用这种方法或其变体。

**为什么PN如此流行？**
- 原理简单，易于实现
- 对目标匀速运动可实现零脱靶
- 对传感器噪声不敏感
- 计算量小，适合早期模拟电路实现

---

## 基本概念

### 视线 (Line of Sight, LOS)

视线是从拦截器指向目标的连线。

```
        目标 T
         /
        /  ← 视线 (LOS)
       /
      /
   拦截器 M
```

### 视线角 (LOS Angle, λ)

视线与参考方向（通常是惯性坐标系的x轴）的夹角：

```
λ = arctan2(y_T - y_M, x_T - x_M)
```

### 视线角速度 (LOS Rate, λ̇)

视线角随时间的变化率，这是PN制导的**核心量**。

---

## 核心原理

### PN制导律

**拦截器的加速度与视线角速度成正比：**

```
a = N × Vc × λ̇
```

其中：
- `a` = 拦截器加速度（垂直于速度方向）
- `N` = 导引系数（Navigation Constant），通常取 3~5
- `Vc` = 接近速度（Closing Velocity）
- `λ̇` = 视线角速度

### 物理意义

> **如果视线角保持不变（λ̇ = 0），两个物体必然会相撞。**

这就是PN的核心思想：
1. 如果 λ̇ > 0（视线顺时针旋转），拦截器向右转
2. 如果 λ̇ < 0（视线逆时针旋转），拦截器向左转
3. 如果 λ̇ = 0，保持当前方向

通过不断消除视线角速度，拦截器最终会命中目标。

---

## 数学推导

### 几何关系

设：
- 拦截器位置：`(x_M, y_M)`，速度：`(vx_M, vy_M)`
- 目标位置：`(x_T, y_T)`，速度：`(vx_T, vy_T)`

相对位置向量：
```
r = [x_T - x_M, y_T - y_M]
```

相对速度向量：
```
v_rel = [vx_T - vx_M, vy_T - vy_M]
```

### 视线角速度推导

视线角：
```
λ = arctan2(r_y, r_x)
```

对时间求导：
```
λ̇ = d/dt[arctan2(r_y, r_x)]
  = (r_x × ṙ_y - r_y × ṙ_x) / (r_x² + r_y²)
  = (r_x × v_rel_y - r_y × v_rel_x) / |r|²
```

这就是**视线角速度**的计算公式。

### 接近速度

接近速度是相对速度在视线方向的分量（取负号使接近为正）：

```
Vc = -v_rel · r̂ = -(v_rel · r) / |r|
```

其中 `r̂` 是视线单位向量。

### PN加速度指令

加速度方向垂直于视线（法向）：

```
n̂ = [-r_y/|r|, r_x/|r|]  (视线法向量)

a = N × Vc × λ̇ × n̂
```

---

## 代码实现

```python
def proportional_navigation(target: State, interceptor: State,
                            nav_gain: float = 4.0) -> np.ndarray:
    """
    比例导引法

    Args:
        target: 目标状态 (pos, vel)
        interceptor: 拦截器状态 (pos, vel)
        nav_gain: 导引系数 N

    Returns:
        加速度向量
    """
    # 相对位置和速度
    rel_pos = target.pos - interceptor.pos
    rel_vel = target.vel - interceptor.vel

    # 视线距离
    range_dist = np.linalg.norm(rel_pos)
    if range_dist < 1e-6:
        return np.array([0.0, 0.0])

    # 视线单位向量
    los_unit = rel_pos / range_dist

    # 视线法向量（垂直于视线）
    los_normal = np.array([-los_unit[1], los_unit[0]])

    # 视线角速度: λ̇ = (r × v) / |r|²
    # 二维叉积: r × v = r_x * v_y - r_y * v_x
    los_rate = (rel_pos[0] * rel_vel[1] - rel_pos[1] * rel_vel[0]) / (range_dist ** 2)

    # 接近速度: Vc = -v_rel · r̂
    closing_speed = -np.dot(rel_vel, los_unit)

    # PN加速度: a = N * Vc * λ̇
    accel_magnitude = nav_gain * closing_speed * los_rate
    acceleration = accel_magnitude * los_normal

    return acceleration
```

---

## 导引系数 N 的选择

| N 值 | 特点 |
|------|------|
| N = 2 | 理论最小值，轨迹为直线碰撞 |
| N = 3 | 经典值，对匀速目标最优 |
| N = 4 | 常用值，对机动目标有一定裕度 |
| N = 5 | 激进值，响应快但能量消耗大 |

**理论分析：**
- N < 2：无法命中目标
- N = 3：对非机动目标，能量消耗最小
- N > 3：对机动目标有更好的鲁棒性

---

## 几何解释

### 碰撞三角形

```
                    T (目标)
                   /|
                  / |
                 /  | V_T (目标速度)
                /   |
               /    ↓
              /     T'(预测位置)
             /     .
            /   .
           / .  ← 碰撞点
          /.
         /
        M ----→ V_M (拦截器速度)
     (拦截器)
```

PN制导使拦截器速度方向始终指向碰撞点，而不是目标当前位置。

### 与追踪法对比

| 方法 | 策略 | 轨迹 |
|------|------|------|
| 追踪法 (Pursuit) | 始终指向目标当前位置 | 弯曲的追逐曲线 |
| 比例导引 (PN) | 消除视线旋转 | 近似直线 |

追踪法的问题：**狗追兔子曲线**（尾追），需要更长路径。

---

## PN的变体

### 1. 真比例导引 (True PN, TPN)

加速度方向垂直于**视线**而非速度：
```
a = N × V_M × λ̇ × n̂_LOS
```

### 2. 增广比例导引 (Augmented PN, APN)

考虑目标加速度：
```
a = N × Vc × λ̇ + (N/2) × a_T
```

适合对抗机动目标。

### 3. 最优制导律 (Optimal Guidance)

基于最优控制理论：
```
a = N × ZEM / t_go²
```

其中 ZEM (Zero Effort Miss) 是零控脱靶量，t_go 是剩余飞行时间。

---

## 优缺点分析

### 优点

1. **简单可靠**：只需测量视线角速度
2. **最优性**：对匀速目标，N=3时能量最优
3. **鲁棒性**：对测量噪声不敏感
4. **无需目标速度**：不需要知道目标的精确速度

### 缺点

1. **末段加速度发散**：接近目标时 λ̇ 可能剧烈变化
2. **对机动目标有滞后**：目标突然机动时响应不足
3. **需要接近速度**：Vc ≤ 0 时失效（目标远离）

---

## 实际应用

### 传感器要求

PN只需要测量：
1. **视线角** λ（方位角）
2. **视线角速度** λ̇

不需要：
- 目标距离（雷达测距）
- 目标速度

这使得**红外制导导弹**特别适合使用PN，因为红外导引头只能测量角度。

### 典型导弹

| 导弹 | 类型 | 制导方式 |
|------|------|----------|
| AIM-9 响尾蛇 | 空空导弹 | 红外 + PN |
| AIM-120 | 空空导弹 | 主动雷达 + PN变体 |
| 标准-2 | 舰空导弹 | 半主动雷达 + PN |
| 爱国者 | 地空导弹 | TVM + 增广PN |

---

## 仿真演示

运行以下命令查看PN制导效果：

```bash
# 基础PN vs 匀速目标
python main.py single -m proportional_navigation -n none --animate

# PN vs 蛇形机动目标
python main.py single -m proportional_navigation -n weave --animate

# 对比所有制导方法
python main.py compare --maneuver weave
```

---

## 总结

比例导引法的核心思想可以用一句话概括：

> **保持视线不转动，你就能命中目标。**

这个简单而优雅的原理，使PN成为了过去80年来最成功的制导算法。尽管现代导弹使用了更复杂的变体（APN、最优制导等），但PN的基本思想仍然是所有这些方法的基础。

---

## 参考资料

1. Zarchan, P. "Tactical and Strategic Missile Guidance" - 制导领域的经典教材
2. Shneydor, N.A. "Missile Guidance and Pursuit" - 详细的数学推导
3. Siouris, G.M. "Missile Guidance and Control Systems" - 系统工程视角
