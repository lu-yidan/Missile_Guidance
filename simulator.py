"""
Missile Intercept Simulator
导弹拦截仿真器
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

from guidance import (
    State, GuidanceMethod, ManeuverType, GuidanceParams,
    GuidanceLaw, get_guidance_law, TargetManeuver
)


@dataclass
class SimConfig:
    """仿真配置"""
    dt: float = 0.05                    # 时间步长 (s)
    max_time: float = 15.0              # 最大仿真时间 (s)
    hit_threshold: float = 5.0          # 命中判定距离 (m)
    interceptor_speed: float = 250.0    # 拦截器速度 (m/s)
    target_speed: float = 100.0         # 目标速度 (m/s)


@dataclass
class SimResult:
    """仿真结果"""
    target_trajectory: np.ndarray
    interceptor_trajectory: np.ndarray
    times: np.ndarray
    hit: bool
    hit_time: float
    miss_distance: float
    guidance_method: str
    maneuver_type: str


class Simulator:
    """拦截仿真器"""

    def __init__(self, config: SimConfig = None):
        self.config = config or SimConfig()

    def run(self,
            target_init: State,
            interceptor_init: State,
            guidance_method: GuidanceMethod = GuidanceMethod.PN,
            maneuver_type: ManeuverType = ManeuverType.NONE,
            guidance_params: GuidanceParams = None,
            maneuver_amplitude: float = 30.0,
            maneuver_frequency: float = 0.5) -> SimResult:
        """
        运行仿真

        Args:
            target_init: 目标初始状态
            interceptor_init: 拦截器初始状态
            guidance_method: 制导方法
            maneuver_type: 目标机动类型
            guidance_params: 制导参数
            maneuver_amplitude: 机动幅度
            maneuver_frequency: 机动频率

        Returns:
            SimResult: 仿真结果
        """
        # 初始化
        target = target_init.copy()
        interceptor = interceptor_init.copy()

        guidance = get_guidance_law(guidance_method, guidance_params)
        maneuver = TargetManeuver(maneuver_type, target.vel,
                                  maneuver_amplitude, maneuver_frequency)

        # 记录轨迹
        target_traj = [target.pos.copy()]
        interceptor_traj = [interceptor.pos.copy()]
        times = [0.0]

        time = 0.0
        hit = False
        min_distance = float('inf')

        while time < self.config.max_time:
            # 计算距离
            distance = np.linalg.norm(target.pos - interceptor.pos)
            min_distance = min(min_distance, distance)

            # 命中判定
            if distance < self.config.hit_threshold:
                hit = True
                break

            # 目标机动加速度
            target_accel = maneuver.get_acceleration(target, interceptor)

            # 制导加速度
            interceptor_accel = guidance.compute(
                target, interceptor, self.config.interceptor_speed
            )

            # 更新目标状态
            target.vel += target_accel * self.config.dt
            # 限制目标速度
            target_speed = np.linalg.norm(target.vel)
            if target_speed > self.config.target_speed * 1.5:
                target.vel = target.vel / target_speed * self.config.target_speed * 1.5
            target.pos += target.vel * self.config.dt

            # 更新拦截器状态
            interceptor.vel += interceptor_accel * self.config.dt
            # 限制拦截器速度
            interceptor_speed = np.linalg.norm(interceptor.vel)
            if interceptor_speed > self.config.interceptor_speed:
                interceptor.vel = interceptor.vel / interceptor_speed * self.config.interceptor_speed
            interceptor.pos += interceptor.vel * self.config.dt

            # 记录
            target_traj.append(target.pos.copy())
            interceptor_traj.append(interceptor.pos.copy())
            time += self.config.dt
            times.append(time)

            # 检查是否已经飞过
            rel_vel = target.vel - interceptor.vel
            rel_pos = target.pos - interceptor.pos
            if np.dot(rel_pos, rel_vel) > 0 and distance > 50:
                # 拦截器已经飞过目标
                break

        return SimResult(
            target_trajectory=np.array(target_traj),
            interceptor_trajectory=np.array(interceptor_traj),
            times=np.array(times),
            hit=hit,
            hit_time=time if hit else -1,
            miss_distance=min_distance,
            guidance_method=guidance_method.value,
            maneuver_type=maneuver_type.value
        )


def run_comparison(target_init: State,
                   interceptor_init: State,
                   methods: List[GuidanceMethod],
                   maneuver_type: ManeuverType = ManeuverType.NONE,
                   config: SimConfig = None) -> List[SimResult]:
    """
    比较多种制导方法

    Args:
        target_init: 目标初始状态
        interceptor_init: 拦截器初始状态
        methods: 制导方法列表
        maneuver_type: 目标机动类型
        config: 仿真配置

    Returns:
        各方法的仿真结果列表
    """
    sim = Simulator(config)
    results = []

    for method in methods:
        result = sim.run(
            target_init.copy(),
            interceptor_init.copy(),
            guidance_method=method,
            maneuver_type=maneuver_type
        )
        results.append(result)

    return results
