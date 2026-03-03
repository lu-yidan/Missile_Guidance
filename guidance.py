"""
Missile Guidance Algorithms Module
多种导弹制导算法实现
"""

import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional
from enum import Enum


class GuidanceMethod(Enum):
    """制导方法枚举"""
    PURSUIT = "pursuit"                    # 追踪法
    PREDICTIVE = "predictive"              # 预测法
    PN = "proportional_navigation"         # 比例导引
    APN = "augmented_pn"                   # 增广比例导引
    TPN = "true_pn"                        # 真比例导引
    OPTIMAL = "optimal"                    # 最优制导


class ManeuverType(Enum):
    """目标机动类型"""
    NONE = "none"                          # 无机动（匀速直线）
    WEAVE = "weave"                        # 蛇形机动
    BARREL_ROLL = "barrel_roll"            # 桶滚机动
    BREAK_TURN = "break_turn"              # 急转弯
    SPIRAL = "spiral"                      # 螺旋机动
    RANDOM = "random"                      # 随机机动
    EVASIVE = "evasive"                    # 规避机动（感知拦截器）


@dataclass
class State:
    """运动状态"""
    pos: np.ndarray      # 位置 [x, y]
    vel: np.ndarray      # 速度 [vx, vy]

    def copy(self) -> 'State':
        return State(self.pos.copy(), self.vel.copy())


@dataclass
class GuidanceParams:
    """制导参数"""
    nav_gain: float = 4.0           # 导引系数 N
    max_accel: float = 50.0         # 最大加速度 m/s^2
    time_to_go_min: float = 0.1     # 最小剩余时间


class GuidanceLaw:
    """制导律基类"""

    def __init__(self, params: GuidanceParams = None):
        self.params = params or GuidanceParams()

    def compute(self, target: State, interceptor: State,
                interceptor_speed: float) -> np.ndarray:
        """计算制导加速度"""
        raise NotImplementedError


class PursuitGuidance(GuidanceLaw):
    """
    追踪法 (Pure Pursuit)
    始终指向目标当前位置
    """

    def compute(self, target: State, interceptor: State,
                interceptor_speed: float) -> np.ndarray:
        rel_pos = target.pos - interceptor.pos
        distance = np.linalg.norm(rel_pos)

        if distance < 1e-6:
            return np.array([0.0, 0.0])

        # 期望速度方向指向目标
        desired_vel = rel_pos / distance * interceptor_speed
        # 加速度 = 速度变化
        accel = (desired_vel - interceptor.vel) * 2.0

        # 限制加速度
        accel_mag = np.linalg.norm(accel)
        if accel_mag > self.params.max_accel:
            accel = accel / accel_mag * self.params.max_accel

        return accel


class PredictiveGuidance(GuidanceLaw):
    """
    预测法 (Predictive/Lead Pursuit)
    指向目标的预测位置
    """

    def compute(self, target: State, interceptor: State,
                interceptor_speed: float) -> np.ndarray:
        # 解碰撞三角形求拦截时间
        rel_pos = target.pos - interceptor.pos
        rel_vel = target.vel

        a = np.dot(rel_vel, rel_vel) - interceptor_speed**2
        b = 2 * np.dot(rel_pos, rel_vel)
        c = np.dot(rel_pos, rel_pos)

        disc = b**2 - 4*a*c
        if disc < 0:
            # 无解，退化为追踪法
            t_go = np.linalg.norm(rel_pos) / interceptor_speed
        else:
            sqrt_disc = np.sqrt(disc)
            t1 = (-b + sqrt_disc) / (2*a) if abs(a) > 1e-6 else -c/b
            t2 = (-b - sqrt_disc) / (2*a) if abs(a) > 1e-6 else -c/b
            valid = [t for t in [t1, t2] if t > 0.01]
            t_go = min(valid) if valid else np.linalg.norm(rel_pos) / interceptor_speed

        # 预测拦截点
        predicted_pos = target.pos + target.vel * t_go
        direction = predicted_pos - interceptor.pos
        dist = np.linalg.norm(direction)

        if dist < 1e-6:
            return np.array([0.0, 0.0])

        desired_vel = direction / dist * interceptor_speed
        accel = (desired_vel - interceptor.vel) * 2.0

        accel_mag = np.linalg.norm(accel)
        if accel_mag > self.params.max_accel:
            accel = accel / accel_mag * self.params.max_accel

        return accel


class ProportionalNavigation(GuidanceLaw):
    """
    比例导引法 (Proportional Navigation, PN)
    a = N * Vc * dλ/dt
    """

    def compute(self, target: State, interceptor: State,
                interceptor_speed: float) -> np.ndarray:
        rel_pos = target.pos - interceptor.pos
        rel_vel = target.vel - interceptor.vel
        range_dist = np.linalg.norm(rel_pos)

        if range_dist < 1e-6:
            return np.array([0.0, 0.0])

        # 视线单位向量和法向量
        los_unit = rel_pos / range_dist
        los_normal = np.array([-los_unit[1], los_unit[0]])

        # 视线角速度 = (r × v) / |r|^2
        los_rate = (rel_pos[0]*rel_vel[1] - rel_pos[1]*rel_vel[0]) / (range_dist**2)

        # 接近速度
        closing_speed = -np.dot(rel_vel, los_unit)

        # PN加速度
        accel_mag = self.params.nav_gain * closing_speed * los_rate
        accel = accel_mag * los_normal

        # 限幅
        if np.linalg.norm(accel) > self.params.max_accel:
            accel = accel / np.linalg.norm(accel) * self.params.max_accel

        return accel


class AugmentedPN(GuidanceLaw):
    """
    增广比例导引 (Augmented PN, APN)
    考虑目标加速度: a = N * Vc * dλ/dt + N/2 * a_t
    """

    def __init__(self, params: GuidanceParams = None):
        super().__init__(params)
        self.prev_target_vel = None
        self.dt = 0.05

    def compute(self, target: State, interceptor: State,
                interceptor_speed: float) -> np.ndarray:
        rel_pos = target.pos - interceptor.pos
        rel_vel = target.vel - interceptor.vel
        range_dist = np.linalg.norm(rel_pos)

        if range_dist < 1e-6:
            return np.array([0.0, 0.0])

        # 估计目标加速度
        if self.prev_target_vel is not None:
            target_accel = (target.vel - self.prev_target_vel) / self.dt
        else:
            target_accel = np.array([0.0, 0.0])
        self.prev_target_vel = target.vel.copy()

        # 视线相关计算
        los_unit = rel_pos / range_dist
        los_normal = np.array([-los_unit[1], los_unit[0]])
        los_rate = (rel_pos[0]*rel_vel[1] - rel_pos[1]*rel_vel[0]) / (range_dist**2)
        closing_speed = -np.dot(rel_vel, los_unit)

        # APN: 基础PN + 目标加速度补偿
        accel_pn = self.params.nav_gain * closing_speed * los_rate * los_normal
        accel_comp = (self.params.nav_gain / 2) * target_accel
        accel = accel_pn + accel_comp

        if np.linalg.norm(accel) > self.params.max_accel:
            accel = accel / np.linalg.norm(accel) * self.params.max_accel

        return accel


class TruePN(GuidanceLaw):
    """
    真比例导引 (True PN, TPN)
    加速度垂直于视线方向
    """

    def compute(self, target: State, interceptor: State,
                interceptor_speed: float) -> np.ndarray:
        rel_pos = target.pos - interceptor.pos
        rel_vel = target.vel - interceptor.vel
        range_dist = np.linalg.norm(rel_pos)

        if range_dist < 1e-6:
            return np.array([0.0, 0.0])

        los_unit = rel_pos / range_dist
        los_normal = np.array([-los_unit[1], los_unit[0]])

        # 视线角速度
        omega = (rel_pos[0]*rel_vel[1] - rel_pos[1]*rel_vel[0]) / (range_dist**2)

        # TPN: 加速度垂直于视线
        interceptor_speed_actual = np.linalg.norm(interceptor.vel)
        accel_mag = self.params.nav_gain * interceptor_speed_actual * omega
        accel = accel_mag * los_normal

        if np.linalg.norm(accel) > self.params.max_accel:
            accel = accel / np.linalg.norm(accel) * self.params.max_accel

        return accel


class OptimalGuidance(GuidanceLaw):
    """
    最优制导律 (Optimal Guidance Law)
    基于剩余飞行时间的ZEM（零脱靶量）制导
    """

    def compute(self, target: State, interceptor: State,
                interceptor_speed: float) -> np.ndarray:
        rel_pos = target.pos - interceptor.pos
        rel_vel = target.vel - interceptor.vel
        range_dist = np.linalg.norm(rel_pos)

        if range_dist < 1e-6:
            return np.array([0.0, 0.0])

        los_unit = rel_pos / range_dist
        closing_speed = -np.dot(rel_vel, los_unit)

        # 估计剩余飞行时间
        t_go = max(range_dist / max(closing_speed, 1.0), self.params.time_to_go_min)

        # 零脱靶量 (Zero Effort Miss)
        zem = rel_pos + rel_vel * t_go

        # 最优制导加速度: a = N * ZEM / t_go^2
        N = self.params.nav_gain
        accel = N * zem / (t_go ** 2)

        if np.linalg.norm(accel) > self.params.max_accel:
            accel = accel / np.linalg.norm(accel) * self.params.max_accel

        return accel


def get_guidance_law(method: GuidanceMethod, params: GuidanceParams = None) -> GuidanceLaw:
    """获取制导律实例"""
    guidance_classes = {
        GuidanceMethod.PURSUIT: PursuitGuidance,
        GuidanceMethod.PREDICTIVE: PredictiveGuidance,
        GuidanceMethod.PN: ProportionalNavigation,
        GuidanceMethod.APN: AugmentedPN,
        GuidanceMethod.TPN: TruePN,
        GuidanceMethod.OPTIMAL: OptimalGuidance,
    }
    return guidance_classes[method](params)


class TargetManeuver:
    """目标机动生成器"""

    def __init__(self, maneuver_type: ManeuverType,
                 base_vel: np.ndarray,
                 amplitude: float = 30.0,
                 frequency: float = 0.5):
        self.maneuver_type = maneuver_type
        self.base_vel = base_vel.copy()
        self.amplitude = amplitude  # 机动幅度 m/s^2
        self.frequency = frequency  # 机动频率 Hz
        self.time = 0.0
        self.rng = np.random.default_rng(42)
        self.random_phase = self.rng.random() * 2 * np.pi

    def get_acceleration(self, target: State, interceptor: Optional[State] = None) -> np.ndarray:
        """获取目标加速度"""
        self.time += 0.05
        t = self.time

        if self.maneuver_type == ManeuverType.NONE:
            return np.array([0.0, 0.0])

        elif self.maneuver_type == ManeuverType.WEAVE:
            # 蛇形机动：正弦加速度
            vel_dir = self.base_vel / np.linalg.norm(self.base_vel)
            perp_dir = np.array([-vel_dir[1], vel_dir[0]])
            return self.amplitude * np.sin(2 * np.pi * self.frequency * t) * perp_dir

        elif self.maneuver_type == ManeuverType.BARREL_ROLL:
            # 桶滚机动：圆周运动叠加
            ax = self.amplitude * np.sin(2 * np.pi * self.frequency * t)
            ay = self.amplitude * np.cos(2 * np.pi * self.frequency * t)
            return np.array([ax, ay])

        elif self.maneuver_type == ManeuverType.BREAK_TURN:
            # 急转弯：在特定时刻突然转向
            if 1.5 < t < 2.0:
                vel_dir = target.vel / (np.linalg.norm(target.vel) + 1e-6)
                perp_dir = np.array([-vel_dir[1], vel_dir[0]])
                return self.amplitude * 2 * perp_dir
            return np.array([0.0, 0.0])

        elif self.maneuver_type == ManeuverType.SPIRAL:
            # 螺旋机动：逐渐增强的转弯
            vel_dir = target.vel / (np.linalg.norm(target.vel) + 1e-6)
            perp_dir = np.array([-vel_dir[1], vel_dir[0]])
            return self.amplitude * (0.5 + 0.5 * np.sin(2 * np.pi * self.frequency * t)) * perp_dir

        elif self.maneuver_type == ManeuverType.RANDOM:
            # 随机机动
            if int(t / 0.3) != int((t - 0.05) / 0.3):
                self.random_phase = self.rng.random() * 2 * np.pi
            ax = self.amplitude * np.cos(self.random_phase)
            ay = self.amplitude * np.sin(self.random_phase)
            return np.array([ax, ay])

        elif self.maneuver_type == ManeuverType.EVASIVE:
            # 规避机动：根据拦截器位置主动规避
            if interceptor is None:
                return np.array([0.0, 0.0])

            rel_pos = interceptor.pos - target.pos
            distance = np.linalg.norm(rel_pos)

            if distance < 300:  # 当拦截器靠近时开始规避
                # 垂直于威胁方向加速
                threat_dir = rel_pos / (distance + 1e-6)
                evade_dir = np.array([-threat_dir[1], threat_dir[0]])
                # 规避强度随距离减小而增加
                intensity = min(2.0, 200 / (distance + 1e-6))
                return self.amplitude * intensity * evade_dir

            return np.array([0.0, 0.0])

        return np.array([0.0, 0.0])
