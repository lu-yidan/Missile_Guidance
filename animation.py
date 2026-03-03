"""
Missile Intercept Animation Module
导弹拦截动画模块
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from typing import List, Optional
import os

from guidance import State, GuidanceMethod, ManeuverType
from simulator import Simulator, SimConfig, SimResult, run_comparison


# 颜色方案
COLORS = {
    'target': '#E74C3C',
    'interceptor': '#3498DB',
    'trail_target': '#E74C3C',
    'trail_interceptor': '#3498DB',
    'los': '#2ECC71',
    'hit': '#F39C12',
    'miss': '#95A5A6',
}

METHOD_COLORS = {
    'pursuit': '#E74C3C',
    'predictive': '#3498DB',
    'proportional_navigation': '#2ECC71',
    'augmented_pn': '#9B59B6',
    'true_pn': '#F39C12',
    'optimal': '#1ABC9C',
}


def create_single_animation(result: SimResult,
                           output_path: str = 'output/intercept.gif',
                           fps: int = 20,
                           show_los: bool = True,
                           show_trail: bool = True) -> str:
    """
    创建单个拦截动画

    Args:
        result: 仿真结果
        output_path: 输出路径
        fps: 帧率
        show_los: 显示视线
        show_trail: 显示轨迹

    Returns:
        输出文件路径
    """
    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    traj_t = result.target_trajectory
    traj_i = result.interceptor_trajectory
    n_frames = len(traj_t)

    fig, ax = plt.subplots(figsize=(10, 8))

    # 计算坐标范围
    all_x = np.concatenate([traj_t[:, 0], traj_i[:, 0]])
    all_y = np.concatenate([traj_t[:, 1], traj_i[:, 1]])
    margin = 80
    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_y.min() - margin, all_y.max() + margin)

    # 绘图元素
    elements = {}

    if show_trail:
        elements['target_trail'], = ax.plot([], [], '-',
            color=COLORS['trail_target'], linewidth=2, alpha=0.6, label='Target')
        elements['interceptor_trail'], = ax.plot([], [], '-',
            color=COLORS['trail_interceptor'], linewidth=2, alpha=0.6, label='Interceptor')

    elements['target'], = ax.plot([], [], 'o',
        color=COLORS['target'], markersize=14)
    elements['interceptor'], = ax.plot([], [], '^',
        color=COLORS['interceptor'], markersize=14)

    if show_los:
        elements['los'], = ax.plot([], [], '--',
            color=COLORS['los'], linewidth=1.5, alpha=0.7, label='LOS')

    # 信息文本
    info_text = ax.text(0.02, 0.96, '', transform=ax.transAxes,
                        fontsize=11, verticalalignment='top',
                        fontfamily='monospace',
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # 标题和标签
    method_name = result.guidance_method.replace('_', ' ').title()
    maneuver_name = result.maneuver_type.replace('_', ' ').title()
    ax.set_title(f'{method_name} vs {maneuver_name} Target', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    def init():
        for elem in elements.values():
            elem.set_data([], [])
        info_text.set_text('')
        return list(elements.values()) + [info_text]

    def animate(frame):
        # 轨迹
        if show_trail:
            elements['target_trail'].set_data(traj_t[:frame+1, 0], traj_t[:frame+1, 1])
            elements['interceptor_trail'].set_data(traj_i[:frame+1, 0], traj_i[:frame+1, 1])

        # 当前位置
        elements['target'].set_data([traj_t[frame, 0]], [traj_t[frame, 1]])
        elements['interceptor'].set_data([traj_i[frame, 0]], [traj_i[frame, 1]])

        # 视线
        if show_los:
            elements['los'].set_data(
                [traj_i[frame, 0], traj_t[frame, 0]],
                [traj_i[frame, 1], traj_t[frame, 1]]
            )

        # 信息
        time = result.times[frame]
        dist = np.linalg.norm(traj_t[frame] - traj_i[frame])
        info_text.set_text(f'Time: {time:.2f} s\nDistance: {dist:.1f} m')

        # 命中效果
        if frame == n_frames - 1:
            if result.hit:
                elements['target'].set_color(COLORS['hit'])
                elements['target'].set_markersize(22)
                elements['interceptor'].set_markersize(22)
                ax.set_title(f'{method_name} - HIT! (t={result.hit_time:.2f}s)',
                           fontsize=14, fontweight='bold', color='green')
            else:
                ax.set_title(f'{method_name} - MISS (d={result.miss_distance:.1f}m)',
                           fontsize=14, fontweight='bold', color='red')

        return list(elements.values()) + [info_text]

    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=n_frames, interval=1000//fps, blit=True
    )

    print(f"Generating animation: {output_path}")
    anim.save(output_path, writer='pillow', fps=fps)
    plt.close()

    return output_path


def create_comparison_animation(results: List[SimResult],
                                output_path: str = 'output/comparison.gif',
                                fps: int = 20) -> str:
    """
    创建多种制导方法对比动画

    Args:
        results: 多个仿真结果
        output_path: 输出路径
        fps: 帧率

    Returns:
        输出文件路径
    """
    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    n_methods = len(results)
    n_cols = min(3, n_methods)
    n_rows = (n_methods + n_cols - 1) // n_cols

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(6*n_cols, 5*n_rows))
    if n_methods == 1:
        axes = np.array([axes])
    axes = axes.flatten()

    # 找出最长的轨迹
    max_frames = max(len(r.target_trajectory) for r in results)

    # 全局坐标范围
    all_x = np.concatenate([r.target_trajectory[:, 0] for r in results] +
                           [r.interceptor_trajectory[:, 0] for r in results])
    all_y = np.concatenate([r.target_trajectory[:, 1] for r in results] +
                           [r.interceptor_trajectory[:, 1] for r in results])
    margin = 60
    xlim = (all_x.min() - margin, all_x.max() + margin)
    ylim = (all_y.min() - margin, all_y.max() + margin)

    elements_list = []

    for idx, (ax, result) in enumerate(zip(axes, results)):
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)

        method_name = result.guidance_method.replace('_', ' ').title()
        color = METHOD_COLORS.get(result.guidance_method, '#3498DB')

        elements = {
            'target_trail': ax.plot([], [], '-', color=COLORS['target'],
                                    linewidth=1.5, alpha=0.5)[0],
            'interceptor_trail': ax.plot([], [], '-', color=color,
                                         linewidth=2, alpha=0.7)[0],
            'target': ax.plot([], [], 'o', color=COLORS['target'], markersize=10)[0],
            'interceptor': ax.plot([], [], '^', color=color, markersize=10)[0],
            'title': ax.set_title(method_name, fontsize=12),
        }
        elements_list.append(elements)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

    # 隐藏多余的子图
    for idx in range(n_methods, len(axes)):
        axes[idx].set_visible(False)

    maneuver_name = results[0].maneuver_type.replace('_', ' ').title()
    fig.suptitle(f'Guidance Methods Comparison - {maneuver_name} Target',
                fontsize=14, fontweight='bold')

    def init():
        all_elements = []
        for elements in elements_list:
            for key, elem in elements.items():
                if key != 'title':
                    elem.set_data([], [])
                    all_elements.append(elem)
        return all_elements

    def animate(frame):
        all_elements = []
        for idx, (result, elements, ax) in enumerate(zip(results, elements_list, axes)):
            traj_t = result.target_trajectory
            traj_i = result.interceptor_trajectory
            n_frames = len(traj_t)

            # 处理不同长度的轨迹
            f = min(frame, n_frames - 1)

            elements['target_trail'].set_data(traj_t[:f+1, 0], traj_t[:f+1, 1])
            elements['interceptor_trail'].set_data(traj_i[:f+1, 0], traj_i[:f+1, 1])
            elements['target'].set_data([traj_t[f, 0]], [traj_t[f, 1]])
            elements['interceptor'].set_data([traj_i[f, 0]], [traj_i[f, 1]])

            # 结束状态
            if f == n_frames - 1:
                method_name = result.guidance_method.replace('_', ' ').title()
                if result.hit:
                    ax.set_title(f'{method_name} - HIT!', color='green', fontsize=12)
                else:
                    ax.set_title(f'{method_name} - Miss ({result.miss_distance:.0f}m)',
                               color='red', fontsize=12)

            for key, elem in elements.items():
                if key != 'title':
                    all_elements.append(elem)

        return all_elements

    plt.tight_layout()

    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=max_frames, interval=1000//fps, blit=True
    )

    print(f"Generating comparison animation: {output_path}")
    anim.save(output_path, writer='pillow', fps=fps)
    plt.close()

    return output_path


def create_maneuver_showcase(output_dir: str = 'output') -> List[str]:
    """
    创建各种机动类型的展示动画

    Args:
        output_dir: 输出目录

    Returns:
        生成的文件列表
    """
    os.makedirs(output_dir, exist_ok=True)

    # 初始条件
    target_init = State(
        pos=np.array([0.0, 500.0]),
        vel=np.array([80.0, -10.0])
    )
    interceptor_init = State(
        pos=np.array([200.0, 0.0]),
        vel=np.array([0.0, 80.0])
    )

    config = SimConfig(
        interceptor_speed=220.0,
        target_speed=100.0,
        max_time=12.0
    )

    sim = Simulator(config)
    output_files = []

    maneuvers = [
        (ManeuverType.NONE, "none"),
        (ManeuverType.WEAVE, "weave"),
        (ManeuverType.BARREL_ROLL, "barrel_roll"),
        (ManeuverType.BREAK_TURN, "break_turn"),
        (ManeuverType.SPIRAL, "spiral"),
        (ManeuverType.EVASIVE, "evasive"),
    ]

    for maneuver_type, name in maneuvers:
        result = sim.run(
            target_init.copy(),
            interceptor_init.copy(),
            guidance_method=GuidanceMethod.PN,
            maneuver_type=maneuver_type,
            maneuver_amplitude=35.0
        )

        output_path = os.path.join(output_dir, f'maneuver_{name}.gif')
        create_single_animation(result, output_path)
        output_files.append(output_path)

        status = "HIT" if result.hit else f"MISS ({result.miss_distance:.1f}m)"
        print(f"  {name}: {status}")

    return output_files


def create_method_comparison(maneuver_type: ManeuverType = ManeuverType.WEAVE,
                            output_path: str = 'output/method_comparison.gif') -> str:
    """
    创建制导方法对比动画

    Args:
        maneuver_type: 目标机动类型
        output_path: 输出路径

    Returns:
        输出文件路径
    """
    target_init = State(
        pos=np.array([0.0, 500.0]),
        vel=np.array([80.0, -10.0])
    )
    interceptor_init = State(
        pos=np.array([200.0, 0.0]),
        vel=np.array([0.0, 80.0])
    )

    methods = [
        GuidanceMethod.PURSUIT,
        GuidanceMethod.PREDICTIVE,
        GuidanceMethod.PN,
        GuidanceMethod.APN,
        GuidanceMethod.TPN,
        GuidanceMethod.OPTIMAL,
    ]

    config = SimConfig(
        interceptor_speed=220.0,
        target_speed=100.0,
        max_time=12.0
    )

    results = run_comparison(
        target_init, interceptor_init, methods,
        maneuver_type=maneuver_type,
        config=config
    )

    return create_comparison_animation(results, output_path)
