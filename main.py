#!/usr/bin/env python3
"""
Missile Guidance Simulation - Main Entry Point
导弹制导仿真主程序

Usage:
    python main.py [command] [options]

Commands:
    single      - Run single guidance simulation
    compare     - Compare multiple guidance methods
    maneuvers   - Showcase different target maneuvers
    all         - Generate all demonstrations
"""

import argparse
import numpy as np
import sys

from guidance import State, GuidanceMethod, ManeuverType, GuidanceParams
from simulator import Simulator, SimConfig
from animation import (
    create_single_animation,
    create_comparison_animation,
    create_method_comparison,
    create_maneuver_showcase
)


def get_default_scenario():
    """获取默认场景"""
    target = State(
        pos=np.array([0.0, 500.0]),
        vel=np.array([80.0, -10.0])
    )
    interceptor = State(
        pos=np.array([200.0, 0.0]),
        vel=np.array([0.0, 80.0])
    )
    return target, interceptor


def cmd_single(args):
    """单次仿真"""
    target, interceptor = get_default_scenario()

    # 解析制导方法
    method_map = {m.value: m for m in GuidanceMethod}
    method = method_map.get(args.method, GuidanceMethod.PN)

    # 解析机动类型
    maneuver_map = {m.value: m for m in ManeuverType}
    maneuver = maneuver_map.get(args.maneuver, ManeuverType.NONE)

    config = SimConfig(
        interceptor_speed=args.speed,
        max_time=args.max_time
    )

    print(f"Running simulation:")
    print(f"  Guidance: {method.value}")
    print(f"  Maneuver: {maneuver.value}")
    print(f"  Interceptor speed: {args.speed} m/s")

    sim = Simulator(config)
    result = sim.run(
        target, interceptor,
        guidance_method=method,
        maneuver_type=maneuver,
        maneuver_amplitude=args.amplitude
    )

    print(f"\nResult:")
    print(f"  Hit: {result.hit}")
    if result.hit:
        print(f"  Hit time: {result.hit_time:.2f} s")
    print(f"  Miss distance: {result.miss_distance:.2f} m")

    if args.animate:
        output = args.output or f"output/{method.value}_{maneuver.value}.gif"
        create_single_animation(result, output, fps=args.fps)
        print(f"\nAnimation saved to: {output}")


def cmd_compare(args):
    """对比多种制导方法"""
    target, interceptor = get_default_scenario()

    # 解析机动类型
    maneuver_map = {m.value: m for m in ManeuverType}
    maneuver = maneuver_map.get(args.maneuver, ManeuverType.WEAVE)

    print(f"Comparing guidance methods against {maneuver.value} target...")

    output = args.output or f"output/comparison_{maneuver.value}.gif"
    create_method_comparison(maneuver, output)

    print(f"\nComparison animation saved to: {output}")


def cmd_maneuvers(args):
    """展示各种机动"""
    print("Generating maneuver showcase animations...")
    output_dir = args.output_dir or 'maneuvers'

    files = create_maneuver_showcase(output_dir)

    print(f"\nGenerated {len(files)} animations in '{output_dir}/'")


def cmd_all(args):
    """生成所有演示"""
    import os
    os.makedirs('output', exist_ok=True)

    print("=" * 50)
    print("Generating all demonstrations...")
    print("=" * 50)

    target, interceptor = get_default_scenario()
    config = SimConfig(interceptor_speed=220.0, max_time=12.0)
    sim = Simulator(config)

    # 1. 基础PN演示
    print("\n[1/4] Basic PN demonstration...")
    result = sim.run(target.copy(), interceptor.copy(),
                     GuidanceMethod.PN, ManeuverType.NONE)
    create_single_animation(result, 'output/demo_basic_pn.gif')

    # 2. 蛇形机动对抗
    print("\n[2/4] PN vs weaving target...")
    result = sim.run(target.copy(), interceptor.copy(),
                     GuidanceMethod.PN, ManeuverType.WEAVE,
                     maneuver_amplitude=35.0)
    create_single_animation(result, 'output/demo_pn_vs_weave.gif')

    # 3. 制导方法对比（无机动）
    print("\n[3/4] Guidance methods comparison (no maneuver)...")
    create_method_comparison(ManeuverType.NONE, 'output/demo_comparison_none.gif')

    # 4. 制导方法对比（蛇形机动）
    print("\n[4/4] Guidance methods comparison (weave maneuver)...")
    create_method_comparison(ManeuverType.WEAVE, 'output/demo_comparison_weave.gif')

    print("\n" + "=" * 50)
    print("All demonstrations generated!")
    print("=" * 50)
    print("\nGenerated files in output/:")
    print("  - demo_basic_pn.gif")
    print("  - demo_pn_vs_weave.gif")
    print("  - demo_comparison_none.gif")
    print("  - demo_comparison_weave.gif")


def main():
    parser = argparse.ArgumentParser(
        description='Missile Guidance Simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py single --method pn --maneuver weave --animate
  python main.py compare --maneuver evasive
  python main.py maneuvers
  python main.py all

Guidance Methods:
  pursuit                 - Pure pursuit (always point at target)
  predictive              - Lead pursuit (point at predicted position)
  proportional_navigation - PN (classic missile guidance)
  augmented_pn            - APN (PN with target acceleration)
  true_pn                 - TPN (acceleration perpendicular to LOS)
  optimal                 - Optimal guidance (ZEM-based)

Maneuver Types:
  none        - No maneuver (constant velocity)
  weave       - Sinusoidal weaving
  barrel_roll - Circular barrel roll
  break_turn  - Sudden break turn
  spiral      - Spiral maneuver
  random      - Random direction changes
  evasive     - Active evasion (reacts to interceptor)
"""
    )

    subparsers = parser.add_subparsers(dest='command', help='Command to run')

    # Single simulation
    p_single = subparsers.add_parser('single', help='Run single simulation')
    p_single.add_argument('--method', '-m', default='proportional_navigation',
                          help='Guidance method')
    p_single.add_argument('--maneuver', '-n', default='none',
                          help='Target maneuver type')
    p_single.add_argument('--speed', '-s', type=float, default=220.0,
                          help='Interceptor speed (m/s)')
    p_single.add_argument('--amplitude', '-a', type=float, default=30.0,
                          help='Maneuver amplitude (m/s^2)')
    p_single.add_argument('--max-time', '-t', type=float, default=15.0,
                          help='Maximum simulation time (s)')
    p_single.add_argument('--animate', action='store_true',
                          help='Generate animation')
    p_single.add_argument('--output', '-o', help='Output file path')
    p_single.add_argument('--fps', type=int, default=20, help='Animation FPS')

    # Compare methods
    p_compare = subparsers.add_parser('compare', help='Compare guidance methods')
    p_compare.add_argument('--maneuver', '-n', default='weave',
                           help='Target maneuver type')
    p_compare.add_argument('--output', '-o', help='Output file path')

    # Maneuver showcase
    p_maneuvers = subparsers.add_parser('maneuvers', help='Showcase maneuvers')
    p_maneuvers.add_argument('--output-dir', '-o', default='maneuvers',
                             help='Output directory')

    # All demos
    p_all = subparsers.add_parser('all', help='Generate all demonstrations')

    args = parser.parse_args()

    if args.command == 'single':
        cmd_single(args)
    elif args.command == 'compare':
        cmd_compare(args)
    elif args.command == 'maneuvers':
        cmd_maneuvers(args)
    elif args.command == 'all':
        cmd_all(args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
