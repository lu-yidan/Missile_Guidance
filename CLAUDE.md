# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

```bash
conda env create -f environment.yml
conda activate missile_guidance
# or: pip install -r requirements.txt
```

## Common Commands

```bash
# Run all demonstrations (outputs to output/)
python main.py all

# Single simulation with animation
python main.py single -m proportional_navigation -n weave --animate

# Compare all 6 guidance methods against a maneuver type
python main.py compare --maneuver evasive

# Generate animations for all 7 maneuver types
python main.py maneuvers
```

## Architecture

Four modules with a clean dependency chain: `guidance` ← `simulator` ← `animation` ← `main`

**`guidance.py`** — Pure physics, no I/O.
- `State(pos, vel)` — shared data class used everywhere
- `GuidanceMethod` / `ManeuverType` — enums for all options
- `GuidanceLaw` subclasses (`ProportionalNavigation`, `AugmentedPN`, `TruePN`, `OptimalGuidance`, etc.) each implement `compute(target, interceptor, speed) → accel`
- `TargetManeuver` generates target acceleration per timestep; `EVASIVE` mode reacts to interceptor position
- `GuidanceParams(nav_gain, max_accel)` — all guidance laws clamp output to `max_accel`

**`simulator.py`** — Euler integration loop.
- `SimConfig` controls `dt`, `max_time`, `interceptor_speed`, `target_speed`, `continue_after_miss`, `post_miss_time`
- Miss is detected when distance starts increasing after closing; after miss, interceptor flies inertially for `post_miss_time` seconds before stopping
- Both target and interceptor speeds are hard-clamped each timestep (target at 1.5× `target_speed`)
- `run_comparison()` runs the same scenario across multiple `GuidanceMethod` values

**`animation.py`** — Matplotlib Agg backend only (no display).
- `create_single_animation()` — single scenario, supports `playback_speed` (0.5 = slow-mo)
- `create_comparison_animation()` — grid of subplots, one per method
- Both functions create `output/` directory automatically
- GIF output via Pillow writer

**`main.py`** — CLI entry point, `argparse` subcommands: `single`, `compare`, `maneuvers`, `all`.

## Key Design Notes

- `matplotlib.use('Agg')` is set at import time in `animation.py` — do not call `plt.show()`
- All GIF output goes to `output/` (git-ignored)
- `State.copy()` must be called before passing to `Simulator.run()` when reusing the same initial state across multiple runs (see `run_comparison`)
- Default scenario in `main.py::get_default_scenario()` uses a slow interceptor (50 m/s) vs fast target (150 m/s) — adjust `SimConfig.interceptor_speed` for hit scenarios
