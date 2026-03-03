# Missile Guidance Simulation

A Python-based missile intercept guidance simulation with multiple guidance laws and target maneuver patterns.

## Features

- **6 Guidance Methods**
  - Pure Pursuit - Always point at target current position
  - Predictive (Lead Pursuit) - Point at predicted intercept position
  - Proportional Navigation (PN) - Classic missile guidance law
  - Augmented PN (APN) - PN with target acceleration compensation
  - True PN (TPN) - Acceleration perpendicular to line-of-sight
  - Optimal Guidance - Zero-effort-miss based guidance

- **7 Target Maneuver Patterns**
  - None - Constant velocity flight
  - Weave - Sinusoidal weaving maneuver
  - Barrel Roll - Circular motion overlay
  - Break Turn - Sudden hard turn
  - Spiral - Gradually increasing turn
  - Random - Random direction changes
  - Evasive - Active evasion (reacts to interceptor)

- **Visualization**
  - Animated GIF output
  - Multi-method comparison
  - Real-time distance and time display

## Installation

### Using Conda (Recommended)

```bash
# Create environment
conda create -n missile_guidance python=3.10
conda activate missile_guidance

# Install dependencies
pip install -r requirements.txt
```

### Using pip

```bash
pip install -r requirements.txt
```

## Usage

### Quick Demo

```bash
# Generate all demonstration animations
python main.py all
```

### Single Simulation

```bash
# Basic PN guidance vs non-maneuvering target
python main.py single --method proportional_navigation --maneuver none --animate

# PN vs weaving target
python main.py single -m proportional_navigation -n weave --animate

# Optimal guidance vs evasive target
python main.py single -m optimal -n evasive -a 40 --animate
```

### Compare Methods

```bash
# Compare all methods against weaving target
python main.py compare --maneuver weave

# Compare against evasive target
python main.py compare -n evasive -o evasive_comparison.gif
```

### Maneuver Showcase

```bash
# Generate animations for all maneuver types
python main.py maneuvers --output-dir maneuvers/
```

## Command Reference

```
python main.py [command] [options]

Commands:
  single      Run single guidance simulation
  compare     Compare multiple guidance methods
  maneuvers   Showcase different target maneuvers
  all         Generate all demonstrations

Options for 'single':
  --method, -m      Guidance method (default: proportional_navigation)
  --maneuver, -n    Target maneuver type (default: none)
  --speed, -s       Interceptor speed in m/s (default: 220)
  --amplitude, -a   Maneuver amplitude in m/s² (default: 30)
  --max-time, -t    Max simulation time in seconds (default: 15)
  --animate         Generate animation GIF
  --output, -o      Output file path
  --fps             Animation frames per second (default: 20)
```

## Guidance Methods

| Method | Description | Best For |
|--------|-------------|----------|
| `pursuit` | Always flies toward target | Slow targets |
| `predictive` | Flies to predicted intercept point | Non-maneuvering targets |
| `proportional_navigation` | Acceleration ∝ LOS rate | General purpose |
| `augmented_pn` | PN + target acceleration term | Maneuvering targets |
| `true_pn` | Acceleration ⊥ to LOS | Energy efficient |
| `optimal` | ZEM-based optimal control | End-game guidance |

## Maneuver Types

| Maneuver | Description |
|----------|-------------|
| `none` | Straight line, constant velocity |
| `weave` | Sinusoidal oscillation perpendicular to velocity |
| `barrel_roll` | Circular motion overlay |
| `break_turn` | Hard turn at specific time |
| `spiral` | Increasing turn rate |
| `random` | Random direction changes |
| `evasive` | Actively evades when interceptor approaches |

## Project Structure

```
Missile_Guidance/
├── main.py           # Main entry point / CLI
├── guidance.py       # Guidance laws and maneuvers
├── simulator.py      # Simulation engine
├── animation.py      # Visualization module
├── requirements.txt  # Python dependencies
└── README.md         # This file
```

## Algorithm Details

### Proportional Navigation (PN)

The classic missile guidance law:

```
a = N × Vc × λ̇
```

Where:
- `a` = commanded acceleration
- `N` = navigation constant (typically 3-5)
- `Vc` = closing velocity
- `λ̇` = line-of-sight rate

### Collision Triangle

For predictive guidance, solve for intercept time `t`:

```
|P_target + V_target × t - P_interceptor| = V_interceptor × t
```

This yields a quadratic equation with the intercept time as solution.

## Examples

### Example 1: Basic Intercept
```python
from guidance import State, GuidanceMethod, ManeuverType
from simulator import Simulator, SimConfig

target = State(pos=np.array([0., 500.]), vel=np.array([80., -10.]))
interceptor = State(pos=np.array([200., 0.]), vel=np.array([0., 80.]))

sim = Simulator(SimConfig(interceptor_speed=220.0))
result = sim.run(target, interceptor, GuidanceMethod.PN, ManeuverType.NONE)

print(f"Hit: {result.hit}, Miss distance: {result.miss_distance:.1f}m")
```

### Example 2: Custom Animation
```python
from animation import create_single_animation

create_single_animation(result, 'my_intercept.gif', fps=30, show_los=True)
```

## License

MIT License
