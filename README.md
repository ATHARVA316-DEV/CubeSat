# CubeSat
# Enhanced CubeSat Attitude Control Simulation

A high-fidelity physics-based simulation of a 1U CubeSat with regenerative reaction wheels for 3-axis attitude control. Features real-time parameter tuning, quaternion-based control, and comprehensive power system modeling.

![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![PyBullet](https://img.shields.io/badge/PyBullet-3.2.5+-green.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

## 🚀 Features

### Advanced Control Systems
- **Quaternion-Based Control**: Avoids gimbal lock for large-angle maneuvers
- **3-Axis PID Controllers**: Independent control for roll, pitch, and yaw
- **Real-Time Tuning**: Live GUI sliders for PID gains and disturbances
- **Momentum Management**: Saturation detection and warning system

### Realistic Physics Modeling
- **Bearing Friction**: Velocity-dependent drag on reaction wheels
- **Temperature Simulation**: Thermal modeling of wheel bearings
- **Environmental Disturbances**: Configurable external torques (solar pressure, magnetic field)
- **Initial Tumble**: Configurable multi-axis angular velocity

### Regenerative Power System
- **Bidirectional Power Flow**: Energy recovery during wheel deceleration
- **Dual Storage**: Battery for consumption, supercapacitor for regeneration
- **Solar Charging**: Simulated solar panel input with duty cycle
- **Efficiency Modeling**: Separate motor drive (85%) and regen (75%) efficiencies
- **Idle Power Draw**: Electronics baseline consumption

### Comprehensive Visualization
6 synchronized plots showing:
1. **Detumble Performance**: Angular velocity and attitude error
2. **Reaction Wheel Dynamics**: RPM of all three wheels
3. **Momentum Saturation**: Real-time saturation levels with warnings
4. **Thermal Monitoring**: Bearing temperature for each wheel
5. **Power Profile**: Consumption vs regeneration over time
6. **Energy Storage**: Battery and supercapacitor levels

## 📋 Requirements

```bash
pip install pybullet numpy matplotlib simple-pid
```

**Dependencies:**
- Python 3.8+
- PyBullet 3.2.5+
- NumPy 1.20+
- Matplotlib 3.3+
- simple-pid 1.2+

## 🎯 Quick Start

### Basic Usage

```python
from cubesat_sim import SimConfig, Simulation

# Use default configuration
config = SimConfig()
sim = Simulation(config)

try:
    sim.run()
    sim.print_summary()
    sim.plot()
finally:
    sim.cleanup()
```

### Custom Configuration

```python
# Create custom configuration
config = SimConfig(
    SIM_DURATION_S=60.0,           # 60 second simulation
    INITIAL_TUMBLE=(30, -40, 25),  # Higher initial tumble rate
    PID_KP=0.03,                   # Adjust PID gains
    PID_KI=0.005,
    PID_KD=0.05,
    SOLAR_W=3.0,                   # Increase solar power
    RW_MAX_RPM=10000               # Higher wheel speed limit
)

sim = Simulation(config)
sim.run()
```

## 🎮 Interactive Controls

### GUI Sliders

The simulation provides real-time control through interactive sliders:

**PID Tuning:**
- `Kp` (0-0.1): Proportional gain - response speed
- `Ki` (0-0.05): Integral gain - steady-state error elimination
- `Kd` (0-0.1): Derivative gain - damping

**Target Attitude:**
- `Target Roll` (-180° to +180°): Desired roll angle
- `Target Pitch` (-180° to +180°): Desired pitch angle
- `Target Yaw` (-180° to +180°): Desired yaw angle

**Environmental Disturbances:**
- `Dist X/Y/Z` (-0.5 to +0.5 mNm): External torques on each axis

### Live Status Display

Real-time HUD shows:
- Current simulation time
- Angular velocity magnitude
- Quaternion error norm
- Saturation warnings

## 📊 Configuration Parameters

### CubeSat Properties
```python
CUBESAT_MASS_KG = 1.3          # 1U standard mass
CUBESAT_DIMS_M = (0.1, 0.1, 0.1)  # 10cm cube
```

### Reaction Wheel Specifications
```python
RW_MASS_KG = 0.1               # Wheel mass
RW_RADIUS_M = 0.04             # 40mm radius
RW_MAX_TORQUE_NM = 0.005       # 5 mNm max torque
RW_MAX_RPM = 8000              # Maximum speed
RW_FRICTION_NM = 0.00005       # Bearing friction
```

### Power System
```python
MOTOR_EFF = 0.85               # 85% motor efficiency
REGEN_EFF = 0.75               # 75% regen efficiency
BATTERY_J = 2000.0             # 2000 J capacity
SUPERCAP_J = 150.0             # 150 J supercapacitor
SOLAR_W = 2.0                  # 2W solar panels
IDLE_W = 0.5                   # 0.5W idle draw
```

### Control Parameters
```python
PID_KP = 0.025                 # Proportional gain
PID_KI = 0.004                 # Integral gain
PID_KD = 0.04                  # Derivative gain
```

### Environment
```python
INITIAL_TUMBLE = (15, -25, 20)  # deg/s on each axis
DISTURBANCE_NM = (0.0001, -0.0001, 0.00005)  # External torques
```

## 🔬 Technical Details

### Quaternion Control

The simulation uses quaternion error vectors for attitude control, avoiding gimbal lock:

```python
error_vector = quat_error_vector(current_quat, target_quat)
control_torque = PID(angular_velocity + error_vector * gain)
```

This provides:
- ✅ No singularities at any orientation
- ✅ Smooth large-angle maneuvers
- ✅ Globally stable control

### Power Flow Logic

**Acceleration Phase (Consuming):**
```
Electrical Power = |Mechanical Power| / Motor_Efficiency
Energy drawn from battery
```

**Deceleration Phase (Regenerating):**
```
Electrical Power = -|Mechanical Power| * Regen_Efficiency
Energy stored in supercapacitor
```

**Energy Transfer:**
```
Supercap → Battery (90% efficiency)
Solar → Battery (100% efficiency)
```

### Momentum Saturation

Wheels track momentum saturation:
```python
saturation_level = |wheel_momentum| / max_momentum * 100%
```

Warnings trigger at 90% saturation. When saturated, control authority is reduced:
```python
if saturated:
    control_torque *= (1.0 - saturation_level)
```

## 📈 Output Statistics

The simulation provides comprehensive performance metrics:

```
=== Simulation Summary ===
Energy Consumed: 45.32 J
Energy Regenerated: 12.67 J
Power Savings: 27.9%
Max Wheel Temps: X:28.4°C Y:31.2°C Z:26.8°C
Max Saturation: X:67.3% Y:72.1% Z:55.8%
Final State: ω=0.023°/s, Error=0.0012
==========================
```

**Metrics Explained:**
- **Energy Consumed**: Total electrical energy used by reaction wheels
- **Energy Regenerated**: Total energy recovered to supercapacitor
- **Power Savings**: Percentage of consumed energy that was regenerated
- **Max Temps**: Peak bearing temperature during simulation
- **Max Saturation**: Highest momentum saturation level reached
- **Final State**: Final angular velocity and attitude error

## 🎓 Use Cases

### Educational
- Spacecraft attitude dynamics
- PID control system design
- Quaternion mathematics
- Energy management systems

### Research
- Control algorithm development
- Power system optimization
- Hardware-in-the-loop testing preparation
- Mission planning and analysis

### Development
- Flight software validation
- Controller gain tuning
- Failure mode analysis
- Performance benchmarking

## 🔧 Troubleshooting

### Simulation Won't Start
```bash
# Check PyBullet installation
python -c "import pybullet as p; print(p.connect(p.DIRECT))"

# Reinstall if needed
pip install --upgrade pybullet
```

### Unstable Control
- **Issue**: CubeSat oscillates wildly
- **Solution**: Reduce PID gains, especially Kp and Kd
- **Try**: Kp=0.015, Ki=0.002, Kd=0.025

### Poor Detumble Performance
- **Issue**: Takes too long to stabilize
- **Solution**: Increase Kp gain for faster response
- **Note**: May increase overshoot

### High Power Consumption
- **Issue**: Battery depletes rapidly
- **Solution**: 
  - Reduce initial tumble rate
  - Increase solar power
  - Optimize PID gains for smoother control

## 🛠️ Advanced Customization

### Adding Custom Scenarios

```python
class CustomScenario(SimConfig):
    # High-speed tumble recovery
    INITIAL_TUMBLE = (50, -60, 45)
    PID_KP = 0.04
    SIM_DURATION_S = 60.0

config = CustomScenario()
```

### Modifying Physics

```python
# In ReactionWheel.update() method:
# Increase friction for older bearings
friction = -np.sign(self.vel) * 0.0001  # Double friction
```

### Custom Data Logging

```python
# Add custom metrics to Simulation class
self.data['custom_metric'] = []

# In run() loop:
self.data['custom_metric'].append(your_calculation)
```

## 📚 References

### Spacecraft Dynamics
- Wertz, J.R., "Spacecraft Attitude Determination and Control"
- Wie, B., "Space Vehicle Dynamics and Control"

### Reaction Wheels
- Sidi, M.J., "Spacecraft Dynamics and Control"
- Markley, F.L., "Fundamentals of Spacecraft Attitude Determination and Control"

### PyBullet
- Official Documentation: https://pybullet.org/
- Physics Tutorial: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA

## 🤝 Contributing

Contributions are welcome! Areas for enhancement:
- Magnetorquer momentum dumping
- IMU sensor noise modeling
- Star tracker simulation
- Orbital mechanics integration
- Multi-satellite formation flying

## 📄 License

MIT License - feel free to use in academic and commercial projects.

## 🙏 Acknowledgments

- PyBullet physics engine by Erwin Coumans
- simple-pid library by Martin Lundberg
- Inspired by real CubeSat missions worldwide

---

For questions or issues, please open a GitHub issue or contact matharva357@gmail.com .
