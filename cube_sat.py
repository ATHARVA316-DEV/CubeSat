import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID
from dataclasses import dataclass
from typing import Tuple, Dict

# --- Configuration ---
@dataclass
class SimConfig:
    """Simulation parameters."""
    SIM_DURATION_S: float = 40.0
    TIME_STEP: float = 1/240.0
    
    # CubeSat
    CUBESAT_MASS_KG: float = 1.3
    CUBESAT_DIMS_M: tuple = (0.1, 0.1, 0.1)
    
    # Reaction Wheels
    RW_MASS_KG: float = 0.1
    RW_RADIUS_M: float = 0.04
    RW_MAX_TORQUE_NM: float = 0.005
    RW_MAX_RPM: float = 8000
    RW_FRICTION_NM: float = 0.00005  # Bearing friction
    
    # Control
    PID_KP: float = 0.025
    PID_KI: float = 0.004
    PID_KD: float = 0.04
    
    # Power
    MOTOR_EFF: float = 0.85
    REGEN_EFF: float = 0.75
    BATTERY_J: float = 2000.0
    SUPERCAP_J: float = 150.0
    SOLAR_W: float = 2.0
    IDLE_W: float = 0.5
    
    # Environment
    INITIAL_TUMBLE: tuple = (15, -25, 20)  # deg/s
    DISTURBANCE_NM: tuple = (0.0001, -0.0001, 0.00005)
    
    @property
    def rw_inertia(self):
        return 0.5 * self.RW_MASS_KG * self.RW_RADIUS_M**2
    
    @property
    def rw_max_rad_s(self):
        return self.RW_MAX_RPM * (2 * np.pi / 60)

# --- Utility Functions ---
def quat_error_vector(q_current, q_target):
    """Compute rotation error vector from quaternions."""
    qc = np.array(q_current)
    qt = np.array(q_target)
    
    # q_error = qt * qc_conjugate
    qc_conj = np.array([qc[0], -qc[1], -qc[2], -qc[3]])
    
    # Quaternion multiplication
    x = qt[3]*qc_conj[0] + qt[0]*qc_conj[3] + qt[1]*qc_conj[2] - qt[2]*qc_conj[1]
    y = qt[3]*qc_conj[1] - qt[0]*qc_conj[2] + qt[1]*qc_conj[3] + qt[2]*qc_conj[0]
    z = qt[3]*qc_conj[2] + qt[0]*qc_conj[1] - qt[1]*qc_conj[0] + qt[2]*qc_conj[3]
    w = qt[3]*qc_conj[3] - qt[0]*qc_conj[0] - qt[1]*qc_conj[1] - qt[2]*qc_conj[2]
    
    q_err = np.array([x, y, z, w])
    
    # Extract rotation vector
    angle = 2 * np.arccos(np.clip(q_err[3], -1, 1))
    if angle < 1e-6:
        return np.zeros(3)
    axis = q_err[:3] / np.sin(angle / 2)
    return axis * angle

# --- Component Classes ---
class PowerSystem:
    """Enhanced power management with solar charging."""
    def __init__(self, config):
        self.config = config
        self.battery = config.BATTERY_J
        self.supercap = 0.0
        self.history = {'time': [], 'battery': [], 'supercap': [], 'power': []}
        
    def update(self, rw_power, solar_on, dt, t):
        """Update energy levels."""
        # Idle consumption
        self.battery -= self.config.IDLE_W * dt
        
        # RW power
        if rw_power > 0:
            self.battery -= rw_power * dt
        else:
            self.supercap = min(self.supercap + abs(rw_power) * dt, self.config.SUPERCAP_J)
        
        # Solar charging
        if solar_on:
            self.battery = min(self.battery + self.config.SOLAR_W * dt, self.config.BATTERY_J)
        
        # Transfer supercap to battery
        if self.supercap > 0 and self.battery < self.config.BATTERY_J:
            transfer = min(self.supercap, 0.5 * dt)
            self.supercap -= transfer
            self.battery += transfer * 0.9
        
        self.history['time'].append(t)
        self.history['battery'].append(self.battery)
        self.history['supercap'].append(self.supercap)
        self.history['power'].append(rw_power + self.config.IDLE_W)
    
    def get_stats(self):
        consumed = sum(p * self.config.TIME_STEP for p in self.history['power'] if p > 0)
        regen = max(self.history['supercap'])
        return {
            'consumed': consumed,
            'regenerated': regen,
            'savings_pct': (regen / max(consumed, 1)) * 100
        }

class ReactionWheel:
    """Advanced RW with friction, saturation, and temperature."""
    def __init__(self, config, axis_name, axis_idx):
        self.config = config
        self.axis = axis_name
        self.idx = axis_idx
        self.vel = 0.0
        self.temp = 20.0
        
        self.pid = PID(
            Kp=config.PID_KP, Ki=config.PID_KI, Kd=config.PID_KD,
            setpoint=0,
            output_limits=(-config.RW_MAX_TORQUE_NM, config.RW_MAX_TORQUE_NM)
        )
    
    def update_gains(self, kp, ki, kd):
        self.pid.tunings = (kp, ki, kd)
    
    def update(self, quat_err, ang_vel, dt):
        """Compute control with quaternion feedback."""
        # Combined feedback: rate + position
        error = ang_vel + quat_err[self.idx] * 3.0
        torque = self.pid(error)
        
        # Apply torque
        delta_v = (torque / self.config.rw_inertia) * dt
        self.vel += delta_v
        
        # Friction
        friction = -np.sign(self.vel) * self.config.RW_FRICTION_NM
        self.vel += (friction / self.config.rw_inertia) * dt
        
        # Limits
        self.vel = np.clip(self.vel, -self.config.rw_max_rad_s, self.config.rw_max_rad_s)
        
        # Temperature (simplified)
        heat = abs(torque * self.vel) * 0.2
        self.temp += heat * dt - (self.temp - 20) * 0.02 * dt
        
        # Power
        mech_power = torque * self.vel
        if np.sign(torque) == np.sign(self.vel) or abs(self.vel) < 1e-3:
            power = abs(mech_power) / self.config.MOTOR_EFF
        else:
            power = -abs(mech_power) * self.config.REGEN_EFF
        
        return -torque, power
    
    def get_saturation(self):
        momentum = abs(self.config.rw_inertia * self.vel)
        max_momentum = self.config.rw_inertia * self.config.rw_max_rad_s
        return (momentum / max_momentum) * 100

class CubeSat:
    """CubeSat with quaternion-based attitude control."""
    def __init__(self, config):
        self.config = config
        
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[d/2 for d in config.CUBESAT_DIMS_M])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[d/2 for d in config.CUBESAT_DIMS_M],
                                     rgbaColor=[0.7, 0.75, 0.8, 1.0])
        
        self.body_id = p.createMultiBody(
            baseMass=config.CUBESAT_MASS_KG,
            baseCollisionShapeIndex=shape,
            baseVisualShapeIndex=visual,
            basePosition=[0, 0, 1]
        )
        
        # Initial tumble
        init_vel = [np.deg2rad(v) for v in config.INITIAL_TUMBLE]
        p.resetBaseVelocity(self.body_id, angularVelocity=init_vel)
        
        # Visual axes
        for i, color in enumerate([[1,0,0], [0,1,0], [0,0,1]]):
            endpoint = [0.12 if j==i else 0 for j in range(3)]
            p.addUserDebugLine([0,0,0], endpoint, color, 2, 
                             parentObjectUniqueId=self.body_id, parentLinkIndex=-1)
        
        self.target_quat = np.array([0, 0, 0, 1])
        
    def get_state(self):
        """Returns (quat, ang_vel_body, quat_error_vector)."""
        _, quat = p.getBasePositionAndOrientation(self.body_id)
        _, ang_vel_world = p.getBaseVelocity(self.body_id)
        
        # To body frame
        _, quat_inv = p.invertTransform([0,0,0], quat)
        ang_vel_body, _ = p.multiplyTransforms([0,0,0], quat_inv, ang_vel_world, [0,0,0,1])
        
        # Error
        quat_np = np.array(quat)
        error = quat_error_vector(quat_np, self.target_quat)
        
        return quat_np, np.array(ang_vel_body), error
    
    def set_target(self, roll, pitch, yaw):
        """Set target orientation in degrees."""
        self.target_quat = np.array(p.getQuaternionFromEuler([
            np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)
        ]))

class Simulation:
    """Main simulation orchestrator."""
    def __init__(self, config):
        self.config = config
        
        # PyBullet setup
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(0.6, 45, -20, [0, 0, 1])
        
        # Components
        self.cubesat = CubeSat(config)
        self.power = PowerSystem(config)
        self.wheels = [
            ReactionWheel(config, 'X', 0),
            ReactionWheel(config, 'Y', 1),
            ReactionWheel(config, 'Z', 2)
        ]
        
        # GUI
        self.sliders = {
            'kp': p.addUserDebugParameter("Kp", 0, 0.1, config.PID_KP),
            'ki': p.addUserDebugParameter("Ki", 0, 0.05, config.PID_KI),
            'kd': p.addUserDebugParameter("Kd", 0, 0.1, config.PID_KD),
            'roll': p.addUserDebugParameter("Target Roll", -180, 180, 0),
            'pitch': p.addUserDebugParameter("Target Pitch", -180, 180, 0),
            'yaw': p.addUserDebugParameter("Target Yaw", -180, 180, 0),
            'dx': p.addUserDebugParameter("Dist X (mNm)", -0.5, 0.5, config.DISTURBANCE_NM[0]*1000),
            'dy': p.addUserDebugParameter("Dist Y (mNm)", -0.5, 0.5, config.DISTURBANCE_NM[1]*1000),
            'dz': p.addUserDebugParameter("Dist Z (mNm)", -0.5, 0.5, config.DISTURBANCE_NM[2]*1000),
        }
        self.status_txt = p.addUserDebugText("", [0, 0, 1.3], [1,1,1], 1.2)
        
        # Data
        self.data = {
            'time': [], 'ang_vel': [], 'error': [],
            'rw_rpm': [[], [], []], 'rw_temp': [[], [], []],
            'rw_sat': [[], [], []], 'torque': [[], [], []]
        }
        
        print("\n=== Enhanced CubeSat Simulation ===")
        print(f"Duration: {config.SIM_DURATION_S}s")
        print("Features: Quaternion control, Friction, Solar, Saturation")
    
    def run(self):
        """Main loop."""
        steps = int(self.config.SIM_DURATION_S / self.config.TIME_STEP)
        
        for i in range(steps):
            t = i * self.config.TIME_STEP
            
            # Read GUI
            kp = p.readUserDebugParameter(self.sliders['kp'])
            ki = p.readUserDebugParameter(self.sliders['ki'])
            kd = p.readUserDebugParameter(self.sliders['kd'])
            for w in self.wheels:
                w.update_gains(kp, ki, kd)
            
            roll = p.readUserDebugParameter(self.sliders['roll'])
            pitch = p.readUserDebugParameter(self.sliders['pitch'])
            yaw = p.readUserDebugParameter(self.sliders['yaw'])
            self.cubesat.set_target(roll, pitch, yaw)
            
            dist = np.array([
                p.readUserDebugParameter(self.sliders['dx']) / 1000,
                p.readUserDebugParameter(self.sliders['dy']) / 1000,
                p.readUserDebugParameter(self.sliders['dz']) / 1000
            ])
            
            # Get state
            _, ang_vel, quat_err = self.cubesat.get_state()
            
            # Control
            total_power = 0
            torques = np.zeros(3)
            for idx, wheel in enumerate(self.wheels):
                t_ctrl, pwr = wheel.update(quat_err, ang_vel[idx], self.config.TIME_STEP)
                torques[idx] = t_ctrl
                total_power += pwr
            
            # Apply physics
            p.applyExternalTorque(self.cubesat.body_id, -1, dist, p.WORLD_FRAME)
            p.applyExternalTorque(self.cubesat.body_id, -1, torques, p.LINK_FRAME)
            
            # Power update (solar on 50% duty cycle)
            solar = (t % 10) < 5
            self.power.update(total_power, solar, self.config.TIME_STEP, t)
            
            # Log
            self.data['time'].append(t)
            self.data['ang_vel'].append(np.linalg.norm(ang_vel) * 180/np.pi)
            self.data['error'].append(np.linalg.norm(quat_err))
            for idx, w in enumerate(self.wheels):
                self.data['rw_rpm'][idx].append(w.vel * 9.5493)
                self.data['rw_temp'][idx].append(w.temp)
                self.data['rw_sat'][idx].append(w.get_saturation())
                self.data['torque'][idx].append(torques[idx] * 1000)
            
            # Status update
            if i % 10 == 0:
                ang_norm = np.linalg.norm(ang_vel) * 180/np.pi
                err_norm = np.linalg.norm(quat_err)
                sat = any(w.get_saturation() > 90 for w in self.wheels)
                status = f"T:{t:.1f}s | ω:{ang_norm:.2f}°/s | Err:{err_norm:.3f}"
                if sat:
                    status += " | SAT!"
                p.addUserDebugText(status, [0, 0, 1.3], 
                                  [1,1,0] if sat else [0,1,0], 1.2, 0,
                                  replaceItemUniqueId=self.status_txt)
            
            p.stepSimulation()
            time.sleep(self.config.TIME_STEP)
        
        print("✓ Simulation complete!")
    
    def plot(self):
        """Generate comprehensive plots."""
        fig, axs = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('Enhanced CubeSat Simulation Results', fontsize=16, fontweight='bold')
        
        t = self.data['time']
        colors = ['#e74c3c', '#2ecc71', '#3498db']
        labels = ['X', 'Y', 'Z']
        
        # Angular velocity & error
        ax = axs[0, 0]
        ax.plot(t, self.data['ang_vel'], 'b-', lw=2, label='Angular Rate')
        ax.set_ylabel('Angular Velocity (°/s)', color='b')
        ax.tick_params(axis='y', labelcolor='b')
        ax.grid(alpha=0.3)
        ax2 = ax.twinx()
        ax2.plot(t, self.data['error'], 'r-', lw=2, label='Attitude Error')
        ax2.set_ylabel('Quaternion Error', color='r')
        ax2.tick_params(axis='y', labelcolor='r')
        ax.set_title('Detumble & Pointing Performance')
        
        # RW speeds
        ax = axs[0, 1]
        for i, (c, l) in enumerate(zip(colors, labels)):
            ax.plot(t, self.data['rw_rpm'][i], c, lw=1.5, label=f'RW-{l}')
        ax.axhline(self.config.RW_MAX_RPM, color='r', ls='--', alpha=0.5)
        ax.axhline(-self.config.RW_MAX_RPM, color='r', ls='--', alpha=0.5)
        ax.set_ylabel('Wheel Speed (RPM)')
        ax.set_title('Reaction Wheel Dynamics')
        ax.legend()
        ax.grid(alpha=0.3)
        
        # RW saturation
        ax = axs[1, 0]
        for i, (c, l) in enumerate(zip(colors, labels)):
            ax.plot(t, self.data['rw_sat'][i], c, lw=1.5, label=f'RW-{l}')
        ax.axhline(90, color='orange', ls='--', alpha=0.7, label='Warning')
        ax.set_ylabel('Saturation (%)')
        ax.set_title('Momentum Saturation Levels')
        ax.legend()
        ax.grid(alpha=0.3)
        
        # RW temperature
        ax = axs[1, 1]
        for i, (c, l) in enumerate(zip(colors, labels)):
            ax.plot(t, self.data['rw_temp'][i], c, lw=1.5, label=f'RW-{l}')
        ax.set_ylabel('Temperature (°C)')
        ax.set_title('Bearing Temperature')
        ax.legend()
        ax.grid(alpha=0.3)
        
        # Power
        ax = axs[2, 0]
        ph = self.power.history
        ax.plot(ph['time'], ph['power'], 'purple', lw=1.5)
        ax.axhline(0, color='k', ls='--', lw=0.7)
        ax.fill_between(ph['time'], ph['power'], 0, 
                        where=np.array(ph['power']) > 0,
                        facecolor='red', alpha=0.3, label='Consumed')
        ax.fill_between(ph['time'], ph['power'], 0,
                        where=np.array(ph['power']) < 0,
                        facecolor='green', alpha=0.3, label='Regenerated')
        ax.set_ylabel('Power (W)')
        ax.set_xlabel('Time (s)')
        ax.set_title('System Power Profile')
        ax.legend()
        ax.grid(alpha=0.3)
        
        # Energy storage
        ax = axs[2, 1]
        ax.plot(ph['time'], ph['battery'], 'b-', lw=2, label='Battery')
        ax.set_ylabel('Battery (J)', color='b')
        ax.tick_params(axis='y', labelcolor='b')
        ax2 = ax.twinx()
        ax2.plot(ph['time'], ph['supercap'], 'orange', lw=2, label='Supercap')
        ax2.set_ylabel('Supercapacitor (J)', color='orange')
        ax2.tick_params(axis='y', labelcolor='orange')
        ax.set_xlabel('Time (s)')
        ax.set_title('Energy Storage Levels')
        ax.grid(alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def print_summary(self):
        """Print statistics."""
        stats = self.power.get_stats()
        print("\n=== Simulation Summary ===")
        print(f"Energy Consumed: {stats['consumed']:.2f} J")
        print(f"Energy Regenerated: {stats['regenerated']:.2f} J")
        print(f"Power Savings: {stats['savings_pct']:.1f}%")
        
        max_temps = [max(self.data['rw_temp'][i]) for i in range(3)]
        max_sats = [max(self.data['rw_sat'][i]) for i in range(3)]
        print(f"Max Wheel Temps: X:{max_temps[0]:.1f}°C Y:{max_temps[1]:.1f}°C Z:{max_temps[2]:.1f}°C")
        print(f"Max Saturation: X:{max_sats[0]:.1f}% Y:{max_sats[1]:.1f}% Z:{max_sats[2]:.1f}%")
        
        final_vel = self.data['ang_vel'][-1]
        final_err = self.data['error'][-1]
        print(f"Final State: ω={final_vel:.3f}°/s, Error={final_err:.4f}")
        print("==========================\n")
    
    def cleanup(self):
        p.disconnect()

if __name__ == "__main__":
    config = SimConfig()
    sim = Simulation(config)
    try:
        sim.run()
        sim.print_summary()
        sim.plot()
    finally:
        sim.cleanup()