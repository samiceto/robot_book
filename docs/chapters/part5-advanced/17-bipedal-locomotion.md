# Chapter 17: Bipedal Locomotion

## Learning Objectives

1. **Understand** Zero Moment Point (ZMP) stability criterion
2. **Implement** basic walking gait with MPC
3. **Simulate** bipedal walking in Isaac Sim
4. **Tune** gait parameters for stable locomotion
5. **Deploy** walking controller to humanoid hardware

---

## 1. Bipedal Stability Fundamentals

### Zero Moment Point (ZMP)
**Definition**: Point on ground where total moment from gravity and inertia equals zero.

**Stability Criterion**: ZMP must stay within support polygon (foot contact area).

```
If ZMP inside foot → Stable
If ZMP outside foot → Robot will fall
```

### Center of Mass (CoM)
```python
def compute_com(link_positions, link_masses):
    """Compute center of mass position."""
    total_mass = sum(link_masses)
    com = sum(pos * mass for pos, mass in zip(link_positions, link_masses))
    return com / total_mass
```

---

## 2. Walking Gait Basics

### Gait Phases
```
1. Double Support (both feet on ground)
2. Single Support (one foot on ground)
3. Swing Phase (moving foot)
4. Heel Strike (foot lands)
```

### Trajectory Generation
```python
import numpy as np

def generate_foot_trajectory(step_length=0.2, step_height=0.05, duration=0.8):
    """Generate swing foot trajectory (parabola)."""
    t = np.linspace(0, duration, 100)

    # X: Linear motion forward
    x = step_length * t / duration

    # Z: Parabolic swing
    z = 4 * step_height * t / duration * (1 - t / duration)

    return x, z
```

---

## 3. Model Predictive Control (MPC)

**Idea**: Plan future robot states to keep ZMP stable.

```python
class WalkingMPC:
    def __init__(self, horizon=10):
        self.horizon = horizon  # Prediction steps

    def solve(self, current_state, zmp_reference):
        """
        Solve MPC optimization.

        Returns:
            Optimal CoM trajectory
        """
        # Simplified linear MPC
        # Minimize: ||CoM - reference||^2 + ||ZMP - zmp_reference||^2

        # Returns next CoM velocity
        return com_velocity_command
```

---

## 4. Isaac Sim Implementation

### Setup Humanoid
```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot

world = World()
humanoid = Robot(prim_path="/World/Humanoid", name="walker")

# Configure ground contact
ground = world.scene.add_default_ground_plane()

# Reset to standing pose
standing_joints = [0, 0, 0, -0.2, 0.4, -0.2,  # Left leg
                   0, 0, 0, -0.2, 0.4, -0.2]  # Right leg
humanoid.set_joint_positions(standing_joints)
```

### Walking Controller
```python
class WalkingController:
    def __init__(self):
        self.phase = 0  # Gait phase (0-1)
        self.step_time = 0.8  # seconds per step

    def update(self, dt):
        self.phase += dt / self.step_time
        if self.phase > 1:
            self.phase -= 1

        # Generate foot positions
        left_foot_pos = self.get_foot_position("left", self.phase)
        right_foot_pos = self.get_foot_position("right", self.phase)

        # Inverse kinematics to get joint angles
        joint_targets = self.ik_solver(left_foot_pos, right_foot_pos)

        return joint_targets
```

---

## 5. Hands-On Lab: Walking Simulation (3 hours)

**Goal**: Implement basic walking in Isaac Sim.

**Steps**:
1. Load humanoid in Isaac Sim
2. Implement ZMP calculation
3. Generate walking trajectory
4. Tune step length (0.1-0.3m) and frequency (0.5-2 Hz)
5. Achieve 10 consecutive steps without falling

**Validation**: Robot walks forward 2 meters stably

---

## 6. End-of-Chapter Project

Implement outdoor walking with uneven terrain.

**Requirements**:
- Walking controller with MPC
- Terrain adaptation (detect slopes)
- Fall recovery behavior
- Walk 5 meters on uneven ground in simulation

**Deliverables**:
- Walking controller code
- Simulation video
- Performance analysis (stability, speed)

---

## Summary

Bipedal locomotion requires careful balance control using ZMP stability criterion and MPC for trajectory planning. Isaac Sim enables safe testing of walking gaits before hardware deployment.

**Next**: Chapter 18 covers whole-body control for simultaneous locomotion and manipulation.
