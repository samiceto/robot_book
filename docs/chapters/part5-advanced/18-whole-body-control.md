# Chapter 18: Whole-Body Control

## Learning Objectives

1. **Understand** hierarchical control for locomotion + manipulation
2. **Implement** task-space control with prioritization
3. **Solve** inverse kinematics for redundant robots
4. **Coordinate** arms and legs simultaneously
5. **Deploy** whole-body behaviors in simulation

---

## 1. Whole-Body Control Overview

**Challenge**: Control 12+ DOF humanoid to walk AND manipulate simultaneously.

**Approach**: Hierarchical task prioritization
```
Priority 1: Balance (don't fall)
Priority 2: Locomotion (reach goal)
Priority 3: Manipulation (grasp object)
```

---

## 2. Task-Space Control

### Operational Space Formulation
```python
def compute_task_torques(robot_state, desired_task):
    """
    Convert task-space goals to joint torques.

    Args:
        robot_state: Current joint positions/velocities
        desired_task: End-effector position/orientation

    Returns:
        Joint torques
    """
    # Jacobian: relates joint velocities to task velocities
    J = robot.compute_jacobian()

    # Task error
    x_current = robot.forward_kinematics()
    error = desired_task - x_current

    # Pseudoinverse control
    dq_desired = np.linalg.pinv(J) @ error

    # PD control in joint space
    torques = Kp * (dq_desired - robot.q_dot) + Kd * error

    return torques
```

---

## 3. Task Prioritization

### Null-Space Projection
```python
def prioritized_control(high_priority_task, low_priority_task):
    """
    Execute high-priority task, use remaining DOF for low-priority.
    """
    # High-priority solution
    J_high = compute_jacobian(high_priority_task)
    dq_high = np.linalg.pinv(J_high) @ high_priority_task.error

    # Null-space of high-priority task
    N = np.eye(n_dof) - np.linalg.pinv(J_high) @ J_high

    # Low-priority solution (projected to null-space)
    J_low = compute_jacobian(low_priority_task)
    dq_low = np.linalg.pinv(J_low @ N) @ low_priority_task.error

    # Combined solution
    dq_total = dq_high + N @ dq_low

    return dq_total
```

**Example**: Walk forward (priority 1) while reaching for cup (priority 2)

---

## 4. Whole-Body Inverse Kinematics

```python
from scipy.optimize import minimize

def whole_body_ik(target_positions, joint_limits):
    """
    Solve IK for multiple end-effectors.

    Args:
        target_positions: Dict of {effector_name: target_pos}
        joint_limits: (min, max) for each joint

    Returns:
        Optimal joint configuration
    """
    def cost(q):
        # Position error for all end-effectors
        error = 0
        for effector, target in target_positions.items():
            current = forward_kinematics(q, effector)
            error += np.linalg.norm(current - target)**2
        return error

    # Optimize
    result = minimize(
        cost,
        x0=current_joints,
        bounds=joint_limits,
        method='SLSQP'
    )

    return result.x
```

---

## 5. Coordinated Behaviors

### Walk-and-Reach
```python
class WalkAndReach:
    def __init__(self):
        self.walking_controller = WalkingMPC()
        self.arm_controller = TaskSpaceController()

    def update(self, dt, target_object):
        # Priority 1: Maintain balance
        zmp_command = self.walking_controller.get_zmp_target()

        # Priority 2: Walk toward object
        distance = np.linalg.norm(target_object.pos - robot.com)
        if distance > 0.5:  # Not in reach
            step_command = self.walking_controller.step_toward(target_object)
        else:
            step_command = None  # Stop walking

        # Priority 3: Reach for object (if close enough)
        if distance < 0.8:
            arm_command = self.arm_controller.reach(target_object)
        else:
            arm_command = None

        # Combine commands with prioritization
        joint_targets = self.combine_tasks(zmp_command, step_command, arm_command)

        return joint_targets
```

---

## 6. ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class WholeBodyController(Node):
    def __init__(self):
        super().__init__('whole_body_controller')

        # Subscribers
        self.create_subscription(PoseStamped, '/target_object', self.object_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.state_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Controllers
        self.controller = WalkAndReach()

    def object_callback(self, msg):
        # Update with new target
        joint_cmds = self.controller.update(0.01, msg.pose)

        # Publish
        joint_msg = JointState()
        joint_msg.position = joint_cmds
        self.cmd_pub.publish(joint_msg)
```

---

## 7. Hands-On Lab: Walk-and-Grasp (4 hours)

**Goal**: Humanoid walks to table and grasps object.

**Steps**:
1. Implement whole-body IK solver
2. Create walking controller (from Chapter 17)
3. Add arm reaching controller
4. Combine with task prioritization
5. Test in Isaac Sim: walk 2m, grasp cup

**Validation**: Successfully grasp object while maintaining balance

---

## 8. End-of-Chapter Project

Implement mobile manipulation task (navigate + manipulate).

**Requirements**:
- Walk to target location (3 meters)
- Avoid obstacles during walking
- Reach and grasp object at destination
- Carry object while walking back
- Success rate >50% over 10 trials

**Deliverables**:
- Whole-body controller implementation
- Simulation video
- Performance analysis

---

## Summary

Whole-body control enables simultaneous locomotion and manipulation through hierarchical task prioritization and null-space projection. This unlocks mobile manipulation capabilities for humanoid robots.

**Next**: Chapter 19 covers human-robot interaction and teleoperation.
