# Chapter 20: Safety and Compliance

## Learning Objectives

1. **Understand** ISO 13482 safety standards for service robots
2. **Implement** collision detection and avoidance
3. **Design** fail-safe behaviors and emergency stops
4. **Conduct** risk assessments for robot operations
5. **Document** safety compliance for certification

---

## 1. Safety Standards Overview

### ISO 13482 (Personal Care Robots)
**Key Requirements**:
- Risk assessment and mitigation
- Emergency stop accessible within 1 second
- Collision force limits (&lt;150N for head, &lt;50N for body)
- Pinch point protection
- Electrical safety (low voltage)

### Safety Levels
```
SIL 0: No safety requirements
SIL 1: Low risk (e.g., vacuum robot)
SIL 2: Medium risk (e.g., delivery robot)
SIL 3: High risk (e.g., humanoid near humans)
```

**Humanoid robots typically require SIL 2-3 certification.**

---

## 2. Collision Detection

### Force/Torque Monitoring
```python
class CollisionDetector:
    def __init__(self, joint_names):
        self.expected_torques = {}
        self.collision_threshold = 10.0  # Nm

    def detect_collision(self, measured_torques):
        """Detect unexpected torques indicating collision."""
        for joint, torque in measured_torques.items():
            expected = self.expected_torques.get(joint, 0)
            residual = abs(torque - expected)

            if residual > self.collision_threshold:
                return True, joint

        return False, None

# Usage
collision, joint = detector.detect_collision(robot.get_torques())
if collision:
    robot.emergency_stop()
    print(f"Collision detected at {joint}")
```

### Skin Sensors (Capacitive/Resistive)
```python
def check_skin_sensors(sensor_readings):
    """Monitor contact across robot surface."""
    for sensor_id, value in sensor_readings.items():
        if value > CONTACT_THRESHOLD:
            return True, sensor_id
    return False, None
```

---

## 3. Collision Avoidance

### Distance-Based Slowdown
```python
def compute_safe_velocity(obstacle_distance):
    """Reduce velocity based on proximity."""
    if obstacle_distance < 0.3:  # Critical zone
        return 0.0  # Stop
    elif obstacle_distance < 1.0:  # Warning zone
        return 0.3 * MAX_VELOCITY
    else:  # Safe zone
        return MAX_VELOCITY

velocity = compute_safe_velocity(distance_to_human)
robot.set_velocity(velocity)
```

### Dynamic Obstacle Avoidance
```python
from nav2_msgs.action import NavigateToPose

class SafeNavigator:
    def __init__(self):
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/costmap', self.costmap_callback, 10
        )

    def costmap_callback(self, msg):
        """Update obstacle map dynamically."""
        # Replan if new obstacles detected
        if self.path_blocked(msg):
            self.replan_path()
```

---

## 4. Emergency Stop System

### Hardware E-Stop
```python
class EmergencyStop:
    def __init__(self):
        # Hardware button on GPIO
        self.button_pin = 17
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.button_pin, GPIO.FALLING, callback=self.estop_pressed)

    def estop_pressed(self, channel):
        """Immediate hardware stop."""
        robot.disable_all_motors()
        robot.apply_brakes()
        print("EMERGENCY STOP ACTIVATED")

        # Cannot resume without physical reset
```

### Software E-Stop
```python
def software_estop():
    """Controlled stop with deceleration."""
    # Decelerate over 0.5 seconds
    for t in np.linspace(0, 0.5, 50):
        scale = 1 - (t / 0.5)
        robot.set_velocity_scale(scale)
        time.sleep(0.01)

    robot.set_velocity(0)
```

---

## 5. Risk Assessment

### HAZOP Analysis (Hazard and Operability)

| Hazard | Cause | Effect | Mitigation | Severity |
|--------|-------|--------|------------|----------|
| Fall | Slippery floor | Robot damage, human injury | Non-slip feet, balance sensors | High |
| Collision | Sensor failure | Human injury | Redundant sensors, E-stop | Critical |
| Pinch | Gripper malfunction | Finger injury | Force limits, soft gripper | Medium |
| Fire | Battery overcharge | Facility damage | BMS, temp monitoring | High |

### Risk Calculation
```python
def calculate_risk_score(probability, severity):
    """
    Risk = Probability × Severity

    Probability: 1-5 (rare to certain)
    Severity: 1-5 (negligible to catastrophic)
    """
    risk = probability * severity

    if risk >= 15:
        return "CRITICAL - Must mitigate"
    elif risk >= 10:
        return "HIGH - Implement safeguards"
    elif risk >= 5:
        return "MEDIUM - Monitor closely"
    else:
        return "LOW - Accept risk"
```

---

## 6. Fail-Safe Behaviors

### Graceful Degradation
```python
class FaultTolerantController:
    def __init__(self):
        self.sensors = ['camera', 'lidar', 'imu']
        self.sensor_status = {s: True for s in self.sensors}

    def update(self):
        """Adapt behavior based on sensor availability."""
        if not self.sensor_status['camera']:
            # Fall back to lidar-only navigation
            self.use_lidar_navigation()

        if not self.sensor_status['imu']:
            # Use joint encoders for balance
            self.use_encoder_balance()

        # If critical sensor fails, stop
        if not self.sensor_status['lidar']:
            self.safe_stop()
```

---

## 7. Hands-On Lab: Safety System Implementation (3 hours)

**Goal**: Implement multi-layered safety system.

**Steps**:
1. Add collision detection (torque-based)
2. Implement E-stop (software + hardware simulation)
3. Add proximity-based slowdown
4. Test safety responses in Isaac Sim
5. Document hazard analysis

**Validation**:
- E-stop triggers within 50ms
- Collision detected before 5N force
- Robot stops within 0.5m of obstacles

---

## 8. End-of-Chapter Project

Conduct full safety assessment for humanoid deployment.

**Requirements**:
- HAZOP analysis (10+ hazards)
- Risk mitigation strategies
- Safety system implementation (collision, E-stop, limits)
- Test report (100 safety-critical scenarios)
- Documentation for ISO 13482 compliance

**Deliverables**:
- Risk assessment report (PDF)
- Safety system code
- Test results with videos
- Compliance checklist

---

## Summary

Safety is paramount for real-world robot deployment. ISO 13482 compliance requires collision detection, emergency stops, risk assessments, and fail-safe behaviors.

**Next**: Chapter 21 covers production deployment at scale with fleet management.
