# Chapter 21: Production Deployment at Scale

## Learning Objectives

1. **Design** fleet management systems for multiple robots
2. **Implement** remote monitoring and telemetry
3. **Deploy** OTA (over-the-air) updates
4. **Optimize** for 24/7 operation and maintenance
5. **Scale** from prototype to production fleet

---

## 1. Fleet Architecture

### System Components
```
Cloud Backend (AWS/Azure)
    ↓
Fleet Manager (ROS 2 + Dashboard)
    ↓
Individual Robots (Jetson Orin)
    ↓
Local Operations (Tasks, Telemetry)
```

### Communication
- **Cloud ↔ Robot**: MQTT (lightweight, reliable)
- **Robot ↔ Robot**: ROS 2 DDS (local coordination)
- **Human ↔ Fleet**: Web dashboard (React)

---

## 2. Fleet Management

### Robot Registry
```python
from dataclasses import dataclass
import redis

@dataclass
class RobotStatus:
    robot_id: str
    status: str  # "idle", "busy", "charging", "error"
    battery: float  # 0-100%
    location: tuple  # (x, y, z)
    current_task: str
    last_heartbeat: float

class FleetManager:
    def __init__(self):
        self.redis = redis.Redis(host='fleet-manager', port=6379)

    def register_robot(self, robot_id, capabilities):
        """Register new robot in fleet."""
        self.redis.hset(f"robot:{robot_id}", mapping={
            "capabilities": json.dumps(capabilities),
            "status": "idle",
            "registered_at": time.time()
        })

    def assign_task(self, task):
        """Assign task to available robot."""
        # Find idle robot with required capabilities
        for robot_id in self.get_idle_robots():
            if self.has_capability(robot_id, task.required_capability):
                self.redis.hset(f"robot:{robot_id}", "status", "busy")
                self.redis.hset(f"robot:{robot_id}", "task", task.id)
                return robot_id

        return None  # No available robot
```

---

## 3. Remote Monitoring

### Telemetry Pipeline
```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
import paho.mqtt.client as mqtt

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry')

        # MQTT client to cloud
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("fleet.example.com", 1883)

        # Subscribe to diagnostics
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )

        # Periodic status
        self.create_timer(5.0, self.send_status)

    def diagnostics_callback(self, msg):
        """Forward diagnostics to cloud."""
        payload = {
            "robot_id": self.get_namespace(),
            "timestamp": time.time(),
            "diagnostics": [
                {"name": d.name, "level": d.level, "message": d.message}
                for d in msg.status
            ]
        }
        self.mqtt_client.publish("fleet/diagnostics", json.dumps(payload))

    def send_status(self):
        """Send periodic heartbeat."""
        status = {
            "robot_id": self.get_namespace(),
            "battery": self.get_battery_level(),
            "cpu": psutil.cpu_percent(),
            "memory": psutil.virtual_memory().percent,
            "uptime": time.time() - self.start_time
        }
        self.mqtt_client.publish("fleet/status", json.dumps(status))
```

---

## 4. OTA Updates

### Deployment Strategy
```python
class OTAUpdater:
    def __init__(self):
        self.current_version = "v1.2.3"

    def check_for_updates(self):
        """Check cloud for new software version."""
        response = requests.get("https://fleet.example.com/api/latest_version")
        latest_version = response.json()["version"]

        if latest_version > self.current_version:
            return latest_version
        return None

    def download_update(self, version):
        """Download new Docker image."""
        subprocess.run([
            "docker", "pull", f"mycompany/humanoid-robot:{version}"
        ])

    def apply_update(self, version):
        """Apply update with rollback capability."""
        # Backup current state
        self.backup_state()

        # Stop current services
        subprocess.run(["docker-compose", "down"])

        # Update docker-compose to new version
        self.update_compose_file(version)

        # Start new version
        result = subprocess.run(["docker-compose", "up", "-d"])

        if result.returncode != 0:
            # Rollback on failure
            self.rollback()
            return False

        return True
```

---

## 5. Maintenance Scheduling

### Predictive Maintenance
```python
def predict_maintenance(robot_history):
    """
    Predict when maintenance is needed.

    Inputs:
        - Battery cycles
        - Motor operating hours
        - Error frequency
    """
    battery_cycles = robot_history['battery_cycles']
    motor_hours = robot_history['motor_hours']
    errors_last_week = robot_history['recent_errors']

    # Simple heuristic (replace with ML model)
    maintenance_score = (
        battery_cycles / 500 +  # Battery: 500 cycles
        motor_hours / 1000 +    # Motors: 1000 hours
        errors_last_week / 10   # Errors threshold
    )

    if maintenance_score > 1.0:
        return "URGENT"
    elif maintenance_score > 0.7:
        return "SOON"
    else:
        return "OK"
```

---

## 6. Scaling Checklist

### Prototype → Production

**Infrastructure**:
- [ ] Cloud backend (AWS/Azure/GCP)
- [ ] MQTT broker (Mosquitto/AWS IoT)
- [ ] Database (PostgreSQL/MongoDB)
- [ ] Monitoring (Grafana/Prometheus)

**Software**:
- [ ] Containerized all services (Docker)
- [ ] CI/CD pipeline (GitHub Actions)
- [ ] Automated testing (unit + integration)
- [ ] OTA update mechanism

**Hardware**:
- [ ] Redundant sensors (lidar, camera)
- [ ] Backup battery (swap capability)
- [ ] Robust networking (4G/5G fallback)
- [ ] Thermal management

**Operations**:
- [ ] On-call rotation (24/7 support)
- [ ] Incident response plan
- [ ] Maintenance schedule
- [ ] User training materials

---

## 7. Hands-On Lab: Fleet Dashboard (4 hours)

**Goal**: Build web dashboard for fleet monitoring.

**Steps**:
1. Setup MQTT broker
2. Implement telemetry forwarding (robots → cloud)
3. Build React dashboard showing:
   - Robot locations (map)
   - Battery levels
   - Task status
   - Alerts
4. Test with 3 simulated robots

**Validation**: Dashboard updates in real-time (<1s latency)

---

## 8. End-of-Chapter Project: Production Fleet

Deploy 5-robot fleet for warehouse automation.

**Requirements**:
- Fleet management system (task assignment, monitoring)
- Remote telemetry dashboard
- OTA update mechanism
- Maintenance prediction
- Operate for 24 hours with <10% downtime

**Deliverables**:
- Fleet management code
- Web dashboard
- Deployment documentation
- 24-hour operation report with metrics

---

## Summary

Production deployment requires robust fleet management, remote monitoring, OTA updates, and predictive maintenance. Scaling from 1 to 100+ robots demands careful architecture and operational planning.

---

## 🎉 BOOK COMPLETE!

**Congratulations!** You've completed all 21 chapters covering:

- **Part 1**: ROS 2 foundations and robot description
- **Part 2**: Simulation with Gazebo and Isaac Sim
- **Part 3**: Perception and edge deployment
- **Part 4**: Vision-Language-Action models
- **Part 5**: Advanced topics (locomotion, HRI, safety, production)

**You Can Now**:
- Build humanoid robots from scratch
- Simulate in Isaac Sim with synthetic data
- Deploy VLA models on Jetson Orin
- Implement whole-body control
- Launch production robot fleets

**Next Steps**:
1. Build your own humanoid project
2. Contribute to open-source robotics (OpenVLA, Isaac ROS)
3. Join robotics competitions
4. Start a robotics company

**Thank you for learning Physical AI & Humanoid Robotics!**
