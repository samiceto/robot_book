# Chapter 12: Containerization & CI/CD

## Learning Objectives

1. **Containerize** ROS 2 applications with Docker
2. **Build** multi-arch images (x86 + ARM64) for Jetson
3. **Deploy** with Docker Compose for multi-node systems
4. **Automate** testing with GitHub Actions
5. **Implement** CI/CD pipeline for continuous deployment

---

## 1. Why Containerization?

**Problems Without Containers**:
- "Works on my machine" syndrome
- Dependency conflicts (ROS 2 Iron vs Jazzy)
- Manual deployment steps
- Difficult to reproduce environments

**Docker Benefits**:
- **Reproducibility**: Identical environment everywhere
- **Isolation**: Each service has own dependencies
- **Portability**: Same container on dev PC and Jetson
- **Scalability**: Easy to orchestrate multiple nodes

---

## 2. Docker Basics

### Install Docker

```bash
# Install on Ubuntu
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group (no sudo needed)
sudo usermod -aG docker $USER
newgrp docker

# Verify
docker run hello-world
```

### Basic Dockerfile (ROS 2)

```dockerfile
FROM ros:iron-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-iron-realsense2-camera \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install ultralytics

# Copy ROS 2 workspace
COPY ./ros2_ws /ros2_ws

# Build workspace
RUN . /opt/ros/iron/setup.sh && \
    cd /ros2_ws && \
    colcon build --symlink-install

# Source workspace on container start
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set entrypoint
CMD ["bash"]
```

### Build and Run

```bash
# Build image
docker build -t humanoid_perception:latest .

# Run container
docker run -it --rm \
  --network host \
  --device /dev/video0 \
  humanoid_perception:latest

# Inside container
ros2 launch my_package perception.launch.py
```

---

## 3. Multi-Arch Images (x86 + ARM64)

**Goal**: Single image runs on dev PC (x86) and Jetson (ARM64).

### Use Docker Buildx

```bash
# Enable buildx
docker buildx create --name multiarch --use

# Build for both architectures
docker buildx build --platform linux/amd64,linux/arm64 \
  -t myuser/humanoid_perception:latest \
  --push \
  .

# Result: Image works on x86 PC and ARM64 Jetson
```

---

## 4. Docker Compose (Multi-Node)

Deploy entire ROS 2 system with one command:

**docker-compose.yml**:
```yaml
version: '3.8'

services:
  realsense:
    image: myuser/realsense:latest
    devices:
      - /dev/bus/usb:/dev/bus/usb  # USB camera access
    network_mode: host
    command: ros2 launch realsense2_camera rs_launch.py

  perception:
    image: myuser/perception:latest
    network_mode: host
    depends_on:
      - realsense
    command: ros2 run my_package perception_node

  navigation:
    image: myuser/navigation:latest
    network_mode: host
    depends_on:
      - perception
    command: ros2 launch nav2_bringup navigation_launch.py

  rviz:
    image: myuser/rviz:latest
    network_mode: host
    environment:
      - DISPLAY=$DISPLAY
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    command: rviz2
```

**Deploy**:
```bash
# Start all services
docker-compose up -d

# View logs
docker-compose logs -f perception

# Stop all
docker-compose down
```

---

## 5. CI/CD with GitHub Actions

Automate build, test, and deployment on every git push.

**.github/workflows/ci.yml**:
```yaml
name: ROS 2 CI

on: [push, pull_request]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04

    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: iron

      - name: Build workspace
        run: |
          source /opt/ros/iron/setup.bash
          colcon build --symlink-install

      - name: Run tests
        run: |
          source /opt/ros/iron/setup.bash
          source install/setup.bash
          colcon test
          colcon test-result --verbose

      - name: Build Docker image
        run: |
          docker build -t humanoid_perception:${{ github.sha }} .

      - name: Push to Docker Hub
        if: github.ref == 'refs/heads/main'
        run: |
          echo "${{ secrets.DOCKER_PASSWORD }}" | docker login -u "${{ secrets.DOCKER_USERNAME }}" --password-stdin
          docker tag humanoid_perception:${{ github.sha }} myuser/humanoid_perception:latest
          docker push myuser/humanoid_perception:latest
```

**Workflow**:
1. Developer pushes code to GitHub
2. GitHub Actions runs automatically
3. Builds ROS 2 workspace
4. Runs all tests
5. Builds Docker image
6. Pushes to Docker Hub (if main branch)
7. Jetson pulls latest image and redeploys

---

## 6. Deployment to Jetson

### One-Command Deployment

```bash
# On Jetson
# Pull latest image
docker pull myuser/humanoid_perception:latest

# Run with GPU support
docker run -it --rm \
  --runtime nvidia \
  --network host \
  --device /dev/video0 \
  myuser/humanoid_perception:latest \
  ros2 run my_package perception_node
```

### Auto-Update on Git Push

**On Jetson** (install watchtower):
```bash
docker run -d \
  --name watchtower \
  -v /var/run/docker.sock:/var/run/docker.sock \
  containrrr/watchtower \
  --interval 300  # Check every 5 minutes

# Watchtower automatically pulls and restarts updated containers
```

---

## 7. Hands-On Lab: Full CI/CD Pipeline (4 hours)

**Goal**: Implement complete CI/CD from git push to Jetson deployment.

**Steps**:
1. Dockerize perception package
2. Setup GitHub Actions workflow
3. Configure Docker Hub repository
4. Deploy to Jetson with docker-compose
5. Test auto-update with code change

**Validation**:
- [ ] Push to GitHub triggers build
- [ ] Tests pass in CI
- [ ] Docker image pushed to registry
- [ ] Jetson auto-pulls and restarts
- [ ] Perception runs correctly after update

---

## 8. End-of-Chapter Project: Production Deployment

Deploy full humanoid perception system with CI/CD.

**Requirements**:
- Multi-service architecture (RealSense + YOLO + Nav2)
- Docker Compose orchestration
- GitHub Actions CI/CD
- Automated testing (unit + integration)
- One-command deployment to Jetson
- Auto-update on main branch merge

**Deliverables**:
- Dockerfile for each service
- docker-compose.yml
- GitHub Actions workflow
- Deployment documentation
- Demo video (git push → auto-deploy → robot operating)

**Grading**:
- **Architecture** (30%): Clean service separation
- **CI/CD** (30%): Automated build/test/deploy
- **Testing** (20%): Comprehensive test coverage
- **Documentation** (20%): Clear deployment guide

---

## Summary

You learned to:
- Containerize ROS 2 applications with Docker
- Build multi-arch images for x86 and ARM64
- Orchestrate multi-node systems with Docker Compose
- Automate testing with GitHub Actions
- Implement CI/CD for continuous deployment

**Part 3 Complete!** You can now build production-ready perception systems with automated deployment to edge devices.

---

## Further Reading

1. **Docker Documentation**: https://docs.docker.com/
2. **ROS 2 Docker Best Practices**: https://docs.ros.org/en/iron/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html
3. **GitHub Actions for ROS**: https://github.com/ros-tooling/action-ros-ci
4. **NVIDIA Container Toolkit**: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/

---

**End of Chapter 12 - Part 3 Complete**

You have completed **Part 3: Perception & Edge Deployment**. You can integrate RealSense cameras, run YOLO object detection, deploy to Jetson Orin with quantization, and automate deployment with CI/CD.

**Progress**: 12 of 21 chapters complete (57%)

**Next**: Part 4 (Vision-Language-Action Models) covers VLA architecture, OpenVLA fine-tuning, multimodal reasoning, and end-to-end visuomotor control.
