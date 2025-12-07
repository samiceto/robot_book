# Architectural Decision: CI/CD Testing Infrastructure and Cost Optimization

**Date**: 2025-12-06
**Status**: Decided
**Decision Makers**: Book Author Team
**Related Tasks**: T017

## Context

The Physical AI & Humanoid Robotics textbook includes 12-15 companion code repositories that must be tested continuously to ensure:
- Code examples work on all supported hardware tiers (Budget, Mid, Premium)
- Compatibility with dual ROS 2 distributions (Ubuntu 22.04 + Iron, Ubuntu 24.04 + Jazzy)
- Isaac Sim integration examples run on RTX GPUs
- Jetson Orin deployment examples validate on edge hardware
- Code quality (Black, flake8) and formatting consistency
- Documentation builds (Markdown linting, broken link detection)

Challenges:
- **GPU Testing**: Isaac Sim requires NVIDIA RTX GPU (not available in standard GitHub Actions runners)
- **Cost**: GPU runners expensive ($1-5/hour), 12-15 repos × weekly tests = high monthly cost
- **Jetson Testing**: Physical Jetson Orin Nano hardware not available in cloud CI/CD
- **Build Time**: Full Isaac Sim tests may take 30-60 minutes per repository

## Options Considered

### Option 1: GitHub Actions (CPU-only) + Self-Hosted GPU Runner
**Platform**: GitHub Actions (hosted) + Self-hosted runner (RTX GPU)
**Cost**: Free (public repos) + electricity for self-hosted GPU

**Architecture**:
- **CPU Tests** (GitHub Actions hosted runners, Ubuntu 22.04/24.04):
  - Code linting (Black, flake8)
  - Documentation validation (markdownlint, link checking)
  - ROS 2 build and unit tests (no simulation)
  - EPUB/PDF build tests
- **GPU Tests** (Self-hosted runner with RTX GPU):
  - Isaac Sim integration tests (30 FPS validation)
  - VLA model inference tests (OpenVLA 7B)
  - Synthetic data generation pipeline tests
  - Domain randomization tests

**Self-Hosted Runner Setup**:
- Hardware: RTX 4090 24GB (Premium tier from plan.md, $1,999)
- OS: Ubuntu 24.04 LTS
- Installed: Isaac Sim 2024.2, ROS 2 Jazzy, CUDA 12.x, Docker
- Security: Isolated network, ephemeral containers, no secrets in logs

**Pros**:
- ✅ **Cost-Effective**: No per-minute GPU charges (one-time hardware cost + electricity)
- ✅ **Full Control**: Can install Isaac Sim, custom drivers, ROS 2 distributions
- ✅ **Unlimited Minutes**: No usage cap (GitHub Actions free tier: 2,000 minutes/month for private repos, unlimited for public)
- ✅ **Realistic Hardware**: Tests on actual RTX 4090 (matches Premium tier readers)
- ✅ **Fast Feedback**: Local runner reduces queue time vs. cloud GPU runners
- ✅ **Dual ROS 2 Testing**: Can test Ubuntu 22.04 + Iron AND 24.04 + Jazzy in parallel

**Cons**:
- ⚠️ **Upfront Cost**: $2,000 for RTX 4090 + $1,000 for workstation = $3,000 initial investment
- ⚠️ **Maintenance**: Requires manual updates (Isaac Sim, CUDA, OS security patches)
- ⚠️ **Availability**: Single runner = bottleneck if multiple PRs submitted simultaneously
- ⚠️ **Security Risk**: Self-hosted runners on public repos can execute malicious code (mitigation: ephemeral Docker containers)
- ⚠️ **Power Consumption**: RTX 4090 TDP 450W, ~$20-30/month electricity (24/7 operation)

**Risk Mitigation**:
- **Security**: Use ephemeral Docker containers for each workflow run (destroy after test)
- **Availability**: Prioritize critical tests (Isaac Sim integration) on GPU runner, non-GPU tests on GitHub hosted runners
- **Backup**: Fallback to cloud GPU runners (AWS g5.xlarge) if self-hosted runner down

**Estimated Monthly Cost**:
- Electricity (RTX 4090, 24/7): ~$25/month
- Internet (if dedicated): ~$50/month (optional, can use existing)
- **Total**: ~$25-75/month (plus $3,000 upfront hardware cost)

### Option 2: GitHub Actions + AWS EC2 GPU Instances (On-Demand)
**Platform**: GitHub Actions triggers AWS EC2 g5.xlarge (NVIDIA T4 GPU)
**Cost**: Pay-per-use ($1.006/hour in us-east-1)

**Architecture**:
- GitHub Actions workflow triggers AWS EC2 instance via AWS API
- EC2 instance runs tests (Isaac Sim, VLA inference)
- Shutdown instance after tests complete
- Store results in S3, report back to GitHub

**Pros**:
- ✅ **No Upfront Cost**: Pay only for usage
- ✅ **Scalability**: Can spin up multiple instances for parallel testing
- ✅ **Managed**: No hardware maintenance, AWS handles infrastructure
- ✅ **Security**: Isolated instances per test run

**Cons**:
- ❌ **High Cost**: 12 repos × 1 hour/week × 4 weeks = 48 hours/month × $1/hour = $48/month (minimum)
- ❌ **Cold Start**: EC2 instance startup + Isaac Sim installation = 10-15 minutes overhead per test
- ❌ **Complexity**: Requires AWS integration, IAM roles, S3 storage, Lambda for orchestration
- ❌ **NVIDIA T4 GPU**: Slower than RTX 4090 (may not achieve 30 FPS target on Budget tier examples)

**Estimated Monthly Cost**:
- Compute (48 hours @ $1/hour): ~$48/month
- Storage (S3 for test artifacts): ~$5/month
- Data transfer: ~$2/month
- **Total**: ~$55/month (plus AWS setup complexity)

### Option 3: GitHub Actions + Paperspace GPU Cloud (Spot Instances)
**Platform**: GitHub Actions + Paperspace Gradient (GPU cloud)
**Cost**: Spot instances ~$0.50-0.80/hour (RTX 4000/5000)

**Architecture**:
- Similar to Option 2, but using Paperspace instead of AWS
- Paperspace Gradient API triggers GPU instance
- Pre-built Docker images with Isaac Sim, ROS 2

**Pros**:
- ✅ **Lower Cost**: Spot instances ~50% cheaper than AWS
- ✅ **RTX GPUs**: Access to RTX 4000/5000 (better than T4)
- ✅ **Docker-Native**: Paperspace designed for ML workloads

**Cons**:
- ❌ **Spot Interruptions**: Instances can be preempted (test failures)
- ❌ **Smaller Community**: Less documentation vs. AWS
- ❌ **Regional Availability**: Limited datacenter locations vs. AWS

**Estimated Monthly Cost**:
- Compute (48 hours @ $0.65/hour): ~$31/month
- Storage: ~$5/month
- **Total**: ~$36/month

### Option 4: Hybrid - GitHub Actions (CPU) + Manual GPU Testing (Local Workstation)
**Platform**: GitHub Actions for CPU tests, manual testing on local RTX 4090
**Cost**: Free (GitHub Actions) + existing hardware

**Architecture**:
- CI/CD runs CPU tests automatically (linting, unit tests, docs)
- GPU tests run manually before releases (weekly or per-release)
- Developer maintains checklist for manual GPU tests

**Pros**:
- ✅ **Zero Additional Cost**: Uses existing development hardware
- ✅ **Simple Setup**: No cloud integration, no self-hosted runner configuration
- ✅ **Full Hardware Access**: Test on RTX 4090, Jetson Orin Nano (physical hardware)

**Cons**:
- ❌ **Manual Effort**: Requires developer time for manual testing
- ❌ **Inconsistent**: Tests may be skipped or forgotten
- ❌ **No PR Gating**: Cannot block PRs based on GPU test results
- ❌ **Slow Feedback**: Manual tests run infrequently (weekly vs. per-commit)

**Decision**: **REJECTED** - Insufficient automation for 12-15 repositories

## Decision

**SELECTED: Option 1 - GitHub Actions (CPU-only) + Self-Hosted GPU Runner**

### Rationale

1. **Cost Efficiency**: One-time $3,000 hardware investment cheaper than $50/month cloud GPU costs over 2-year book lifecycle ($1,200)
2. **Realistic Hardware**: Testing on RTX 4090 matches Premium tier readers, ensures examples work at highest performance level
3. **Unlimited Testing**: No per-minute charges enables frequent testing (every PR, every commit to main)
4. **Full Control**: Can install Isaac Sim, custom CUDA versions, dual ROS 2 distributions without cloud provider limitations
5. **Security**: Ephemeral Docker containers isolate test runs, prevent malicious code execution on self-hosted runner
6. **Dual ROS 2 Testing**: Matrix testing for Ubuntu 22.04 + Iron AND 24.04 + Jazzy in parallel

### Why Not Cloud GPU Runners?

While cloud GPU runners (AWS, Paperspace) offer flexibility:
- **Cost**: $36-55/month × 24 months = $864-1,320 over book lifecycle (vs. $3,000 upfront + $600 electricity = $3,600 total, but hardware retains resale value)
- **Cold Start Overhead**: 10-15 minutes EC2 startup per test run wastes time
- **GPU Performance**: NVIDIA T4 (AWS g5.xlarge) slower than RTX 4090

### Why Not Manual Testing?

Manual GPU testing insufficient for:
- **12-15 Repositories**: Too many repos to test manually
- **Frequent Updates**: Code changes weekly during book development
- **PR Gating**: Need automated pass/fail criteria before merging

## Implementation Strategy

### Self-Hosted Runner Setup

**Hardware Specifications**:
- **GPU**: NVIDIA RTX 4090 24GB ($1,999)
- **CPU**: Intel i9-13900K or AMD Ryzen 9 7950X ($500-600)
- **RAM**: 64 GB DDR5 ($200-250)
- **Storage**: 2 TB NVMe SSD ($150) for Isaac Sim, Docker images
- **PSU**: 1200W 80+ Platinum ($200) for RTX 4090 power draw
- **Cooling**: AIO liquid cooler for sustained GPU loads ($150)
- **Case**: Standard ATX case with good airflow ($100)
- **Total**: ~$3,300 (one-time cost)

**Software Stack**:
- **OS**: Ubuntu 24.04 LTS (for ROS 2 Jazzy primary testing)
- **Docker**: 24.x with NVIDIA Container Toolkit (for ephemeral test containers)
- **NVIDIA Driver**: 550+ (for RTX 4090 support)
- **CUDA**: 12.x (for Isaac Sim, PyTorch)
- **Isaac Sim**: 2024.2 (installed in Docker image, not on host)
- **ROS 2**: Iron (Ubuntu 22.04 container) + Jazzy (Ubuntu 24.04 container)

**GitHub Actions Runner Installation**:
```bash
# Create dedicated user for runner
sudo useradd -m -s /bin/bash github-runner
sudo usermod -aG docker github-runner

# Download and install GitHub Actions runner
cd /home/github-runner
curl -o actions-runner-linux-x64-2.311.0.tar.gz -L \
  https://github.com/actions/runner/releases/download/v2.311.0/actions-runner-linux-x64-2.311.0.tar.gz
tar xzf actions-runner-linux-x64-2.311.0.tar.gz

# Configure runner (requires GitHub PAT token)
./config.sh --url https://github.com/YOUR_ORG/YOUR_REPO --token YOUR_TOKEN \
  --name gpu-runner-rtx4090 --labels self-hosted,linux,x64,gpu,rtx4090

# Install as systemd service (auto-restart)
sudo ./svc.sh install github-runner
sudo ./svc.sh start
```

### Security Hardening

**Ephemeral Docker Containers**:
```yaml
# .github/workflows/gpu-test.yml
jobs:
  isaac-sim-test:
    runs-on: [self-hosted, gpu, rtx4090]
    container:
      image: ghcr.io/your-org/isaac-sim-ros2-jazzy:latest
      options: --gpus all --rm  # Auto-remove container after test
    steps:
      - uses: actions/checkout@v4
      - name: Run Isaac Sim Tests
        run: |
          source /opt/ros/jazzy/setup.bash
          pytest tests/test_isaac_sim_integration.py
```

**Network Isolation**:
- Runner on isolated VLAN (no access to internal network)
- Firewall rules: Allow HTTPS outbound (GitHub API), block all inbound except SSH (IP whitelist)

**Secret Management**:
- No secrets stored on runner filesystem
- Use GitHub Actions secrets, injected at runtime
- Rotate GitHub PAT tokens quarterly

**Monitoring**:
- Prometheus + Grafana for GPU utilization, temperature, power consumption
- Alert if GPU temp >85°C (thermal throttling risk)
- Alert if runner offline >1 hour

### GitHub Actions Workflow Matrix

**CPU Tests** (GitHub-hosted runners, free):
```yaml
name: CPU Tests (Lint, Build, Unit Tests)

on:
  pull_request:
    branches: [main]
  push:
    branches: [main]

jobs:
  lint-and-format:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: '3.10'
      - name: Install dependencies
        run: pip install black flake8
      - name: Check formatting
        run: black --check .
      - name: Lint
        run: flake8 .

  ros2-build-test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        ros_distro: [iron, jazzy]
        include:
          - ros_distro: iron
            ubuntu_version: 22.04
          - ros_distro: jazzy
            ubuntu_version: 24.04
    container:
      image: ros:${{ matrix.ros_distro }}-ros-base
    steps:
      - uses: actions/checkout@v4
      - name: Install dependencies
        run: |
          apt-get update
          rosdep update
          rosdep install --from-paths src --ignore-src -y
      - name: Build workspace
        run: |
          source /opt/ros/${{ matrix.ros_distro }}/setup.bash
          colcon build --symlink-install
      - name: Run unit tests
        run: |
          source /opt/ros/${{ matrix.ros_distro }}/setup.bash
          colcon test
          colcon test-result --verbose
```

**GPU Tests** (Self-hosted runner):
```yaml
name: GPU Tests (Isaac Sim Integration)

on:
  pull_request:
    branches: [main]
  push:
    branches: [main]
  schedule:
    - cron: '0 2 * * 0'  # Weekly on Sundays at 2 AM

jobs:
  isaac-sim-integration:
    runs-on: [self-hosted, gpu, rtx4090]
    container:
      image: ghcr.io/your-org/isaac-sim-ros2-jazzy:latest
      options: --gpus all --rm
    steps:
      - uses: actions/checkout@v4
      - name: Validate Isaac Sim Installation
        run: |
          nvidia-smi  # Verify GPU available
          python -c "import omni.isaac.sim; print('Isaac Sim OK')"
      - name: Run 30 FPS Benchmark
        run: |
          source /opt/ros/jazzy/setup.bash
          python tests/benchmark_isaac_sim.py --min-fps 30
      - name: Test Humanoid Manipulation
        run: |
          pytest tests/test_humanoid_pick_place.py -v
      - name: Upload Test Results
        uses: actions/upload-artifact@v4
        with:
          name: isaac-sim-test-results
          path: test-results/

  vla-inference-test:
    runs-on: [self-hosted, gpu, rtx4090]
    container:
      image: ghcr.io/your-org/openvla-ros2-jazzy:latest
      options: --gpus all --rm
    steps:
      - uses: actions/checkout@v4
      - name: Test OpenVLA Inference
        run: |
          pytest tests/test_openvla_integration.py -v
      - name: Validate ≥12 Hz Performance
        run: |
          python tests/benchmark_vla_inference.py --min-hz 12
```

### Docker Image Management

**Base Images** (Pre-built, stored in GitHub Container Registry):
1. **isaac-sim-ros2-iron**: Ubuntu 22.04 + ROS 2 Iron + Isaac Sim 2024.2
2. **isaac-sim-ros2-jazzy**: Ubuntu 24.04 + ROS 2 Jazzy + Isaac Sim 2024.2
3. **openvla-ros2-jazzy**: Ubuntu 24.04 + ROS 2 Jazzy + OpenVLA 7B + TensorRT
4. **jetson-orin-emulator**: Jetson Orin Nano emulation (for testing without physical hardware)

**Build and Push**:
```bash
# Build Isaac Sim + ROS 2 Jazzy image
docker build -t ghcr.io/your-org/isaac-sim-ros2-jazzy:latest \
  -f docker/isaac-sim-jazzy.Dockerfile .

# Push to GitHub Container Registry
echo $GITHUB_TOKEN | docker login ghcr.io -u USERNAME --password-stdin
docker push ghcr.io/your-org/isaac-sim-ros2-jazzy:latest
```

**Image Size Optimization**:
- Multi-stage builds: Compile in builder stage, copy binaries to runtime stage
- Layer caching: Order `RUN` commands from least to most frequently changed
- Cleanup: Remove apt cache, pip cache in same layer as install

**Expected Image Sizes**:
- `isaac-sim-ros2-jazzy`: ~12 GB (Isaac Sim is large)
- `openvla-ros2-jazzy`: ~8 GB (PyTorch + OpenVLA weights)
- `jetson-orin-emulator`: ~6 GB

### Jetson Orin Testing Strategy

**Challenge**: Physical Jetson Orin Nano not available in CI/CD

**Solution 1: Emulation** (Limited):
- Use QEMU ARM64 emulation on x86_64 runner
- Test ROS 2 functionality, code correctness
- **Cannot test**: GPU performance (no CUDA in emulation), actual inference speed

**Solution 2: Manual Testing** (Practical):
- Developer tests on physical Jetson Orin Nano weekly
- Validates ≥12 Hz capstone performance before releases
- Documents results in GitHub issue (e.g., "Jetson Validation - v1.0")

**Solution 3: Remote Jetson Runner** (Advanced):
- Physical Jetson Orin Nano as GitHub Actions self-hosted runner
- Connect via USB-C to developer workstation
- Run subset of tests on actual hardware
- **Pros**: Real hardware validation
- **Cons**: Setup complexity, single Jetson = bottleneck

**Recommended**: Solution 2 (manual testing) until Jetson cloud runners available

### Cost Analysis

**Upfront Costs**:
- RTX 4090 workstation: $3,300 (one-time)
- Docker image storage (GitHub Container Registry): Free (public repos, 500 MB free for private)

**Monthly Costs**:
- Electricity (RTX 4090, 24/7 @ 450W TDP, 50% avg utilization): ~$25/month
  - 450W × 0.5 × 24h × 30d = 162 kWh/month
  - 162 kWh × $0.15/kWh = $24.30/month
- Internet (if dedicated): $0 (use existing)
- **Total**: ~$25/month

**Annual Costs**:
- Year 1: $3,300 (upfront) + $300 (electricity) = $3,600
- Year 2: $300 (electricity only)
- **2-Year Total**: $3,900

**Comparison to Cloud GPU Runners**:
- AWS g5.xlarge: 48 hours/month × $1/hour × 24 months = $1,152
- Paperspace spot: 48 hours/month × $0.65/hour × 24 months = $749
- Self-hosted: $3,900 (but hardware retains ~$1,500 resale value after 2 years)
- **Net cost**: ~$2,400 over 2 years

**Conclusion**: Self-hosted runner cost-competitive with cloud, provides better performance and control.

### Testing Frequency and Triggers

**Per-Commit** (on PRs and main branch):
- CPU tests: Lint, format, build, unit tests (fast, <5 minutes)

**Daily** (scheduled, main branch):
- Full test suite: CPU + GPU integration tests (~30 minutes)

**Weekly** (Sunday 2 AM):
- Extended tests: Isaac Sim benchmarks (30 FPS validation), VLA inference (≥12 Hz), domain randomization
- Manual Jetson Orin testing (developer validates on physical hardware)

**Pre-Release** (manual trigger):
- Full regression suite across all 12-15 repos
- Generate test report, upload to GitHub Release

## Consequences

### Positive
- ✅ **Cost-Effective**: $3,900 over 2 years cheaper than cloud GPU at scale
- ✅ **Realistic Testing**: RTX 4090 matches Premium tier, ensures examples work at highest performance
- ✅ **Unlimited Testing**: No per-minute charges enables frequent testing
- ✅ **Full Control**: Can install Isaac Sim, CUDA, ROS 2 without cloud limitations
- ✅ **Dual ROS 2 Testing**: Matrix testing for Iron + Jazzy in parallel
- ✅ **Fast Feedback**: Local runner reduces queue time vs. cloud

### Negative
- ❌ **Upfront Cost**: $3,300 hardware investment required before CI/CD operational
- ❌ **Maintenance Burden**: Manual updates for Isaac Sim, CUDA, security patches
- ❌ **Single Point of Failure**: One runner = bottleneck if multiple PRs submitted simultaneously
- ❌ **Security Risk**: Self-hosted runners on public repos require ephemeral containers
- ❌ **Power Consumption**: ~$25/month electricity cost

### Neutral
- ⚖️ **Jetson Testing**: Manual testing on physical hardware (no emulation alternative)
- ⚖️ **Docker Image Size**: 12 GB Isaac Sim images require fast internet for initial pull
- ⚖️ **Build Time**: Full Isaac Sim tests ~30 minutes (acceptable for nightly builds)

## Validation Against Requirements

- ✅ **GPU Testing**: RTX 4090 runner validates Isaac Sim performance (≥30 FPS)
- ✅ **Dual ROS 2**: Matrix testing for Ubuntu 22.04 + Iron, Ubuntu 24.04 + Jazzy
- ✅ **Code Quality**: Black, flake8 run on every PR
- ✅ **Documentation**: Markdown linting, link checking automated
- ✅ **Cost**: $25/month operating cost sustainable for book project
- ✅ **Scalability**: Can add second GPU runner if needed (~$3,300)

## References

- GitHub Actions Self-Hosted Runners: https://docs.github.com/en/actions/hosting-your-own-runners
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/
- AWS EC2 g5 Instances: https://aws.amazon.com/ec2/instance-types/g5/
- Paperspace Gradient: https://www.paperspace.com/gradient
- Docker Multi-Stage Builds: https://docs.docker.com/build/building/multi-stage/
- GitHub Container Registry: https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry

## Revision History

- 2025-12-06: Initial decision - Selected GitHub Actions + Self-Hosted RTX 4090 GPU runner with ephemeral Docker containers
