# Chapter 2: Development Environment Setup - ROS 2, Isaac Sim, and Hardware

**Learning Objectives**

By the end of this chapter, you will be able to:

1. Verify hardware requirements and select an appropriate development configuration (local workstation vs. cloud)
2. Install Ubuntu 22.04 LTS with proper partitioning for robotics development
3. Install and configure NVIDIA drivers (550+) and CUDA Toolkit 12.x
4. Install ROS 2 Iron Irwini and configure the development environment
5. Install NVIDIA Isaac Sim 2024.2 and verify GPU-accelerated rendering
6. Set up essential development tools (VS Code, Git, Python environments)
7. Complete a hands-on lab demonstrating ROS 2 + Isaac Sim integration
8. Troubleshoot common installation issues (driver conflicts, ROS 2 sourcing, Isaac Sim crashes)

**Prerequisites**: Basic familiarity with Linux command line, ability to follow terminal commands, access to hardware meeting minimum specifications (RTX 4070 Ti or equivalent)

**Estimated Time**: 8-12 hours (including downloads and troubleshooting)

---

## 1. System Requirements Verification

### 1.1 Hardware Checklist (GPU, CPU, RAM, Storage)

Before beginning installation, verify your system meets the minimum requirements. Physical AI development is computationally intensive—attempting to run Isaac Sim on underpowered hardware will result in frustration.

**GPU Requirements (CRITICAL)**

Isaac Sim requires an NVIDIA RTX GPU for GPU-accelerated PhysX simulation and RTX raytracing. AMD and Intel GPUs are **not supported**.

**Minimum (Budget Tier - $700)**:
- NVIDIA RTX 4070 Ti (12 GB VRAM)
- Performance: Isaac Sim at 30-40 FPS (1080p resolution)
- Limitations: Cannot run multiple simultaneous Isaac Sim instances; VLA model training requires INT8 quantization

**Recommended (Mid-Range - $1,100)**:
- NVIDIA RTX 4080 (16 GB VRAM)
- Performance: Isaac Sim at 60-80 FPS; train OpenVLA in 6-8 hours (500 demonstrations)
- Sweet spot for serious learners and researchers

**Premium (Research Labs - $1,800)**:
- NVIDIA RTX 4090 (24 GB VRAM)
- Performance: Isaac Sim at 120+ FPS; multi-GPU training; run multiple Isaac Sim instances in parallel

**Verification Command**:
```bash
# Check if NVIDIA GPU is detected (run on existing Linux system or LiveUSB)
lspci | grep -i nvidia

# Expected output (example):
# 01:00.0 VGA compatible controller: NVIDIA Corporation AD104 [GeForce RTX 4070 Ti]
```

If you see no output, your system either has no NVIDIA GPU or the GPU is disabled in BIOS. Check BIOS settings and ensure the discrete GPU is enabled (not integrated graphics).

**CPU Requirements**

**Minimum**: Intel Core i5-12400 (6 cores) or AMD Ryzen 5 5600 (6 cores)
**Recommended**: Intel Core i7-13700 (8 cores) or AMD Ryzen 7 7700X (8 cores)
**Premium**: Intel Core i9-13900K (16 cores) or AMD Ryzen 9 7950X (16 cores)

**Why CPU Matters**: ROS 2 nodes run on CPU; multi-node systems (Chapter 3) benefit from higher core counts. Isaac Sim's USD scene loading and script execution are CPU-bound.

**Verification Command**:
```bash
lscpu | grep "Model name"
# Example output: Model name: AMD Ryzen 7 7700X 8-Core Processor
```

**RAM Requirements**

**Minimum**: 32 GB DDR4
**Recommended**: 64 GB DDR5
**Premium**: 128 GB DDR5

**Why RAM Matters**: Isaac Sim loads large USD scenes (5-10 GB), ROS 2 bag files can exceed 20 GB, and PyTorch training requires batches in GPU+CPU RAM. Running out of RAM causes system freezes.

**Verification Command**:
```bash
free -h
# Check "total" row under Mem - should show ≥32 GB
```

**Storage Requirements**

**Minimum**: 1 TB NVMe SSD
**Recommended**: 2 TB NVMe SSD

**Space Breakdown**:
- Ubuntu 22.04: ~25 GB
- NVIDIA drivers + CUDA: ~10 GB
- ROS 2 Iron (full desktop): ~8 GB
- Isaac Sim 2024.2: ~50 GB
- PyTorch + dependencies: ~15 GB
- Datasets (Open X-Embodiment): ~200 GB
- ROS 2 bag files: ~100 GB
- Workspace builds: ~50 GB
- **Total**: ~460 GB (budget 500-600 GB with headroom)

**Verification Command**:
```bash
lsblk
# Check for NVMe drives (nvme0n1, nvme1n1) and total size
df -h /
# Check available space on root partition
```

**Performance Note**: Use NVMe SSD, not SATA SSD or HDD. Isaac Sim scene loading is I/O-bound; NVMe provides 3-5× faster load times (3,500 MB/s vs. 550 MB/s for SATA).

### 1.2 Operating System: Ubuntu 22.04 vs. 24.04

**Recommended: Ubuntu 22.04.4 LTS (Jammy Jellyfish)**

**Why Ubuntu 22.04**:
- ROS 2 Iron (May 2023 release) officially supports Ubuntu 22.04
- Isaac Sim 2024.2 tested on Ubuntu 22.04 (Ubuntu 24.04 support experimental as of mid-2024)
- NVIDIA driver 550+ stable on Ubuntu 22.04 (fewer kernel compatibility issues)
- LTS (Long Term Support) until April 2027—5 years of security updates

**Ubuntu 24.04 LTS (Noble Numbat) - Alternative**:
- Supports ROS 2 Jazzy Jalisco (May 2024 release, LTS until May 2029)
- Newer kernel (6.8 vs. 5.15) with better hardware support for Intel 14th-gen CPUs and latest AMD GPUs
- **Tradeoff**: Some third-party ROS 2 packages lag Ubuntu 24.04 support; Isaac Sim may require manual CUDA path fixes

**Recommendation**: Use Ubuntu 22.04 LTS for maximum compatibility. If you have very new hardware (Intel 14th-gen, AMD Ryzen 9000 series), consider Ubuntu 24.04 + ROS 2 Jazzy.

**Download Link**: https://releases.ubuntu.com/22.04.4/

**SHA256 Verification** (security best practice):
```bash
sha256sum ubuntu-22.04.4-desktop-amd64.iso
# Compare output to official hash on ubuntu.com/download
```

### 1.3 Cloud Alternatives (AWS, GCP, Paperspace) for Non-NVIDIA Systems

If you lack an RTX GPU or prefer not to invest in hardware upfront, cloud GPU instances are a viable alternative.

**AWS EC2 g5.xlarge (NVIDIA A10G, 24 GB VRAM)**
- **Specs**: 4 vCPUs, 16 GB RAM, A10G GPU (10,752 CUDA cores)
- **Cost**: $1.006/hour on-demand ($0.301/hour spot) → ~$240/month (240 hours of use)
- **Performance**: Isaac Sim at 60-80 FPS; suitable for training and development
- **Setup**: Launch Ubuntu 22.04 AMI, install NVIDIA drivers via `ubuntu-drivers autoinstall`

**GCP Compute Engine n1-standard-4 + NVIDIA T4 (16 GB VRAM)**
- **Specs**: 4 vCPUs, 15 GB RAM, T4 GPU (2,560 CUDA cores)
- **Cost**: $0.95/hour → $230/month
- **Performance**: Isaac Sim at 30-40 FPS (similar to RTX 4070 Ti)
- **Setup**: Use Deep Learning VM image (pre-configured with CUDA, cuDNN)

**Paperspace Gradient (RTX 4000 Ada, 20 GB VRAM)**
- **Specs**: 8 vCPUs, 30 GB RAM, RTX 4000 Ada
- **Cost**: $0.76/hour → $180/month
- **Performance**: Isaac Sim at 50-60 FPS
- **Advantage**: Pre-configured Jupyter notebooks, easier for beginners

**Lambda Labs (RTX 4090, 24 GB VRAM)**
- **Specs**: 30 vCPUs, 200 GB RAM, RTX 4090
- **Cost**: $1.10/hour → $260/month
- **Performance**: Isaac Sim at 120+ FPS; premium option for research
- **Advantage**: Same consumer GPU as local workstation (easier debugging)

**Cloud Workflow Considerations**:
- **Data Transfer**: Upload/download ROS 2 bag files (10-50 GB) is time-consuming; budget for egress costs ($0.09/GB on AWS)
- **Persistence**: Use EBS volumes (AWS) or persistent disks (GCP) to save workspaces; charged when instance is stopped (~$0.10/GB-month)
- **Jupyter Access**: Isaac Sim GUI requires X11 forwarding or VNC; Paperspace provides web-based desktop (easier setup)

**Hybrid Strategy** (recommended for students):
- Development and testing: Cloud GPU ($200/month)
- Final experiments and paper results: Rent premium GPU (RTX 4090) for 40-80 hours ($88-$176)

---

## 2. Installing Ubuntu 22.04 LTS

### 2.1 Dual Boot vs. Dedicated Machine

**Option 1: Dedicated Machine (Recommended)**
- Entire SSD dedicated to Ubuntu
- No Windows conflicts (driver issues, boot errors)
- Maximum performance (no partition overhead)
- **Best for**: Serious learners with a spare machine or building a new workstation

**Option 2: Dual Boot (Ubuntu + Windows)**
- Share SSD between Ubuntu and Windows
- Requires careful partitioning (mistakes can erase Windows)
- GRUB bootloader manages OS selection
- **Best for**: Users who need Windows for other tasks (gaming, Adobe software)

**Option 3: Dedicated Ubuntu Drive + Separate Windows Drive**
- Install Ubuntu on one physical drive (e.g., NVMe #1)
- Keep Windows on separate drive (e.g., NVMe #2)
- Use BIOS boot menu (F11/F12) to select OS
- **Best practice**: Physically disconnect Windows drive during Ubuntu installation to avoid accidental overwrites

**Common Mistake to Avoid**: Do NOT install Ubuntu on an external USB drive for daily use (slow I/O causes ROS 2 bag recording to drop messages). External drives are fine for LiveUSB installers, not for the operating system.

### 2.2 Partition Layout for ROS 2 Development

**Simple Layout (Recommended for Beginners)**:
- **Swap**: 16 GB (for 32 GB RAM) or 32 GB (for 64+ GB RAM)
- **Root (`/`)**: Remaining space (e.g., 950 GB on a 1 TB drive)
- **Boot (`/boot/efi`)**: 512 MB (auto-created by installer for UEFI systems)

**Advanced Layout (Multi-Drive Systems)**:
- **Drive 1 (NVMe 500 GB)**: Root `/` (OS, ROS 2, Isaac Sim)
- **Drive 2 (NVMe 2 TB)**: `/home` (datasets, ROS 2 bags, workspaces)
- **Rationale**: Separating OS from data prevents OS reinstalls from wiping datasets

**File System**: Use ext4 (default). Btrfs and ZFS offer advanced features (snapshots, compression) but add complexity.

**Installation Steps (Dedicated Machine)**:
1. Create bootable USB with Ubuntu 22.04 ISO using Rufus (Windows) or Balena Etcher (macOS/Linux)
2. Boot from USB (press F12/F11 during startup, select USB drive)
3. Select "Install Ubuntu" (not "Try Ubuntu")
4. Choose "Erase disk and install Ubuntu" (wipes entire drive—ensure backups!)
5. Click "Install Now" → Confirm partitioning → Select timezone and create user account
6. Wait 15-20 minutes for installation
7. Reboot and remove USB drive

**Post-Install Verification**:
```bash
# Check Ubuntu version
lsb_release -a
# Expected: Ubuntu 22.04.4 LTS

# Check kernel version
uname -r
# Expected: 5.15.0-xxx-generic or higher
```

### 2.3 Post-Install Configuration (SSH, Firewall, Updates)

**System Updates (CRITICAL - run immediately)**:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install build-essential curl wget git vim -y
```

**Why Immediate Updates Matter**: NVIDIA driver installation fails if kernel headers are mismatched. Running `apt upgrade` ensures the latest kernel headers are installed.

**Enable SSH Server** (for remote development):
```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh

# Check SSH is running
sudo systemctl status ssh

# Find your IP address
ip addr show
# Look for inet 192.168.x.x (local network) or 10.x.x.x
```

**Configure Firewall (UFW)**:
```bash
sudo ufw allow ssh
sudo ufw enable
sudo ufw status
# Expected: Status: active, with "22/tcp ALLOW"
```

**Set Hostname** (for easier identification on network):
```bash
sudo hostnamectl set-hostname robotics-workstation
# Verify
hostnamectl
```

**Install Additional Utilities**:
```bash
# htop: better than top for monitoring CPU/GPU
sudo apt install htop -y

# net-tools: includes ifconfig (deprecated but still useful)
sudo apt install net-tools -y

# screen/tmux: terminal multiplexers for long-running processes
sudo apt install tmux -y
```

---

## 3. NVIDIA Driver and CUDA Installation

### 3.1 Verifying GPU Detection (`lspci`, `nvidia-smi`)

**Step 1: Confirm GPU is Detected by Kernel**:
```bash
lspci | grep -i nvidia
# Expected output (example for RTX 4080):
# 01:00.0 VGA compatible controller: NVIDIA Corporation AD103 [GeForce RTX 4080]
# 01:00.1 Audio device: NVIDIA Corporation Device 22bc
```

If you see no output, the GPU may be:
- Not properly seated in PCIe slot (reseat the GPU)
- Disabled in BIOS (enable "Discrete Graphics" in BIOS settings)
- Using integrated graphics instead (set BIOS to "Discrete Only" or "Hybrid Mode")

**Step 2: Check for Nouveau Driver** (open-source NVIDIA driver—must be disabled):
```bash
lsmod | grep nouveau
# If you see "nouveau", it must be blacklisted (step 3.4)
```

### 3.2 Installing NVIDIA Driver 550+

**Recommended Method: ubuntu-drivers Tool (Easiest)**:
```bash
# Detect recommended driver
ubuntu-drivers devices
# Example output:
# vendor   : NVIDIA Corporation
# model    : AD103 [GeForce RTX 4080]
# driver   : nvidia-driver-550 - third-party non-free recommended
# driver   : nvidia-driver-545 - third-party non-free
# driver   : xserver-xorg-video-nouveau - distro free builtin

# Install recommended driver (550 in this example)
sudo ubuntu-drivers autoinstall

# Alternative: specify exact driver version
sudo apt install nvidia-driver-550 -y
```

**Reboot Required**:
```bash
sudo reboot
```

**Post-Reboot Verification**:
```bash
nvidia-smi
# Expected output:
# +-----------------------------------------------------------------------------------------+
# | NVIDIA-SMI 550.54.15              Driver Version: 550.54.15      CUDA Version: 12.4     |
# |---------|-------------------------|-------------------------|-------------------------|
# |   0   GeForce RTX 4080   On      | 00000000:01:00.0  Off |                  N/A |
# | 30C    N/A        P8     15W /  320W |   432MiB / 16376MiB |      0%      Default |
# +-----------------------------------------------------------------------------------------+
```

**Troubleshooting: "nvidia-smi: command not found"**:
```bash
# Check if driver is loaded
lsmod | grep nvidia
# If empty, driver installation failed

# Check dmesg for errors
sudo dmesg | grep -i nvidia

# Common issue: Secure Boot enabled (NVIDIA driver is unsigned)
# Solution: Disable Secure Boot in BIOS or enroll MOK (Machine Owner Key)
```

### 3.3 Installing CUDA Toolkit 12.x

Isaac Sim 2024.2 requires CUDA 12.0 or later. PyTorch 2.0+ supports CUDA 12.1.

**Official NVIDIA Installation (Recommended)**:
```bash
# Download CUDA 12.2 runfile installer (2.5 GB)
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda_12.2.0_535.54.03_linux.run

# Run installer
sudo sh cuda_12.2.0_535.54.03_linux.run

# Installer prompts:
# - Accept EULA
# - Deselect "Driver" (already installed via ubuntu-drivers)
# - Select "CUDA Toolkit 12.2"
# - Install to default location: /usr/local/cuda-12.2
```

**Add CUDA to PATH** (add to `~/.bashrc`):
```bash
echo 'export PATH=/usr/local/cuda-12.2/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

**Verification**:
```bash
nvcc --version
# Expected output:
# nvcc: NVIDIA (R) Cuda compiler driver
# Cuda compilation tools, release 12.2, V12.2.140
```

**Install cuDNN (Deep Neural Network library)**:
```bash
# Download cuDNN 8.9 for CUDA 12.x from NVIDIA (requires free account):
# https://developer.nvidia.com/cudnn

# Extract tarball
tar -xvf cudnn-linux-x86_64-8.9.5.30_cuda12-archive.tar.xz

# Copy files to CUDA directory
sudo cp cudnn-*-archive/include/cudnn*.h /usr/local/cuda-12.2/include
sudo cp cudnn-*-archive/lib/libcudnn* /usr/local/cuda-12.2/lib64
sudo chmod a+r /usr/local/cuda-12.2/include/cudnn*.h /usr/local/cuda-12.2/lib64/libcudnn*
```

### 3.4 Troubleshooting: Nouveau Blacklisting, Secure Boot

**Blacklist Nouveau Driver** (if `lsmod | grep nouveau` shows output):

Create blacklist file:
```bash
sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nvidia-nouveau.conf"

# Update initramfs
sudo update-initramfs -u

# Reboot
sudo reboot
```

**Disable Secure Boot** (if driver refuses to load):

Secure Boot prevents unsigned kernel modules (NVIDIA driver) from loading. Two options:

**Option 1: Disable Secure Boot in BIOS** (easier):
1. Reboot into BIOS (press F2/Del during startup)
2. Navigate to "Security" → "Secure Boot"
3. Set to "Disabled"
4. Save and reboot

**Option 2: Enroll MOK (Machine Owner Key)** (keeps Secure Boot enabled):
```bash
# Generate MOK
sudo mokutil --import /var/lib/shim-signed/mok/MOK.der

# Reboot - MOK Manager will appear
# Select "Enroll MOK" → Enter password from previous step → Reboot
```

**Verification After Blacklist/Secure Boot Fix**:
```bash
# Nouveau should be absent
lsmod | grep nouveau
# Expected: (no output)

# NVIDIA driver should be loaded
lsmod | grep nvidia
# Expected: nvidia, nvidia_drm, nvidia_modeset
```

---

## 4. ROS 2 Iron Installation

### 4.1 Adding ROS 2 APT Repository

**Set Locale** (required for ROS 2):
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale
```

**Add ROS 2 GPG Key**:
```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

**Add ROS 2 Repository**:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 4.2 Installing `ros-iron-desktop-full`

**Update Package List**:
```bash
sudo apt update
```

**Install ROS 2 Iron Desktop Full** (~8 GB download):
```bash
sudo apt install ros-iron-desktop-full -y
```

**What `desktop-full` Includes**:
- **ros-iron-desktop**: ROS 2 core, RViz2, demos, tutorials
- **Simulation tools**: Gazebo Ignition (now Gazebo Harmonic)
- **Navigation**: Nav2 stack
- **Perception**: image_pipeline, vision_opencv

**Install Development Tools**:
```bash
sudo apt install ros-dev-tools -y
# Includes: colcon (build tool), rosdep (dependency management), python3-vcstool
```

### 4.3 Configuring ROS 2 Environment (bashrc Setup)

**Add ROS 2 Sourcing to `.bashrc`**:
```bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Why Sourcing Matters**: ROS 2 commands (`ros2`, `colcon`, `rviz2`) are only available after sourcing. Without sourcing, you'll get "command not found" errors.

**Alternative: Manual Sourcing** (for advanced users with multiple ROS 2 versions):
```bash
# Don't add to .bashrc; source manually when needed
source /opt/ros/iron/setup.bash
```

**Configure ROS Domain ID** (prevents cross-talk between projects):
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

**What is ROS_DOMAIN_ID?**: ROS 2 uses DDS (Data Distribution Service) for communication. Nodes with different domain IDs cannot see each other's topics. Useful in labs with multiple students.

### 4.4 Verifying Installation with `ros2 topic list`

**Test ROS 2 Installation**:
```bash
# Start ROS 2 daemon (first-time setup)
ros2 daemon start

# List topics (should show /rosout and /parameter_events)
ros2 topic list
# Expected output:
# /parameter_events
# /rosout

# Check ROS 2 version
ros2 --version
# Expected: ros2 cli version: 0.25.4
```

**Run Talker-Listener Demo** (classic ROS 2 test):
```bash
# Terminal 1: Start talker (publisher)
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener (subscriber)
ros2 run demo_nodes_cpp listener

# Expected output in Terminal 2:
# [INFO] [listener]: I heard: 'Hello World: 1'
# [INFO] [listener]: I heard: 'Hello World: 2'
```

**Troubleshooting: "Package 'demo_nodes_cpp' not found"**:
```bash
# Ensure desktop-full was installed
dpkg -l | grep ros-iron-desktop

# Reinstall if missing
sudo apt install ros-iron-desktop-full -y
```

---

## 5. Isaac Sim 2024.2 Installation

### 5.1 Installing Omniverse Launcher

**Download Omniverse Launcher** (~500 MB):
```bash
# Download from NVIDIA (requires free account)
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

**First-Time Setup**:
1. Sign in with NVIDIA account (or create free account)
2. Accept EULA
3. Launcher installs to `~/.local/share/ov/pkg/launcher/`

**Add Launcher to PATH** (optional, for `omniverse-launcher` command):
```bash
sudo ln -s ~/omniverse-launcher-linux.AppImage /usr/local/bin/omniverse-launcher
```

### 5.2 Downloading Isaac Sim 2024.2 (~50 GB)

**Within Omniverse Launcher**:
1. Click "Exchange" tab (left sidebar)
2. Search for "Isaac Sim"
3. Click "Install" on "Isaac Sim 2024.2.0" (~50 GB download, 1-3 hours on fast connection)
4. Installation location: `~/.local/share/ov/pkg/isaac_sim-2024.2.0/`

**Verify Installation**:
```bash
ls ~/.local/share/ov/pkg/isaac_sim-2024.2.0/
# Expected: isaac-sim.sh, python.sh, kit/, exts/, ...
```

**Add Isaac Sim to PATH**:
```bash
echo 'export ISAACSIM_PATH=~/.local/share/ov/pkg/isaac_sim-2024.2.0' >> ~/.bashrc
echo 'export PATH=$ISAACSIM_PATH:$PATH' >> ~/.bashrc
source ~/.bashrc
```

### 5.3 Launching Isaac Sim and Verifying GPU Rendering

**Launch Isaac Sim** (first launch takes 2-3 minutes to compile shaders):
```bash
cd ~/.local/share/ov/pkg/isaac_sim-2024.2.0
./isaac-sim.sh
```

**First-Launch Shader Compilation**:
- Progress bar shows "Compiling shaders..."
- RTX 4070 Ti: ~2 minutes
- RTX 4090: ~1 minute
- This only happens on first launch or after driver updates

**Verify RTX Rendering**:
1. Isaac Sim window opens with default scene
2. Top menu: "Window" → "Render Settings" → "Real-Time Rendering"
3. Confirm "RTX - Interactive (Path Tracing)" is selected
4. Load demo scene: "Isaac Examples" → "Simple Objects" → "Falling Cubes"
5. Press "Play" (triangle button in left toolbar)
6. Cubes should fall with realistic physics

**Check FPS** (frames per second):
- Bottom-right corner shows "FPS: XX"
- Expected FPS on RTX 4070 Ti: 30-40 FPS
- Expected FPS on RTX 4080: 60-80 FPS
- Expected FPS on RTX 4090: 120+ FPS

**If FPS < 20**:
- Check "Window" → "Render Settings" → Resolution (lower to 1280×720)
- Disable "Real-Time Raytracing" (use rasterization instead)
- Check `nvidia-smi` for GPU utilization (should be 70-95% during play)

### 5.4 Testing Isaac ROS Bridge

**Install Isaac Sim ROS 2 Bridge Extension**:

Within Isaac Sim:
1. Top menu: "Window" → "Extensions"
2. Search for "ROS2 Bridge"
3. Click "Install" on "omni.isaac.ros2_bridge"
4. Wait for installation (1-2 minutes)

**Test ROS 2 Topic Publishing**:

1. In Isaac Sim: Load "Isaac Examples" → "ROS" → "Navigation" → "Carter v2 Navigation"
2. Press "Play"
3. In a terminal:
```bash
source /opt/ros/iron/setup.bash
ros2 topic list
# Expected topics from Isaac Sim:
# /carter/camera_left/rgb
# /carter/camera_right/rgb
# /carter/cmd_vel
# /tf
# /tf_static

# Echo a topic to verify data
ros2 topic echo /carter/camera_left/rgb --no-arr
# Expected: Image messages with header timestamps
```

**Verify Topic Rate** (should be ~30-60 Hz):
```bash
ros2 topic hz /carter/camera_left/rgb
# Expected: average rate: 30.1 (for 30 FPS simulation)
```

**Troubleshooting: No ROS 2 Topics Visible**:
```bash
# Check ROS 2 domain ID matches
echo $ROS_DOMAIN_ID
# Should match domain set in Isaac Sim (default: 0)

# If using non-zero domain, set in Isaac Sim:
# Window → Extensions → ROS2 Bridge → Settings → Domain ID
```

---

## 6. Development Tools

### 6.1 VS Code with ROS 2 Extensions

**Install VS Code**:
```bash
# Download .deb package
wget -O code.deb 'https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64'
sudo dpkg -i code.deb
sudo apt install -f -y  # Fix dependencies

# Launch VS Code
code
```

**Essential Extensions**:
1. **Python** (Microsoft): IntelliSense, debugging
2. **C/C++** (Microsoft): For C++ ROS 2 nodes
3. **ROS** (Microsoft): Syntax highlighting for .launch, .urdf, .xacro files
4. **CMake** (twxs): CMakeLists.txt support
5. **GitLens** (GitKraken): Advanced Git integration

**Install Extensions via Command Line**:
```bash
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension ms-iot.vscode-ros
code --install-extension twxs.cmake
code --install-extension eamodio.gitlens
```

**Configure Python Interpreter** (for ROS 2 Python packages):
1. Open VS Code in workspace: `code ~/ros2_ws`
2. Press `Ctrl+Shift+P` → "Python: Select Interpreter"
3. Choose `/usr/bin/python3` (system Python with ROS 2 packages)

### 6.2 Python Environment Management (venv, pyenv)

**Option 1: venv (Built-in, Recommended for ROS 2)**:
```bash
# Create virtual environment
python3 -m venv ~/robotics_env

# Activate
source ~/robotics_env/bin/activate

# Install PyTorch 2.0+ with CUDA 12.1
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install robotics packages
pip install numpy scipy matplotlib opencv-python transformers
```

**Option 2: pyenv (For Managing Multiple Python Versions)**:
```bash
# Install pyenv
curl https://pyenv.run | bash

# Add to .bashrc
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init --path)"' >> ~/.bashrc
source ~/.bashrc

# Install Python 3.10 (ROS 2 Iron uses Python 3.10)
pyenv install 3.10.12
pyenv global 3.10.12
```

**Warning**: Do NOT install ROS 2 packages (`rclpy`, `sensor_msgs`) via pip in virtual environments. ROS 2 packages must use system Python (`/usr/bin/python3`). Only install non-ROS packages (PyTorch, Hugging Face) in venv.

### 6.3 Git and GitHub Configuration

**Install Git** (already installed in step 2.3, but verify):
```bash
git --version
# Expected: git version 2.34.1 or higher
```

**Configure Git Identity**:
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Verify
git config --list
```

**Generate SSH Key for GitHub**:
```bash
ssh-keygen -t ed25519 -C "your.email@example.com"
# Press Enter to accept default location (~/.ssh/id_ed25519)
# Enter passphrase (optional but recommended)

# Copy public key to clipboard
cat ~/.ssh/id_ed25519.pub
# Paste into GitHub: Settings → SSH and GPG keys → New SSH key
```

**Test GitHub Connection**:
```bash
ssh -T git@github.com
# Expected: Hi <username>! You've successfully authenticated...
```

### 6.4 Docker for Reproducible Environments

**Install Docker**:
```bash
# Add Docker GPG key
sudo apt install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add repository
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y

# Add user to docker group (avoid sudo for docker commands)
sudo usermod -aG docker $USER
newgrp docker

# Verify
docker run hello-world
```

**NVIDIA Container Toolkit** (for GPU access in Docker containers):
```bash
# Add NVIDIA Container Toolkit repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install
sudo apt update
sudo apt install nvidia-container-toolkit -y

# Configure Docker to use NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Test GPU access in Docker
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
# Should show nvidia-smi output inside container
```

---

## 7. Hands-On Lab: First ROS 2 + Isaac Sim Demo

**Objective**: Load a humanoid URDF in Isaac Sim, start the ROS 2 bridge, and visualize joint states in RViz2.

**Time Estimate**: 4 hours (including troubleshooting)

**Prerequisites**: Completed sections 1-6 (Ubuntu, NVIDIA drivers, ROS 2, Isaac Sim installed)

### Step 1: Create ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace (initially empty)
colcon build
source install/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 2: Download Simple Humanoid URDF

We'll use a companion repository with a pre-built 12-DOF humanoid URDF.

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/robot-book-code.git
cd robot-book-code/chapter-02-quickstart/

# Verify URDF exists
ls urdf/simple_humanoid.urdf
```

**URDF Structure** (simple_humanoid.urdf):
- **12 DOF**: 2 hip pitch, 2 hip roll, 2 knee, 2 ankle, 2 shoulder, 2 elbow
- **Links**: base_link, torso, left_thigh, right_thigh, left_shin, right_shin, left_foot, right_foot, left_upper_arm, right_upper_arm, left_forearm, right_forearm
- **Mass**: Total 50 kg (realistic for lightweight humanoid)

### Step 3: Load URDF in Isaac Sim

**Launch Isaac Sim**:
```bash
cd ~/.local/share/ov/pkg/isaac_sim-2024.2.0
./isaac-sim.sh
```

**Import URDF**:
1. Top menu: "Isaac Utils" → "URDF Importer"
2. Click "Browse" → Navigate to `~/ros2_ws/src/robot-book-code/chapter-02-quickstart/urdf/simple_humanoid.urdf`
3. Settings:
   - Import Inertia Tensor: ✅ Checked
   - Fix Base Link: ❌ Unchecked (humanoid should not be fixed)
   - Joint Drive Strength: 1000 (for position control)
   - Joint Drive Damping: 100
4. Click "Import"

**Expected Result**: Humanoid appears in viewport standing upright (may take 10-20 seconds to import).

### Step 4: Start ROS 2 Bridge

**Enable ROS 2 Bridge Extension** (if not already enabled):
1. "Window" → "Extensions" → Search "ROS2 Bridge" → Ensure "Enabled" is checked

**Add Joint State Publisher**:
1. Select humanoid in "Stage" panel (left sidebar)
2. Right-click → "Create" → "Isaac" → "Sensors" → "Joint State Sensor"
3. In "Property" panel (right sidebar):
   - Topic Name: `/joint_states`
   - Publish Rate: 60 Hz

**Press Play** (triangle button in toolbar).

### Step 5: Verify `/joint_states` Topic in Terminal

Open new terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep joint_states
# Expected: /joint_states

# Check topic rate
ros2 topic hz /joint_states
# Expected: average rate: 60.0

# Echo topic (verify joint names and positions)
ros2 topic echo /joint_states --once
```

**Expected Output**:
```
header:
  stamp:
    sec: 10
    nanosec: 500000000
  frame_id: 'base_link'
name:
- left_hip_pitch
- left_hip_roll
- left_knee
- left_ankle
- right_hip_pitch
- right_hip_roll
- right_knee
- right_ankle
- left_shoulder
- left_elbow
- right_shoulder
- right_elbow
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### Step 6: Visualize in RViz2

**Install RViz2** (if not already installed with desktop-full):
```bash
sudo apt install ros-iron-rviz2 -y
```

**Launch RViz2**:
```bash
rviz2
```

**Add Robot Model**:
1. Click "Add" (bottom-left panel)
2. Select "RobotModel"
3. In "RobotModel" settings:
   - Robot Description: `robot_description` (leave blank if not using robot_state_publisher)
   - TF Prefix: (leave blank)

**If Robot Model Doesn't Appear** (common issue):

We need to publish the URDF to `/robot_description` topic:
```bash
# Install robot_state_publisher and joint_state_publisher
sudo apt install ros-iron-robot-state-publisher ros-iron-joint-state-publisher -y

# Publish URDF
cd ~/ros2_ws/src/robot-book-code/chapter-02-quickstart/urdf/
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_humanoid.urdf)"
```

**In RViz2**: Robot model should now appear. You should see the humanoid standing with joints at zero positions.

### Step 7: Validate Performance Metrics

**Isaac Sim FPS**:
- Check bottom-right corner of Isaac Sim window
- **Target**: ≥30 FPS on RTX 4070 Ti, ≥60 FPS on RTX 4080, ≥120 FPS on RTX 4090

**ROS 2 Topic Rate**:
```bash
ros2 topic hz /joint_states
# Expected: 60 Hz (matches publish rate set in step 4)
```

**GPU Utilization**:
```bash
nvidia-smi
# Check "GPU-Util" column - should be 60-95% during simulation
# If <40%, GPU is underutilized (check if CPU-bound)
```

**Troubleshooting: FPS < 30 on RTX 4070 Ti**:
- Reduce Isaac Sim resolution: Window → Render Settings → Resolution → 1280×720
- Disable RTX Real-Time Raytracing: Render Settings → Renderer → "Rasterize" instead of "RTX"
- Close background applications (web browsers, Slack)

---

## 8. End-of-Chapter Project: Multi-Environment Setup Validation

**Objective**: Set up the development environment on TWO different hardware configurations and benchmark performance.

**Configuration Options**:
1. **Local Workstation** (RTX 4070 Ti / 4080 / 4090)
2. **Cloud GPU** (AWS g5.xlarge, GCP n1 + T4, Paperspace RTX 4000)

**Deliverables**:
1. **Setup Documentation**: Step-by-step log of installation process (what worked, what failed)
2. **Benchmark Results**: Table comparing Isaac Sim FPS, ROS 2 topic rates, GPU utilization
3. **Troubleshooting Log**: List of errors encountered and solutions

**Benchmark Script** (create `benchmark.sh`):
```bash
#!/bin/bash
# benchmark.sh - Automated performance testing

echo "=== System Info ==="
lsb_release -a
uname -r
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv

echo "=== Isaac Sim FPS Test ==="
echo "Manually run Isaac Sim with Falling Cubes demo for 60 seconds"
echo "Record average FPS from bottom-right corner"

echo "=== ROS 2 Topic Rate Test ==="
# Start Isaac Sim with joint_states publisher, then:
timeout 30 ros2 topic hz /joint_states

echo "=== GPU Utilization Test ==="
nvidia-smi dmon -s u -c 10
# Sample GPU utilization every 1 second for 10 seconds
```

**Expected Benchmark Results** (example table):

| Metric | RTX 4070 Ti (Local) | AWS g5.xlarge (A10G) | Paperspace RTX 4000 |
|--------|---------------------|----------------------|---------------------|
| Isaac Sim FPS (1080p) | 35 FPS | 72 FPS | 58 FPS |
| Isaac Sim FPS (720p) | 58 FPS | 105 FPS | 88 FPS |
| `/joint_states` Rate | 60.1 Hz | 59.8 Hz | 60.0 Hz |
| GPU Utilization (avg) | 78% | 82% | 75% |
| CUDA Cores | 7,680 | 9,216 | 6,144 |

**Validation Criteria**:
- ✅ Both environments achieve ≥30 FPS in Isaac Sim (1080p)
- ✅ `/joint_states` topic publishes within 5% of target rate (57-63 Hz for 60 Hz target)
- ✅ No driver crashes or GPU errors during 10-minute stress test

---

## 9. Further Reading

**Official Documentation**:
- ROS 2 Iron Docs: https://docs.ros.org/en/iron/
- NVIDIA Isaac Sim Manual: https://docs.omniverse.nvidia.com/isaacsim/latest/
- CUDA Installation Guide: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/

**Troubleshooting Resources**:
- ROS Answers (community Q&A): https://answers.ros.org/
- NVIDIA Developer Forums (Isaac Sim): https://forums.developer.nvidia.com/c/omniverse/isaac-sim/
- Ubuntu Community Help: https://help.ubuntu.com/

**Recommended Books**:
- *ROS 2 Robotics Developer Guide* (Ibrahim, 2024): Covers ROS 2 Iron in depth
- *CUDA by Example* (Sanders & Kandrot, 2010): GPU programming fundamentals
- *The Linux Command Line* (Shotts, 2019): Essential Linux skills

---

## Chapter Summary

This chapter guided you through setting up a complete Physical AI development environment. You verified hardware requirements (RTX GPU, 32+ GB RAM, 1+ TB SSD), installed Ubuntu 22.04 LTS, configured NVIDIA drivers (550+) and CUDA Toolkit 12.x, installed ROS 2 Iron Irwini, and set up NVIDIA Isaac Sim 2024.2.

You completed a hands-on lab demonstrating ROS 2 + Isaac Sim integration: loading a 12-DOF humanoid URDF, publishing joint states at 60 Hz, and visualizing in RViz2. Finally, you benchmarked performance across multiple hardware configurations.

**Key Takeaways**:
1. Isaac Sim requires NVIDIA RTX GPU (AMD/Intel not supported); minimum RTX 4070 Ti for 30 FPS
2. Ubuntu 22.04 LTS is recommended over 24.04 for ROS 2 Iron compatibility
3. Nouveau driver must be blacklisted; Secure Boot may need to be disabled for NVIDIA driver installation
4. ROS 2 environment must be sourced (`source /opt/ros/iron/setup.bash`) in every terminal session
5. Isaac Sim's ROS 2 Bridge publishes topics at simulation rate (30-120 Hz depending on GPU)
6. Cloud GPUs (AWS g5.xlarge, Paperspace) are viable alternatives to local hardware (~$200-$300/month)

**Next Steps**: In Chapter 3, we'll dive into ROS 2 fundamentals—creating publishers, subscribers, services, and actions. You'll build a multi-node sensor simulation system and learn to debug with `ros2 topic`, `ros2 service`, and `rqt_graph`.

---

## References

Ibrahim, A. (2024). *ROS 2 robotics developer guide*. Packt Publishing.

NVIDIA Corporation. (2024). *Isaac Sim documentation* (Version 2024.2). https://docs.omniverse.nvidia.com/isaacsim/latest/

ROS 2 Documentation. (2024). *ROS 2 Iron Irwini documentation*. https://docs.ros.org/en/iron/

Sanders, J., & Kandrot, E. (2010). *CUDA by example: An introduction to general-purpose GPU programming*. Addison-Wesley Professional.

Shotts, W. (2019). *The Linux command line* (2nd ed.). No Starch Press.

Ubuntu Documentation Team. (2024). *Ubuntu 22.04 LTS (Jammy Jellyfish) installation guide*. https://help.ubuntu.com/22.04/installation-guide/

---

## Troubleshooting Quick Reference

**NVIDIA Driver Issues**:
```bash
# Check if driver loaded
lsmod | grep nvidia

# Reinstall driver
sudo apt purge nvidia-* -y
sudo ubuntu-drivers autoinstall
sudo reboot
```

**ROS 2 Not Sourced**:
```bash
# Add to .bashrc permanently
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Isaac Sim Crashes on Launch**:
```bash
# Check dmesg for GPU errors
sudo dmesg | grep -i nvidia

# Verify CUDA is installed
nvcc --version

# Check Isaac Sim logs
tail -f ~/.nvidia-omniverse/logs/Kit/Isaac-Sim/*/kit.log
```

**RViz2 "No transform from base_link to map"**:
```bash
# Publish static transform
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

**Docker GPU Access Denied**:
```bash
# Verify NVIDIA Container Toolkit installed
dpkg -l | grep nvidia-container-toolkit

# Test GPU access
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```
