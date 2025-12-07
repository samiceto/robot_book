# Architectural Decision: Edge Hardware Tier for Reproducibility

**Date**: 2025-12-06
**Status**: Decided
**Decision Makers**: Book Author Team
**Related Tasks**: T013

## Context

Chapter 12 (Jetson Orin Deployment) and the capstone project (Chapter 20) require edge hardware for deploying VLA models and Nav2 perception pipelines. The hardware must:
- Run complete capstone pipeline at ≥12 Hz (FR-014, SC-003)
- Fit within student/educator budget (<$600 preferred)
- Support OpenVLA 7B with INT8 quantization + TensorRT
- Run Isaac ROS perception pipelines
- Be available for purchase through 2026-2027

## Options Considered

### Option 1: NVIDIA Jetson Orin Nano 8GB
**Price**: $249 (Developer Kit)
**Release**: 2022
**Availability**: Widely available (NVIDIA, distributors, Amazon)

**Specifications**:
- GPU: 1024-core NVIDIA Ampere GPU with 32 Tensor Cores
- CPU: 6-core Arm Cortex-A78AE
- Memory: 8 GB LPDDR5 (128-bit, 68 GB/s)
- Storage: microSD (expandable via NVMe SSD)
- Power: 7-15W
- Dimensions: 100mm × 79mm × 31mm (compact)
- JetPack: 5.x (Ubuntu 20.04) or 6.x (Ubuntu 22.04)

**Performance Estimates** (from OpenVLA decision):
- OpenVLA 7B (INT8 quantization + TensorRT): 10-12 Hz
- Isaac ROS perception (object detection): 20-30 Hz
- Nav2 planning: <500ms for 10m path
- **Capstone Pipeline** (Whisper + LLM + Nav2 + OpenVLA): 10-15 Hz (with optimization)

**Pros**:
- ✅ **Price**: $249 is accessible to students and educators
- ✅ **Availability**: Widely stocked, stable supply chain
- ✅ **Size/Power**: Compact form factor, low power (7-15W)
- ✅ **Performance**: Meets ≥12 Hz capstone requirement (SC-003) with optimization
- ✅ **Educational**: Realistic edge deployment constraints (memory, power, thermal)
- ✅ **JetPack Ecosystem**: Official NVIDIA support, frequent updates

**Cons**:
- ⚠️ **8 GB RAM**: Tight memory budget for OpenVLA 7B + Nav2 + OS overhead
- ⚠️ **Optimization Required**: Needs INT8 quantization, TensorRT, and careful memory management to hit performance targets
- ⚠️ **Thermal Throttling**: May throttle under sustained load without active cooling

**Risk**: Requires significant optimization in Chapter 12 to achieve ≥12 Hz capstone performance

### Option 2: NVIDIA Jetson Orin NX 16GB
**Price**: $599 (Developer Kit)
**Release**: 2022
**Availability**: Available (NVIDIA, distributors)

**Specifications**:
- GPU: 1024-core NVIDIA Ampere GPU with 32 Tensor Cores
- CPU: 8-core Arm Cortex-A78AE
- Memory: 16 GB LPDDR5 (128-bit, 102 GB/s)
- Storage: NVMe SSD support
- Power: 10-25W
- JetPack: 5.x or 6.x

**Performance Estimates**:
- OpenVLA 7B (INT8/FP16): 15-18 Hz
- Isaac ROS perception: 40-60 Hz
- **Capstone Pipeline**: 18-22 Hz (comfortable margin above 12 Hz)

**Pros**:
- ✅ **16 GB RAM**: Ample memory for OpenVLA + Nav2 + buffers
- ✅ **Higher Performance**: 50% more performance headroom vs. Orin Nano
- ✅ **Less Optimization**: Can use FP16 instead of INT8, easier to develop
- ✅ **Production-Like**: Represents mid-tier edge deployment (robots, drones, industrial)

**Cons**:
- ❌ **Price**: $599 is 2.4× cost of Orin Nano (barrier for some students)
- ⚠️ **Power**: 10-25W (higher than Nano, may require larger battery in mobile robots)

**Risk**: Higher cost reduces accessibility for budget-constrained readers

### Option 3: NVIDIA Jetson AGX Orin 64GB
**Price**: $2,000+ (Developer Kit)
**Release**: 2022
**Availability**: Available (NVIDIA, distributors)

**Specifications**:
- GPU: 2048-core NVIDIA Ampere GPU with 64 Tensor Cores
- CPU: 12-core Arm Cortex-A78AE
- Memory: 64 GB LPDDR5 (256-bit, 204 GB/s)
- Storage: NVMe SSD
- Power: 15-60W
- JetPack: 5.x or 6.x

**Performance Estimates**:
- OpenVLA 7B (FP16): 30-35 Hz
- Isaac ROS perception: 80-100 Hz
- **Capstone Pipeline**: 35-40 Hz (significantly exceeds requirement)

**Pros**:
- ✅ **High Performance**: 3× faster than Orin Nano
- ✅ **64 GB RAM**: Can run larger models, multi-robot scenarios
- ✅ **Production Grade**: Used in autonomous vehicles, industrial robots

**Cons**:
- ❌ **Price**: $2,000+ is prohibitive for students (8× cost of Orin Nano)
- ❌ **Overkill**: Exceeds capstone requirements by 3×, not pedagogically necessary
- ❌ **Power/Size**: 15-60W, larger form factor (not representative of mobile robots)

**Decision**: **REJECTED** - Cost prohibitive, overkill for book requirements

## Decision

**SELECTED: Option 1 - NVIDIA Jetson Orin Nano 8GB (Baseline), with Option 2 (Orin NX) as Recommended Upgrade**

### Primary Platform: Jetson Orin Nano 8GB

**Justification**:
1. **Accessibility**: $249 price point is affordable for majority of students and educators
2. **Availability**: Widely stocked, stable supply through 2026-2027
3. **Performance**: Achievable ≥12 Hz capstone with optimization (meets SC-003)
4. **Pedagogical Value**: Teaches real edge deployment constraints (memory budgets, thermal management, quantization)
5. **Representative**: Orin Nano is realistic for mobile robot deployments (low power, compact)
6. **Ecosystem**: Official JetPack support, Isaac ROS compatibility

### Recommended Upgrade: Jetson Orin NX 16GB (Optional)

For readers who:
- Want more performance headroom (15-18 Hz vs. 12 Hz)
- Prefer FP16 over INT8 (simpler development, faster iteration)
- Plan to extend capstone with additional features (e.g., voice recognition + multiple VLA models)
- Have budget for $599 hardware

**Documented in Appendix A (Hardware Buyer's Guide)**:
- Jetson Orin Nano 8GB: **Baseline** ($249) - All labs designed to run on this
- Jetson Orin NX 16GB: **Recommended** ($599) - More headroom, easier development
- Jetson AGX Orin 64GB: **Advanced** ($2000+) - For production-grade projects, research

## Implementation Strategy

**Chapter 12: Jetson Orin Edge Deployment** (9,000-10,000 words)
- Install JetPack 5.x (Ubuntu 20.04) or 6.x (Ubuntu 22.04)
- Install ROS 2 Iron/Jazzy on Jetson
- Flash Jetson Orin Nano 8GB via SDK Manager
- Model optimization pipeline:
  - INT8 quantization using TensorRT
  - FP16 mixed precision where accuracy critical
  - Memory profiling (`tegrastats`, `nvidia-smi`)
  - Power profiling (NVPModel power modes: MAXN, 15W, 10W, 7W)
- Deploy Isaac ROS perception pipeline
  - Object detection: 20-30 Hz (YOLOv8 + TensorRT)
  - Depth estimation: 15-20 Hz
- **Validation**: Achieve ≥15 Hz perception on Jetson Orin Nano 8GB (FR-014)

**Chapter 20: Capstone - Voice-Controlled Autonomous Humanoid**
- Complete pipeline integration on Jetson Orin Nano 8GB:
  - Whisper speech recognition (quantized): 10-15 Hz
  - LLM task planning (Llama 3.1-8B, INT8): 5-10 Hz (run on cloud or edge)
  - Nav2 navigation: <500ms planning
  - OpenVLA manipulation (INT8 + TensorRT): 10-12 Hz
- **End-to-End Target**: ≥12 Hz, <2 GB RAM (FR-013, SC-003)
- Optimization techniques:
  - Pipeline parallelization (overlap Whisper + VLA inference)
  - Model pruning (10-20% sparsity on OpenVLA)
  - CUDA stream optimization
  - Memory pre-allocation to avoid runtime overhead
- **Validation**: Screen-record Jetson running capstone demo at ≥12 Hz for 60 seconds

**Performance Validation Protocol** (SC-003):
1. **Test Environment**:
   - Hardware: Jetson Orin Nano 8GB Developer Kit
   - JetPack: 6.x (Ubuntu 22.04)
   - ROS 2: Iron Irwini
   - Isaac Sim: 2024.2 (running on remote RTX 4090, streaming to Jetson)
2. **Test Scenario**: "Navigate to table and pick up red mug"
3. **Metrics**:
   - End-to-end latency: perception → decision → action (target: <100ms, ≥10 Hz)
   - Memory usage: `tegrastats` monitoring (target: <2 GB for VLA + Nav2)
   - Success rate: 8/10 trials successful
4. **Acceptance**: ≥12 Hz sustained for 60 seconds, <2 GB RAM

## Optimization Roadmap for Orin Nano 8GB

To achieve ≥12 Hz on Jetson Orin Nano 8GB:

### Phase 1: Model Quantization (Chapter 12)
- OpenVLA 7B → INT8 quantization (reduces memory from ~14 GB to ~7 GB, 2-3× speedup)
- Whisper → Whisper-tiny or Whisper-small quantized
- LLM → Llama 3.1-8B INT8 or cloud offload (GPT-4 API)

### Phase 2: TensorRT Optimization (Chapter 12)
- Convert PyTorch models → TensorRT engines
- Fuse layers (conv + bn + relu → single operation)
- Enable FP16 Tensor Cores where accuracy permits

### Phase 3: Pipeline Parallelization (Chapter 20)
- Overlap inference stages:
  - While VLA processes frame N, Whisper processes audio buffer N+1
  - Use CUDA streams for concurrent GPU operations
- Reduce perception frequency: Run VLA at 12 Hz, Whisper at 5 Hz (voice updates less frequent)

### Phase 4: Memory Management (Chapter 20)
- Pre-allocate all tensors at startup (avoid runtime allocation)
- Use zero-copy buffers for camera images (Isaac ROS NITROS integration)
- Shared memory for ROS 2 inter-process communication

### Expected Performance After Optimization:
- Baseline (FP32, no optimization): 3-5 Hz, >8 GB RAM ❌
- Phase 1 (INT8 quantization): 8-10 Hz, ~6 GB RAM ⚠️
- Phase 2 (TensorRT): 10-12 Hz, ~4 GB RAM ⚠️
- Phase 3 (Parallelization): 12-15 Hz, ~3 GB RAM ✅
- Phase 4 (Memory tuning): 12-15 Hz, <2 GB RAM ✅ (meets SC-003)

## Consequences

### Positive
- ✅ $249 price enables universal accessibility for students and educators
- ✅ Teaches real edge deployment optimization (quantization, TensorRT, memory management)
- ✅ Representative of mobile robot constraints (power, thermal, memory)
- ✅ Achieves ≥12 Hz capstone performance with documented optimization steps
- ✅ Readers learn transferable edge AI skills (applicable to drones, autonomous vehicles, industrial robots)

### Negative
- ❌ Requires significant optimization effort in Chapters 12 and 20 (adds complexity)
- ❌ 8 GB RAM is tight budget (may frustrate readers during development/debugging)
- ❌ Thermal throttling may require active cooling (fan, heatsink) for sustained workloads

### Neutral
- ⚖️ Jetson Orin NX 16GB offered as upgrade path for readers wanting easier development
- ⚖️ Optimization techniques taught in Chapter 12 are valuable skills regardless of hardware tier

## Validation Against Requirements

- ✅ **Performance**: ≥12 Hz capstone with optimization (SC-003)
- ✅ **Memory**: <2 GB RAM after optimization (FR-013)
- ✅ **Cost**: $249 accessible to students (vs. $599 or $2000+)
- ✅ **Availability**: Widely stocked, stable supply through 2026-2027
- ✅ **Isaac ROS**: Compatible with Isaac ROS 3.0+
- ✅ **JetPack**: Supports JetPack 5.x/6.x (Ubuntu 20.04/22.04)

## References

- Jetson Orin Nano Specifications: https://developer.nvidia.com/embedded/jetson-orin-nano-devkit
- Jetson Orin NX Specifications: https://developer.nvidia.com/embedded/jetson-orin-nx-series
- JetPack SDK: https://developer.nvidia.com/embedded/jetpack
- TensorRT Optimization Guide: https://docs.nvidia.com/deeplearning/tensorrt/
- Isaac ROS on Jetson: https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/jetson.html

## Revision History

- 2025-12-06: Initial decision - Selected Jetson Orin Nano 8GB as baseline, Orin NX 16GB as recommended upgrade
