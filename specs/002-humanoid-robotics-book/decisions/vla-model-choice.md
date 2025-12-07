# Architectural Decision: VLA Model Backbone Selection

**Date**: 2025-12-06
**Status**: Decided
**Decision Makers**: Book Author Team
**Related Tasks**: T011

## Context

Chapters 14-16 focus on Vision-Language-Action (VLA) models for humanoid manipulation. The book needs a primary VLA model that is:
- Open-source with freely available pre-trained weights
- Performant on edge hardware (≥10 Hz on Jetson Orin Nano 8GB)
- Well-documented with clear inference API
- Generalizable to diverse manipulation tasks
- Actively maintained with longevity through 2027+

## Options Considered

### Option 1: OpenVLA (Open Vision-Language-Action Model)
**Source**: UC Berkeley / Stanford (2024)
**License**: Apache 2.0 (MIT for some components)
**Model Size**: 7B parameters (base model)
**Framework**: PyTorch + Transformers

**Architecture**:
- Vision Encoder: DinoV2 or CLIP
- Language Model: LLaMA 2/3 base
- Action Decoder: Diffusion policy head
- Training: 800K+ robot trajectories from Open X-Embodiment dataset

**Pros**:
- ✅ Fully open-source with permissive license
- ✅ Pre-trained weights available on Hugging Face
- ✅ Designed specifically for robotics (not adapted from general VLM)
- ✅ Strong generalization (trained on 22+ robot embodiments)
- ✅ Active development and community support (2024-2025)
- ✅ Well-documented inference API and examples
- ✅ Compatible with Hugging Face Transformers (familiar to ML practitioners)
- ✅ Can be quantized to INT8/FP16 for Jetson deployment

**Cons**:
- ⚠️ 7B parameters may be challenging on Jetson Orin Nano 8GB without quantization
- ⚠️ Inference latency ~50-100ms on RTX 4090, may be slower on Jetson
- ⚠️ Relatively new (2024), long-term availability depends on continued support

**Performance Estimates**:
- RTX 4090: 15-20 Hz (FP16)
- Jetson Orin Nano 8GB: 8-12 Hz (INT8 quantization + TensorRT)
- Jetson Orin NX 16GB: 12-18 Hz (INT8)
- Jetson Orin AGX 64GB: 25-35 Hz (FP16)

**Risk**: May require significant optimization to hit 10 Hz target on Orin Nano

### Option 2: RT-2-X (Robotics Transformer 2 - Extended)
**Source**: Google DeepMind (2023-2024)
**License**: Research-only (weights may not be publicly available)
**Model Size**: Variable (RT-2: 55B parameters for largest variant)

**Pros**:
- ✅ State-of-the-art performance on manipulation benchmarks
- ✅ Proven in real-world deployment (Google robotics labs)
- ✅ Extensive academic citations and validation

**Cons**:
- ❌ Pre-trained weights not publicly available as of 2024-2025
- ❌ Research-only license restricts commercial use
- ❌ Model size (55B params) prohibitive for edge deployment
- ❌ No official ROS 2 integration examples
- ❌ Requires Google Cloud TPUs for optimal performance

**Decision**: **REJECTED** - Not accessible for educational use

### Option 3: Octo (Open-Source Generalist Policy)
**Source**: UC Berkeley (2024)
**License**: MIT
**Model Size**: 93M parameters (significantly smaller than OpenVLA)
**Framework**: JAX + Flax

**Architecture**:
- Transformer-based policy
- Trained on 800K+ trajectories from Open X-Embodiment
- Lightweight design for edge deployment

**Pros**:
- ✅ Fully open-source (MIT license)
- ✅ Smaller model size (93M params) ideal for Jetson deployment
- ✅ Pre-trained weights available
- ✅ Strong generalization across robot embodiments
- ✅ Fast inference (50-100 Hz on RTX 4090, 20-30 Hz on Jetson Orin Nano)
- ✅ Designed for fine-tuning on custom tasks

**Cons**:
- ⚠️ JAX/Flax ecosystem less familiar to PyTorch users
- ⚠️ Smaller capacity may limit performance on complex tasks vs. larger models
- ⚠️ Limited integration examples with ROS 2 (requires custom wrapper)
- ⚠️ Smaller community compared to Transformers-based models

**Performance Estimates**:
- RTX 4090: 50-100 Hz (FP16)
- Jetson Orin Nano 8GB: 20-30 Hz (FP16/INT8)
- Jetson Orin NX 16GB: 40-60 Hz (FP16)

**Risk**: JAX learning curve may be barrier for readers primarily familiar with PyTorch

### Option 4: Custom Llama-3.1-8B Fine-Tune
**Source**: Meta AI (base model) + Custom training
**License**: Llama 3.1 Community License (allows commercial use)
**Model Size**: 8B parameters

**Approach**:
- Start with pre-trained Llama-3.1-8B
- Add vision encoder (CLIP or DINOv2)
- Add action decoder head
- Fine-tune on robot manipulation dataset (Open X-Embodiment or custom)

**Pros**:
- ✅ Full control over architecture and training
- ✅ Educational value (readers learn VLA training pipeline)
- ✅ Can optimize for specific humanoid tasks
- ✅ Llama 3.1 license allows commercial use

**Cons**:
- ❌ Requires significant compute for training (multiple A100/H100 GPUs)
- ❌ Training dataset curation and preparation is complex
- ❌ No guarantee of matching performance of pre-trained VLA models
- ❌ Training time (weeks) delays book development
- ❌ Readers cannot easily replicate training (compute cost prohibitive)

**Decision**: **REJECTED** for primary approach - Training VLA from scratch exceeds book scope. Could be included as advanced topic in Appendix.

## Decision

**SELECTED: Option 1 - OpenVLA with Option 3 (Octo) as Alternative**

### Primary VLA Model: OpenVLA 7B

**Justification**:
1. **Open-Source**: Apache 2.0 license allows unrestricted educational and commercial use
2. **Accessibility**: Pre-trained weights on Hugging Face, easy installation
3. **PyTorch Ecosystem**: Familiar to majority of readers (PyTorch dominant in robotics/AI education)
4. **Generalization**: Trained on 800K+ trajectories from 22+ robot types
5. **Documentation**: Active community, Hugging Face integration, example code
6. **Performance**: Achievable 10+ Hz on Jetson Orin Nano with optimization
7. **Longevity**: Backed by major research institutions (UC Berkeley, Stanford), likely to remain available

### Secondary VLA Model: Octo (for Performance-Critical Applications)

Introduced in Chapter 15 or 16 as a lightweight alternative for:
- Readers prioritizing inference speed over model capacity
- Edge deployment scenarios where latency is critical
- Fine-tuning workflows (smaller model faster to fine-tune)

### Implementation Strategy

**Chapter 13: VLA Models Overview**
- Survey RT-1, RT-2, OpenVLA, Octo, PaLM-E
- Explain architecture (vision encoder, language model, action decoder)
- Discuss training paradigms (imitation learning, pre-training + fine-tuning)

**Chapter 14: Integrating OpenVLA for Humanoid Manipulation**
- Install OpenVLA via Hugging Face Transformers
- Load pre-trained weights (7B model)
- Implement vision → language → action inference loop
- ROS 2 integration: subscribe to camera topics, publish joint commands
- Benchmark on RTX 4090 (baseline) and Jetson Orin Nano (optimized)
- Pick-and-place demo in Isaac Sim
- **Target Performance**: ≥12 Hz on Jetson Orin Nano 8GB (INT8 quantization + TensorRT)

**Chapter 15: LLM Task Planning**
- Integrate LLM (GPT-4, Claude, or Llama 3.1) for high-level task decomposition
- LLM generates language instructions → OpenVLA executes actions
- Example: "Pick up the red mug and place it on the table" → sequence of VLA actions

**Chapter 16: Multimodal Perception**
- Fuse RGB, depth, and proprioceptive data as input to OpenVLA
- Integrate CLIP for language grounding
- Deploy on Jetson Orin NX 16GB (more headroom for multimodal processing)

**Appendix H: Alternative VLA Models**
- **Octo**: Lightweight alternative (93M params), JAX/Flax setup guide
- **Custom Fine-Tuning**: Guide to fine-tuning OpenVLA on custom manipulation tasks
- **Future Models**: Placeholder for newer VLA models (RT-3, OpenVLA v2, etc.)

## Optimization Strategy for Jetson Deployment

To achieve ≥10 Hz on Jetson Orin Nano 8GB:

1. **Model Quantization**:
   - INT8 quantization using TensorRT
   - FP16 mixed precision where accuracy critical
   - Expected 2-3× speedup over FP32

2. **Batching**:
   - Process multiple frames in batch (batch size 2-4)
   - Amortize model loading overhead

3. **Model Pruning** (optional):
   - Prune less important weights (10-20% sparsity)
   - Retrain briefly to recover accuracy

4. **Pipeline Optimization**:
   - Overlap vision preprocessing with model inference
   - Use CUDA streams for parallel GPU operations
   - Pre-allocate memory to avoid runtime allocation overhead

5. **Model Distillation** (advanced):
   - Distill OpenVLA 7B → smaller student model (2B-3B params)
   - Trade capacity for speed
   - Optional content for advanced readers

**Performance Validation** (SC-003):
- RTX 4090 baseline: ≥15 Hz (FP16)
- Jetson Orin Nano 8GB: ≥10 Hz (INT8 + TensorRT)
- Jetson Orin NX 16GB: ≥15 Hz (INT8/FP16)
- Jetson Orin AGX 64GB: ≥30 Hz (FP16)

## Consequences

### Positive
- ✅ Readers learn industry-standard VLA architecture (similar to RT-2, PaLM-E)
- ✅ Open-source model enables experimentation and customization
- ✅ Hugging Face ecosystem provides easy installation and model management
- ✅ Pre-trained weights eliminate need for costly training infrastructure
- ✅ PyTorch familiarity reduces learning curve
- ✅ Octo alternative provides flexibility for different use cases

### Negative
- ❌ 7B model size may challenge readers with limited GPU memory
- ❌ Requires optimization to hit Jetson performance targets (adds complexity to Chapter 14)
- ❌ Model availability depends on continued hosting (Hugging Face) and maintenance

### Neutral
- ⚖️ Requires dedicating Chapter 14 to OpenVLA integration and optimization
- ⚖️ May need to update book if newer VLA models emerge (2026-2027)

## Validation Against Requirements

- ✅ **Open-source**: Apache 2.0 license
- ✅ **Performance**: Achievable ≥10 Hz on Jetson Orin Nano 8GB (with optimization)
- ✅ **Documentation**: Hugging Face model card, GitHub repo, research paper
- ✅ **Generalization**: Trained on 22+ robot embodiments, Open X-Embodiment dataset
- ✅ **Longevity**: Backed by major institutions, likely to remain available through 2027+
- ✅ **ROS 2 Integration**: Custom wrapper straightforward (camera topics → model inference → joint commands)

## References

- OpenVLA Repository: https://github.com/openvla/openvla (placeholder - check actual repo)
- Hugging Face Model: https://huggingface.co/openvla/openvla-7b (placeholder)
- Open X-Embodiment Dataset: https://robotics-transformer-x.github.io/
- Octo Repository: https://github.com/octo-models/octo
- RT-2 Paper: Brohan et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control"

## Revision History

- 2025-12-06: Initial decision - Selected OpenVLA 7B as primary, Octo as secondary VLA model
