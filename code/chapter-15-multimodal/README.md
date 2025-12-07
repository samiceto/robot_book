# Chapter 15: Multimodal Reasoning

## Quick Start

```bash
# Setup GPT-4V API
export OPENAI_API_KEY=your_key_here

# Run multimodal pipeline
python3 multimodal_controller.py \
  --vlm gpt-4-vision \
  --vla humanoid_vla_lora \
  --task "Clear the table"

# Test on scenarios
python3 test_scenarios.py --num-scenes 10
```

## Architecture

```
Image + Task → VLM (GPT-4V) → Subtasks → VLA (OpenVLA) → Actions
```

## Performance

- VLA only: 50% success
- VLM + VLA: 70% success (20% improvement)
