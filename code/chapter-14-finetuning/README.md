# Chapter 14: OpenVLA Fine-Tuning

## Quick Start

```bash
# Prepare dataset
python3 prepare_dataset.py --input datasets/grasping_dataset --output rlds/

# Fine-tune with LoRA
python3 finetune_vla.py \
  --model openvla/openvla-7b \
  --dataset rlds/ \
  --lora-rank 16 \
  --epochs 10

# Evaluate
python3 evaluate.py --model humanoid_vla_lora --test-set rlds/test/
```

## Training Time

- RTX 4090: 4-6 hours (1000 demos)
- A100: 2-3 hours

## Success Rate

- Pre-trained: 40-50%
- Fine-tuned: 70-80%
