#!/usr/bin/env python3
"""
Convert Isaac Sim Replicator output to COCO format.

COCO (Common Objects in Context) is the industry-standard format for object
detection datasets. This script converts BasicWriter bounding boxes to COCO
format for training with PyTorch, TensorFlow, or Detectron2.

Usage:
    # Convert Replicator output to COCO format
    python3 convert_to_coco.py \
      --input-dir ~/datasets/replicator_output \
      --output-file ~/datasets/coco_annotations.json

    # Convert grasping dataset
    python3 convert_to_coco.py \
      --input-dir ~/datasets/grasping_dataset \
      --output-file ~/datasets/grasping_coco.json

    # Specify custom category mapping
    python3 convert_to_coco.py \
      --input-dir ~/datasets/grasping_dataset \
      --output-file ~/datasets/grasping_coco.json \
      --categories "cube,cylinder,sphere,humanoid"
"""

import argparse
import json
import os
import numpy as np
from datetime import datetime

parser = argparse.ArgumentParser(description="Convert Replicator output to COCO format")
parser.add_argument("--input-dir", type=str, required=True,
                    help="Directory containing Replicator output (bounding_box_2d_tight.npy)")
parser.add_argument("--output-file", type=str, required=True,
                    help="Output COCO JSON file path")
parser.add_argument("--categories", type=str, default="cube,cylinder,sphere",
                    help="Comma-separated list of category names (in order of semantic IDs)")
args = parser.parse_args()

# Expand paths
input_dir = os.path.expanduser(args.input_dir)
output_file = os.path.expanduser(args.output_file)

# Parse categories
category_names = [name.strip() for name in args.categories.split(",")]


def load_bounding_boxes(input_dir: str):
    """
    Load aggregated bounding boxes from BasicWriter output.

    Args:
        input_dir: Directory containing bounding_box_2d_tight.npy

    Returns:
        NumPy structured array with bounding boxes
    """
    bbox_path = os.path.join(input_dir, "bounding_box_2d_tight.npy")

    if not os.path.exists(bbox_path):
        print(f"ERROR: Bounding box file not found: {bbox_path}")
        return None

    try:
        data = np.load(bbox_path, allow_pickle=True)
        print(f"Loaded bounding boxes from: {bbox_path}")
        print(f"  Total frames: {len(data)}")
        return data
    except Exception as e:
        print(f"ERROR loading bounding boxes: {e}")
        return None


def get_image_dimensions(input_dir: str, frame_idx: int = 0):
    """
    Get image dimensions from the first RGB image.

    Args:
        input_dir: Dataset directory
        frame_idx: Frame index to check

    Returns:
        (width, height) tuple
    """
    import cv2
    rgb_path = os.path.join(input_dir, f"rgb/rgb_{frame_idx:04d}.png")

    if not os.path.exists(rgb_path):
        print(f"WARNING: RGB image not found: {rgb_path}")
        return (640, 480)  # Default resolution

    img = cv2.imread(rgb_path)
    if img is None:
        return (640, 480)

    height, width = img.shape[:2]
    return (width, height)


def create_coco_categories(category_names: list):
    """
    Create COCO categories list.

    Args:
        category_names: List of category names

    Returns:
        List of category dicts
    """
    categories = []
    for i, name in enumerate(category_names, 1):
        categories.append({
            "id": i,
            "name": name,
            "supercategory": "object"
        })
    return categories


def convert_to_coco(bbox_data, input_dir: str, category_names: list):
    """
    Convert BasicWriter bounding boxes to COCO format.

    Args:
        bbox_data: NumPy structured array from BasicWriter
        input_dir: Dataset directory (for image paths)
        category_names: List of category names

    Returns:
        COCO format dict
    """
    # Get image dimensions
    width, height = get_image_dimensions(input_dir)

    # Initialize COCO structure
    coco_dataset = {
        "info": {
            "description": "Synthetic dataset generated with Isaac Sim Replicator",
            "version": "1.0",
            "year": datetime.now().year,
            "date_created": datetime.now().isoformat(),
        },
        "licenses": [
            {
                "id": 1,
                "name": "MIT License",
                "url": "https://opensource.org/licenses/MIT"
            }
        ],
        "images": [],
        "annotations": [],
        "categories": create_coco_categories(category_names),
    }

    # Counters
    annotation_id = 0
    total_annotations = 0

    # Process each frame
    for image_id, frame_data in enumerate(bbox_data):
        # Add image entry
        image_filename = f"rgb/rgb_{image_id:04d}.png"
        coco_dataset["images"].append({
            "id": image_id,
            "file_name": image_filename,
            "width": width,
            "height": height,
        })

        # Process bounding boxes in this frame
        if 'data' in frame_data.dtype.names:
            bbox_list = frame_data['data']

            for bbox in bbox_list:
                # BasicWriter format: [x_min, y_min, x_max, y_max]
                x_min = float(bbox['x_min'])
                y_min = float(bbox['y_min'])
                x_max = float(bbox['x_max'])
                y_max = float(bbox['y_max'])

                # Convert to COCO format: [x, y, width, height]
                x = x_min
                y = y_min
                w = x_max - x_min
                h = y_max - y_min

                # Skip invalid bounding boxes
                if w <= 0 or h <= 0:
                    continue

                # Get semantic ID (1-indexed in COCO)
                semantic_id = int(bbox.get('semanticId', 1))

                # Ensure semantic_id is valid
                if semantic_id < 1 or semantic_id > len(category_names):
                    print(f"WARNING: Invalid semantic_id {semantic_id} in frame {image_id}, skipping")
                    continue

                # Add annotation
                coco_dataset["annotations"].append({
                    "id": annotation_id,
                    "image_id": image_id,
                    "category_id": semantic_id,
                    "bbox": [x, y, w, h],
                    "area": w * h,
                    "iscrowd": 0,
                })

                annotation_id += 1
                total_annotations += 1

    print(f"Total annotations created: {total_annotations}")
    return coco_dataset


def save_coco_json(coco_dataset: dict, output_file: str):
    """
    Save COCO dataset to JSON file.

    Args:
        coco_dataset: COCO format dict
        output_file: Output file path
    """
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    with open(output_file, 'w') as f:
        json.dump(coco_dataset, f, indent=2)

    print(f"COCO dataset written to: {output_file}")


def print_statistics(coco_dataset: dict):
    """
    Print dataset statistics.

    Args:
        coco_dataset: COCO format dict
    """
    num_images = len(coco_dataset["images"])
    num_annotations = len(coco_dataset["annotations"])
    num_categories = len(coco_dataset["categories"])

    print("")
    print("=" * 70)
    print("COCO CONVERSION STATISTICS")
    print("=" * 70)
    print(f"Images:      {num_images}")
    print(f"Annotations: {num_annotations}")
    print(f"Categories:  {num_categories}")
    print("")
    print("Category Breakdown:")
    for category in coco_dataset["categories"]:
        cat_id = category["id"]
        cat_name = category["name"]
        count = sum(1 for ann in coco_dataset["annotations"] if ann["category_id"] == cat_id)
        print(f"  - {cat_name} (ID: {cat_id}): {count} annotations")
    print("=" * 70)


def main():
    """Main conversion function."""

    print("=" * 70)
    print("REPLICATOR TO COCO CONVERSION")
    print("=" * 70)
    print(f"Input directory:  {input_dir}")
    print(f"Output file:      {output_file}")
    print(f"Categories:       {', '.join(category_names)}")
    print("=" * 70)

    # Load bounding boxes
    bbox_data = load_bounding_boxes(input_dir)
    if bbox_data is None:
        print("ERROR: Failed to load bounding boxes. Exiting.")
        return

    # Convert to COCO format
    print("\nConverting to COCO format...")
    coco_dataset = convert_to_coco(bbox_data, input_dir, category_names)

    # Save to file
    save_coco_json(coco_dataset, output_file)

    # Print statistics
    print_statistics(coco_dataset)

    print("")
    print("CONVERSION COMPLETE")
    print("")
    print("To use this dataset with PyTorch:")
    print("  from pycocotools.coco import COCO")
    print(f"  coco = COCO('{output_file}')")
    print("  image_ids = coco.getImgIds()")
    print("")


if __name__ == "__main__":
    main()
