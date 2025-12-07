/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Part 1: Foundations & ROS 2',
      items: [
        'part1-foundations/introduction-to-physical-ai',
        'part1-foundations/development-environment-setup',
        'part1-foundations/ros2-fundamentals',
        'part1-foundations/robot-description-urdf-sdf',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: Digital Twins & Simulation',
      items: [
        'part2-simulation/gazebo-basics',
        'part2-simulation/isaac-sim-introduction',
        'part2-simulation/isaac-sim-advanced',
        'part2-simulation/simulation-benchmarking',
      ],
    },
    {
      type: 'category',
      label: 'Part 3: Perception & Edge AI',
      items: [
        'part3-perception/realsense-integration',
        'part3-perception/object-detection-yolo',
        'part3-perception/jetson-orin-deployment',
        'part3-perception/containerization-cicd',
      ],
    },
    {
      type: 'category',
      label: 'Part 4: Vision-Language-Action Models',
      items: [
        'part4-vla/vla-architecture',
        'part4-vla/openvla-finetuning',
        'part4-vla/multimodal-reasoning',
        'part4-vla/visuomotor-control',
      ],
    },
    {
      type: 'category',
      label: 'Part 5: Advanced Topics',
      items: [
        'part5-advanced/bipedal-locomotion',
        'part5-advanced/whole-body-control',
        'part5-advanced/human-robot-interaction',
        'part5-advanced/safety-compliance',
        'part5-advanced/production-deployment',
      ],
    },
  ],
};

module.exports = sidebars;
