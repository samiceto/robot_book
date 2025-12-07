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
        'part1-foundations/01-introduction-to-physical-ai',
        'part1-foundations/02-development-environment-setup',
        'part1-foundations/03-ros2-fundamentals',
        'part1-foundations/04-robot-description-urdf-sdf',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: Digital Twins & Simulation',
      items: [
        'part2-simulation/05-gazebo-basics',
        'part2-simulation/06-isaac-sim-introduction',
        'part2-simulation/07-isaac-sim-advanced',
        'part2-simulation/08-simulation-benchmarking',
      ],
    },
    {
      type: 'category',
      label: 'Part 3: Perception & Edge AI',
      items: [
        'part3-perception/09-realsense-integration',
        'part3-perception/10-object-detection-yolo',
        'part3-perception/11-jetson-orin-deployment',
        'part3-perception/12-containerization-cicd',
      ],
    },
    {
      type: 'category',
      label: 'Part 4: Vision-Language-Action Models',
      items: [
        'part4-vla/13-vla-architecture',
        'part4-vla/14-openvla-finetuning',
        'part4-vla/15-multimodal-reasoning',
        'part4-vla/16-visuomotor-control',
      ],
    },
    {
      type: 'category',
      label: 'Part 5: Advanced Topics',
      items: [
        'part5-advanced/17-bipedal-locomotion',
        'part5-advanced/18-whole-body-control',
        'part5-advanced/19-human-robot-interaction',
        'part5-advanced/20-safety-compliance',
        'part5-advanced/21-production-deployment',
      ],
    },
  ],
};

module.exports = sidebars;
