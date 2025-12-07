// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const {themes} = require('prism-react-renderer');
const math = require('remark-math');
const katex = require('rehype-katex');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulated Brains to Walking Bodies',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://samiceto.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/robot_book/',

  // GitHub pages deployment config
  organizationName: 'samiceto', // Usually your GitHub org/user name.
  projectName: 'robot_book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Internationalization
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          remarkPlugins: [math],
          rehypePlugins: [katex],
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      crossorigin: 'anonymous',
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'bookSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            to: '/resources',
            label: 'Resources',
            position: 'left'
          },
          {
            to: '/errata',
            label: 'Errata',
            position: 'left'
          },
          {
            href: 'https://github.com/samiceto/robot_book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book',
            items: [
              {
                label: 'Part 1: Foundations',
                to: '/docs/part1-foundations/introduction',
              },
              {
                label: 'Part 2: Simulation',
                to: '/docs/part2-simulation/gazebo-basics',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Code Repositories',
                to: '/resources',
              },
              {
                label: 'Hardware Guide',
                to: '/docs/appendices/hardware-buyers-guide',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/samiceto/robot_book',
              },
              {
                label: 'Errata',
                to: '/errata',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.dracula,
        additionalLanguages: ['python', 'cpp', 'bash', 'yaml', 'xml'],
      },
      // Algolia search - uncomment when ready to set up
      // algolia: {
      //   appId: 'YOUR_APP_ID',
      //   apiKey: 'YOUR_SEARCH_API_KEY',
      //   indexName: 'robot_book',
      //   contextualSearch: true,
      // },
    }),
};

module.exports = config;
