// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://muskaanfayyaz.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Physical-AI-Humanoid-Robotics/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'muskaanfayyaz', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics', // Usually your repo name.
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
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
          routeBasePath: '/', // Serve docs at the site root
          sidebarPath: require.resolve('./sidebars.js'),

          // HACKATHON SUBMISSION: editUrl removed to make textbook read-only
          // This prevents "Edit this page" links from appearing in the deployed site
          // Repository remains public, but content is presented as a finished product
          // editUrl: 'https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics/tree/main/', // REMOVED
        },
        blog: false, // Disable blog
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo-transparent.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Course',
            items: [
              {
                label: 'Introduction',
                to: '/',
              },
              {
                label: 'Foundations',
                to: '/chapters/chapter-01-introduction-to-physical-ai',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Panaversity',
                href: 'https://panaversity.org',
              },
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'NVIDIA Isaac',
                href: 'https://developer.nvidia.com/isaac-ros',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'yaml', 'json'],
      },
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    }),
};

module.exports = config;
