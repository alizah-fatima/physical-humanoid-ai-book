// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Course',
  tagline: 'AI Systems in the Physical World. Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.',
  favicon: 'img/ai.ico',

  // Set the production url of your site here
  url: 'https://physical-humanoid-ai-book-oi3595z3h-alizah-fatimas-projects.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, use '/' for root
  baseUrl: '/',

  // For proper URL handling on Vercel
  trailingSlash: false,

  // GitHub pages deployment config.
  organizationName: 'alizah-fatima', // Usually your GitHub org/user name.
  projectName: 'physical-humanoid-ai-book', // Usually your repo name.

  onBrokenLinks: 'throw',
  markdown: {
    mermaid: true,
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'], // Only English for now, since we don't have Urdu translations
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/alizah-fatima/physical-humanoid-ai-book/tree/main/',
        },
        blog: false, // Disable blog plugin as we're creating a textbook
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
      image: 'img/logo.svg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'AI Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/alizah-fatima/physical-humanoid-ai-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'ROS 2 Fundamentals',
                to: '/docs/module1-ros2/chapter1-architecture',
              },
              {
                label: 'Digital Twin',
                to: '/docs/module2-digital-twin/chapter1-gazebo-intro',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'AI & Robotics Guide',
                to: '/docs/module3-nvidia-isaac/chapter1-isaac-sim-intro',
              },
              {
                label: 'Vision-Language-Action',
                to: '/docs/module4-vla/chapter1-vla-intro',
              },
            ],
          },
          {
            title: 'Connect',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/alizah-fatima',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/alizah-fatima008/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),

  plugins: [],
};

module.exports = config;
