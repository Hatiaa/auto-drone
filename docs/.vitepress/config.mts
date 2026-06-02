import { withDuxTheme } from '@duxweb/vitepress-theme/config'
import taskLists from 'markdown-it-task-lists'

export default withDuxTheme({
  title: 'auto-drone',
  description: '基于 Pixhawk、ArduPilot、Jetson Nano 和 ArUco 的自主飞行项目文档',
  lang: 'zh-CN',
  cleanUrls: true,
  head: [
    ['link', { rel: 'icon', type: 'image/svg+xml', href: '/favicon.svg' }]
  ],
  markdown: {
    config(md) {
      md.use(taskLists)
    }
  },
  themeConfig: {
    siteTitle: 'auto-drone',
    nav: [
      { text: '文档首页', link: '/' }
    ],
    sidebar: [
      {
        text: '项目文档',
        items: [
          { text: '首页', link: '/index' },
          { text: '硬件接线', link: '/hardware-wiring' },
          { text: 'Nano 热点与 SSH', link: '/nano-hotspot-ssh' },
          { text: 'ArduPilot 参数配置', link: '/ardupilot-params' },
          { text: '自主前飞流程', link: '/flight-guided-forward' },
          { text: 'ArUco 视觉跟随', link: '/vision-aruco-follow' },
          { text: '相机与布局标定', link: '/calibration' },
          { text: '检查表', link: '/checklists' },
          { text: '排错指南', link: '/troubleshooting' }
        ]
      }
    ],
    socialLinks: [
      { icon: 'github', link: 'https://github.com/Hatiaa/auto-drone' }
    ],
    search: {
      provider: 'local'
    },
    footer: {
      message: 'Released under the MIT License.',
      copyright: 'Copyright © 2026 Hatiaa'
    }
  }
})
