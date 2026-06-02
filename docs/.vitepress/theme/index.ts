import DefaultTheme from 'vitepress/theme'
import { h } from 'vue'
import AutoDroneFooter from './AutoDroneFooter.vue'

export default {
  extends: DefaultTheme,
  Layout() {
    return h(DefaultTheme.Layout, null, {
      'layout-bottom': () => h(AutoDroneFooter)
    })
  }
}

