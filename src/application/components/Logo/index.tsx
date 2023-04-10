import React from 'react'
import { useDarkMode } from '@electricui/components-desktop'

import logo from './logo.png'
import logoOrange from './logo-orange.png'

export const Logo = () => {
  const isDark = useDarkMode()

  if (isDark) {
    return (
      <img
        src={logo}
        style={{
          maxWidth: 500,
          display: 'block',
          margin: '0 auto',
          paddingTop: '10vh',
        }}
      />
    )
  }

  return (
    <img
      src={logo}
      style={{
        maxWidth: 500,
        display: 'block',
        margin: '0 auto',
        paddingTop: '10vh',
      }}
    />
  )
}
