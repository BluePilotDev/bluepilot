/**
 * Settings Context
 * Provides panel context information to all nested control components
 */

import { createContext, useContext, type ReactNode } from 'react'

interface SettingsContextValue {
  panelId: string
  panelName: string
  groupName: string
}

const SettingsContext = createContext<SettingsContextValue | null>(null)

interface SettingsProviderProps {
  panelId: string
  panelName: string
  groupName: string
  children: ReactNode
}

export function SettingsProvider({ panelId, panelName, groupName, children }: SettingsProviderProps) {
  return (
    <SettingsContext.Provider value={{ panelId, panelName, groupName }}>
      {children}
    </SettingsContext.Provider>
  )
}

export function useSettingsContext(): SettingsContextValue | null {
  return useContext(SettingsContext)
}

export function useRequiredSettingsContext(): SettingsContextValue {
  const context = useContext(SettingsContext)
  if (!context) {
    throw new Error('useRequiredSettingsContext must be used within a SettingsProvider')
  }
  return context
}
