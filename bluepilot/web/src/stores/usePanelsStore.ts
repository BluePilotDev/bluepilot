/**
 * Panels Store
 * Manages panel list and configurations
 */

import { create } from 'zustand'
import type { PanelMetadata, PanelConfig } from '@/types/panels'
import { panelAPI } from '@/services/panelAPI'

interface PanelsState {
  // State
  panels: PanelMetadata[]
  loadedPanels: Record<string, PanelConfig>
  loading: boolean
  error: string | null

  // Actions
  fetchPanels: () => Promise<void>
  fetchPanel: (panelId: string) => Promise<void>
  clearError: () => void
}

export const usePanelsStore = create<PanelsState>((set, get) => ({
  // Initial state
  panels: [],
  loadedPanels: {},
  loading: false,
  error: null,

  // Fetch list of all panels
  fetchPanels: async () => {
    set({ loading: true, error: null })
    try {
      const response = await panelAPI.getPanels()
      if (response.success) {
        set({ panels: response.panels, loading: false })
      } else {
        set({ error: 'Failed to load panels', loading: false })
      }
    } catch (error) {
      console.error('Error fetching panels:', error)
      set({
        error: error instanceof Error ? error.message : 'Failed to load panels',
        loading: false,
      })
    }
  },

  // Fetch specific panel configuration
  fetchPanel: async (panelId: string) => {
    // Don't re-fetch if already loaded
    if (get().loadedPanels[panelId]) {
      return
    }

    set({ loading: true, error: null })
    try {
      const response = await panelAPI.getPanel(panelId)
      if (response.success && response.panel) {
        set((state) => ({
          loadedPanels: {
            ...state.loadedPanels,
            [panelId]: response.panel,
          },
          loading: false,
        }))
      } else {
        set({ error: `Failed to load panel: ${panelId}`, loading: false })
      }
    } catch (error) {
      console.error(`Error fetching panel ${panelId}:`, error)
      set({
        error: error instanceof Error ? error.message : `Failed to load panel: ${panelId}`,
        loading: false,
      })
    }
  },

  // Clear error
  clearError: () => set({ error: null }),
}))
