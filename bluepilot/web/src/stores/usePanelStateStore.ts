/**
 * Panel State Store
 * Manages device state for panel conditionals (onroad, car params, etc.)
 */

import { create } from 'zustand'
import type { PanelState } from '@/types/panels'
import { panelAPI } from '@/services/panelAPI'

interface PanelStateStore {
  // State
  state: PanelState
  loading: boolean
  error: string | null

  // Actions
  fetchState: () => Promise<void>
  updateState: (partialState: Partial<PanelState>) => void
  clearError: () => void
}

// Default state when no car params available
const defaultState: PanelState = {
  isOnroad: false,
  isOffroad: true,
  hasCarParams: false,
}

export const usePanelStateStore = create<PanelStateStore>((set) => ({
  // Initial state
  state: defaultState,
  loading: false,
  error: null,

  // Fetch current device state
  fetchState: async () => {
    set({ loading: true, error: null })
    try {
      const response = await panelAPI.getPanelState()
      if (response.success && response.state) {
        set({ state: response.state, loading: false })
      } else {
        set({ error: 'Failed to load panel state', loading: false })
      }
    } catch (error) {
      console.error('Error fetching panel state:', error)
      set({
        error: error instanceof Error ? error.message : 'Failed to load panel state',
        loading: false,
      })
    }
  },

  // Update state (for WebSocket updates)
  updateState: (partialState: Partial<PanelState>) => {
    set((state) => ({
      state: {
        ...state.state,
        ...partialState,
      },
    }))
  },

  // Clear error
  clearError: () => set({ error: null }),
}))
