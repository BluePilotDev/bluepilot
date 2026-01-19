/**
 * Change Tracking Store
 * Manages pending settings changes for staged save workflow
 */

import { create } from 'zustand'
import { paramsAPI } from '@/services/api'

export interface PendingChange {
  param: string
  oldValue: string | number | boolean | null
  newValue: string | number | boolean
  controlTitle: string
  controlType: 'toggle' | 'selection' | 'segmented_control' | 'integer' | 'float'
  panelId: string
  panelName: string
  groupName: string
  timestamp: number
}

interface ChangeTrackingState {
  // State
  pendingChanges: Map<string, PendingChange>
  isSaving: boolean
  saveError: string | null
  autoSaveEnabled: boolean // Session-only auto-save mode

  // Actions
  recordChange: (change: Omit<PendingChange, 'timestamp'>) => void
  discardChange: (param: string) => void
  discardAllChanges: () => void
  commitAllChanges: () => Promise<{ success: boolean; error?: string }>
  getChangeSummary: () => PendingChange[]
  getChangeCount: () => number
  hasChanges: () => boolean
  getChangeForParam: (param: string) => PendingChange | undefined
  clearSaveError: () => void
  setAutoSave: (enabled: boolean) => void
  isAutoSaveEnabled: () => boolean
}

export const useChangeTrackingStore = create<ChangeTrackingState>((set, get) => ({
  pendingChanges: new Map(),
  isSaving: false,
  saveError: null,
  autoSaveEnabled: false,

  recordChange: (change) => {
    set((state) => {
      const newChanges = new Map(state.pendingChanges)
      const existingChange = newChanges.get(change.param)

      // If this change reverts back to the original value, remove it from pending
      if (existingChange && String(existingChange.oldValue) === String(change.newValue)) {
        newChanges.delete(change.param)
      } else {
        // Keep the original oldValue if we already have a pending change for this param
        const oldValue = existingChange ? existingChange.oldValue : change.oldValue

        newChanges.set(change.param, {
          ...change,
          oldValue,
          timestamp: Date.now(),
        })
      }

      return { pendingChanges: newChanges }
    })
  },

  discardChange: (param) => {
    set((state) => {
      const newChanges = new Map(state.pendingChanges)
      newChanges.delete(param)
      return { pendingChanges: newChanges }
    })
  },

  discardAllChanges: () => {
    set({ pendingChanges: new Map(), saveError: null })
  },

  commitAllChanges: async () => {
    const { pendingChanges } = get()

    if (pendingChanges.size === 0) {
      return { success: true }
    }

    set({ isSaving: true, saveError: null })

    try {
      // Convert Map to array and save all changes
      const changes = Array.from(pendingChanges.values())
      const results = await Promise.allSettled(
        changes.map((change) => paramsAPI.update(change.param, change.newValue))
      )

      // Check for any failures
      const failures = results.filter((r) => r.status === 'rejected')

      if (failures.length > 0) {
        const errorMessage = `Failed to save ${failures.length} of ${changes.length} changes`
        set({ isSaving: false, saveError: errorMessage })
        return { success: false, error: errorMessage }
      }

      // All successful - clear pending changes
      set({ pendingChanges: new Map(), isSaving: false, saveError: null })
      return { success: true }
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Failed to save changes'
      set({ isSaving: false, saveError: errorMessage })
      return { success: false, error: errorMessage }
    }
  },

  getChangeSummary: () => {
    const { pendingChanges } = get()
    return Array.from(pendingChanges.values()).sort((a, b) => a.timestamp - b.timestamp)
  },

  getChangeCount: () => {
    return get().pendingChanges.size
  },

  hasChanges: () => {
    return get().pendingChanges.size > 0
  },

  getChangeForParam: (param) => {
    return get().pendingChanges.get(param)
  },

  clearSaveError: () => {
    set({ saveError: null })
  },

  setAutoSave: (enabled: boolean) => {
    set({ autoSaveEnabled: enabled })
  },

  isAutoSaveEnabled: () => {
    return get().autoSaveEnabled
  },
}))
