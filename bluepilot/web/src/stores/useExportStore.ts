import { create } from 'zustand'

export interface ExportProgress {
  routeId: string
  type: 'videos_zip' | 'backup'
  status: 'processing' | 'complete' | 'error'
  progress: number
  message: string
  error?: string
}

interface ExportState {
  activeExports: Map<string, ExportProgress>

  // Actions
  updateProgress: (routeId: string, progress: ExportProgress) => void
  setComplete: (routeId: string, type: 'videos_zip' | 'backup') => void
  setError: (routeId: string, type: 'videos_zip' | 'backup', error: string) => void
  clearExport: (routeId: string) => void
  getProgress: (routeId: string) => ExportProgress | undefined
}

export const useExportStore = create<ExportState>((set, get) => ({
  activeExports: new Map(),

  updateProgress: (routeId: string, progress: ExportProgress) => {
    set((state) => {
      const newMap = new Map(state.activeExports)
      newMap.set(routeId, progress)
      return { activeExports: newMap }
    })
  },

  setComplete: (routeId: string, type: 'videos_zip' | 'backup') => {
    set((state) => {
      const newMap = new Map(state.activeExports)
      const existing = newMap.get(routeId)
      if (existing && existing.type === type) {
        newMap.set(routeId, {
          ...existing,
          status: 'complete',
          progress: 100,
          message: type === 'videos_zip' ? 'Videos ready for download' : 'Backup ready for download'
        })
      }
      return { activeExports: newMap }
    })
  },

  setError: (routeId: string, type: 'videos_zip' | 'backup', error: string) => {
    set((state) => {
      const newMap = new Map(state.activeExports)
      const existing = newMap.get(routeId)
      if (existing && existing.type === type) {
        newMap.set(routeId, {
          ...existing,
          status: 'error',
          error,
          message: error
        })
      }
      return { activeExports: newMap }
    })
  },

  clearExport: (routeId: string) => {
    set((state) => {
      const newMap = new Map(state.activeExports)
      newMap.delete(routeId)
      return { activeExports: newMap }
    })
  },

  getProgress: (routeId: string) => {
    return get().activeExports.get(routeId)
  }
}))
