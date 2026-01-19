import { create } from 'zustand'

export type ExportType = 'videos_zip' | 'backup' | 'qlog' | 'rlog'

export interface ExportProgress {
  routeId: string
  type: ExportType
  status: 'processing' | 'complete' | 'error'
  progress: number
  message: string
  error?: string
  totalFiles?: number
  filesProcessed?: number
}

interface ExportState {
  activeExports: Map<string, ExportProgress>

  // Actions
  updateProgress: (routeId: string, progress: ExportProgress) => void
  setComplete: (routeId: string, type: ExportType) => void
  setError: (routeId: string, type: ExportType, error: string) => void
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

  setComplete: (routeId: string, type: ExportType) => {
    set((state) => {
      const newMap = new Map(state.activeExports)
      const existing = newMap.get(routeId)
      if (existing && existing.type === type) {
        const messages: Record<ExportType, string> = {
          'videos_zip': 'Videos ready for download',
          'backup': 'Backup ready for download',
          'qlog': 'qlog ready for download',
          'rlog': 'rlog ready for download'
        }
        newMap.set(routeId, {
          ...existing,
          status: 'complete',
          progress: 100,
          message: messages[type]
        })
      }
      return { activeExports: newMap }
    })
  },

  setError: (routeId: string, type: ExportType, error: string) => {
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
