import { create } from 'zustand'
import { getWebSocketService } from '@/services/websocket'
import type { WebSocketMessage } from '@/types'
import { useRoutesStore } from './useRoutesStore'
import { useParamsStore } from './useParamsStore'
import { useSystemStore } from './useSystemStore'
import { useExportStore } from './useExportStore'
import { useToastStore } from './useToastStore'

interface WebSocketState {
  connected: boolean
  connecting: boolean

  // Actions
  connect: () => void
  disconnect: () => void
  handleMessage: (message: WebSocketMessage) => void
}

export const useWebSocketStore = create<WebSocketState>((set) => {
  const ws = getWebSocketService()

  return {
    connected: false,
    connecting: false,

    connect: () => {
      set({ connecting: true })

      // Subscribe to WebSocket messages
      ws.subscribe((message: WebSocketMessage) => {
        useWebSocketStore.getState().handleMessage(message)
      })

      // Connect
      ws.connect()

      // Update connection status after a brief delay
      setTimeout(() => {
        set({
          connected: ws.isConnected(),
          connecting: false,
        })
      }, 1000)
    },

    disconnect: () => {
      ws.disconnect()
      set({ connected: false })
    },

    handleMessage: (message: WebSocketMessage) => {
      const { type, data } = message

      switch (type) {
        case 'connection':
          set({ connected: (data as { connected: boolean }).connected })
          break

        case 'route_updated':
        case 'route_deleted':
        case 'route_added':
          // Refresh routes
          useRoutesStore.getState().fetchRoutes(1)
          break

        case 'param_changed':
          // Update specific parameter
          if (data && typeof data === 'object' && 'key' in data && 'value' in data) {
            const { key, value } = data as { key: string; value: unknown }
            const paramsStore = useParamsStore.getState()
            if (paramsStore.params[key]) {
              paramsStore.params[key].value = value as string | number | boolean
            }
          }
          break

        case 'export_progress':
          // Handle export progress updates
          if (data && typeof data === 'object' && 'route_base' in data) {
            const progressData = data as {
              type: 'videos_zip' | 'backup'
              route_base: string
              progress: number
              message: string
              status: string
            }
            console.log('Export/backup progress:', progressData)

            useExportStore.getState().updateProgress(progressData.route_base, {
              routeId: progressData.route_base,
              type: progressData.type,
              status: 'processing',
              progress: progressData.progress,
              message: progressData.message
            })
          }
          break

        case 'export_complete':
          // Handle export completion
          if (data && typeof data === 'object' && 'route_base' in data) {
            const completeData = data as {
              type: 'videos_zip' | 'backup'
              route_base: string
              status: string
              message: string
            }
            console.log('Export complete:', completeData)

            useExportStore.getState().setComplete(completeData.route_base, completeData.type)
            useToastStore.getState().addToast(
              completeData.message || 'Export completed successfully',
              'success'
            )
          }
          break

        case 'export_error':
          // Handle export errors
          if (data && typeof data === 'object' && 'route_base' in data) {
            const errorData = data as {
              type: 'videos_zip' | 'backup'
              route_base: string
              error: string
            }
            console.error('Export error:', errorData)

            useExportStore.getState().setError(
              errorData.route_base,
              errorData.type,
              errorData.error
            )
            useToastStore.getState().addToast(
              errorData.error || 'Export failed',
              'error'
            )
          }
          break

        case 'system_update':
          // Refresh system metrics
          useSystemStore.getState().fetchMetrics()
          break

        case 'log_line':
        case 'log_stream_status':
          // Diagnostics panel manages its own WebSocket feed; ignore to avoid noise
          break

        case 'heartbeat':
          // Respond to heartbeat with pong to keep connection alive
          const ws = getWebSocketService()
          ws.send({ type: 'pong', data: { timestamp: new Date().toISOString() } })
          break

        default:
          console.log('Unhandled WebSocket message:', type, data)
      }
    },
  }
})
