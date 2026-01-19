import axios, { AxiosError } from 'axios'
import type {
  Route,
  RouteDetails,
  Parameter,
  SystemMetrics,
  ServerStatus,
  DiskSpace,
  VehicleInfo,
  DeviceInfo,
} from '@/types'
import { useToastStore } from '@/stores/useToastStore'

// Rate limit tracking to avoid toast spam
let lastRateLimitToast = 0
const RATE_LIMIT_TOAST_COOLDOWN = 5000 // 5 seconds between toasts

// Create axios instance
const api = axios.create({
  baseURL: '',
  timeout: 30000,
  headers: {
    'Content-Type': 'application/json',
  },
})

// Request interceptor
api.interceptors.request.use(
  (config) => {
    return config
  },
  (error) => {
    return Promise.reject(error)
  }
)

// Response interceptor
api.interceptors.response.use(
  (response) => response,
  (error: AxiosError) => {
    // Handle rate limiting (429)
    if (error.response?.status === 429) {
      const now = Date.now()
      const data = error.response.data as {
        retry_after_seconds?: number
        reason?: string
        limit?: string
      }

      // Only show toast if cooldown has passed (avoid spam)
      if (now - lastRateLimitToast > RATE_LIMIT_TOAST_COOLDOWN) {
        lastRateLimitToast = now
        const retryAfter = data.retry_after_seconds || 1
        const isOnroad = data.reason === 'onroad_protection'

        const message = isOnroad
          ? `Rate limited while driving. Retry in ${retryAfter}s`
          : `Too many requests. Retry in ${retryAfter}s`

        useToastStore.getState().addToast(message, 'error', 3000)
      }

      console.warn('Rate limited:', data)
      return Promise.reject(error)
    }

    console.error('API Error:', error.message)
    // Log the full error response for debugging
    if (error.response) {
      console.error('Response status:', error.response.status)
      console.error('Response data:', error.response.data)
    }
    return Promise.reject(error)
  }
)

// Routes API
export const routesAPI = {
  getAll: async (page = 1, limit = 50): Promise<Route[]> => {
    const { data } = await api.get('/api/routes', { params: { page, limit } })
    return data
  },

  getOne: async (routeId: string): Promise<RouteDetails> => {
    const { data } = await api.get(`/api/routes/${routeId}`)
    return data
  },

  delete: async (routeId: string): Promise<void> => {
    await api.delete(`/api/delete/${routeId}`)
  },

  preserve: async (routeId: string): Promise<void> => {
    await api.post(`/api/preserve/${routeId}`)
  },

  clearCache: async (): Promise<void> => {
    await api.post('/api/clear-cache')
  },
}

// Video API
export const videoAPI = {
  getStreamUrl: (route: string, segment: number, camera: string): string => {
    return `/api/video/${route}/${segment}/${camera}`
  },
}

// Export/Backup API
export const exportAPI = {
  createVideosZip: async (routeId: string, cameras: string[]): Promise<void> => {
    await api.post(`/api/videos-zip/${routeId}`, { cameras })
  },

  getVideosZipStatus: async (routeId: string): Promise<{
    status: string
    progress: number
    message?: string
  }> => {
    const { data } = await api.get(`/api/videos-zip/${routeId}/status`)
    return data
  },

  downloadVideosZip: (routeId: string): string => {
    return `/api/videos-zip/${routeId}/download`
  },

  createRouteBackup: async (routeId: string): Promise<void> => {
    await api.post(`/api/route-backup/${routeId}`)
  },

  getRouteBackupStatus: async (routeId: string): Promise<{
    status: string
    progress: number
    message?: string
  }> => {
    const { data } = await api.get(`/api/route-backup/${routeId}/status`)
    return data
  },

  downloadRouteBackup: (routeId: string): string => {
    return `/api/route-backup/${routeId}/download`
  },

  importRouteBackup: async (file: File): Promise<void> => {
    const formData = new FormData()
    formData.append('backup', file)
    await api.post('/api/route-import', formData, {
      headers: {
        'Content-Type': 'multipart/form-data',
      },
    })
  },
}

// Parameters API
export const paramsAPI = {
  getAll: async (): Promise<Record<string, Parameter>> => {
    const { data } = await api.get('/api/params')
    // Handle response format: { success: true, params: {...} }
    if (data.success && data.params) {
      return data.params
    }
    // Fallback to direct params object
    return data
  },

  update: async (key: string, value: string | number | boolean): Promise<void> => {
    console.log('[API] Updating param:', { key, value, valueType: typeof value })
    await api.post('/api/params/set', { key, value })
  },
}

// System API
export const systemAPI = {
  getStatus: async (): Promise<ServerStatus> => {
    const { data} = await api.get('/api/status')
    return data
  },

  getDetailedStatus: async (): Promise<{
    connection: string
    rate_limit: string
    last_update: string
  }> => {
    const { data } = await api.get('/api/status/detailed')
    return data
  },

  getMetrics: async (): Promise<SystemMetrics> => {
    const { data } = await api.get('/api/system/metrics')
    return data
  },

  getDiskSpace: async (): Promise<DiskSpace> => {
    const { data } = await api.get('/api/disk-space')
    return data
  },

  getDiskAnalysis: async (): Promise<any> => {
    const { data } = await api.get('/api/disk-analysis')
    return data
  },

  getVehicleInfo: async (): Promise<VehicleInfo> => {
    const { data } = await api.get('/api/vehicle-info')
    return data
  },

  getDeviceInfo: async (): Promise<DeviceInfo> => {
    const { data } = await api.get('/api/system/device-info')
    return data
  },
}

export default api
