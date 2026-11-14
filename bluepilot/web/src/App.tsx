import { useEffect, useState } from 'react'
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom'
import { useWebSocketStore } from '@/stores/useWebSocketStore'
import { useSystemStore } from '@/stores/useSystemStore'
import { useToastStore } from '@/stores/useToastStore'
import { WarningBanners, StatusOverlay } from '@/components/common'
import { ToastContainer } from '@/components/common/Toast'
import { systemAPI } from '@/services/api'
import type { DeviceStatus } from '@/types'

// Views
import { Home } from '@/views/Home'
import { RoutesView } from '@/views/RoutesView'
import { ParametersView } from '@/views/ParametersView'
import { SettingsView } from '@/views/SettingsView'
import { LogsView } from '@/views/LogsView'

// Styles
import '@/styles/variables.css'
import '@/styles/App.css'

function App() {
  const { connect, disconnect } = useWebSocketStore()
  const { startPolling, stopPolling } = useSystemStore()
  const { toasts, removeToast } = useToastStore()
  const [deviceStatus, setDeviceStatus] = useState<DeviceStatus>('checking')

  useEffect(() => {
    // Connect to WebSocket on mount
    connect()

    // Check device status
    checkDeviceStatus()

    // Start polling for system metrics (every 30 seconds to avoid rate limiting)
    startPolling(30000)

    // Poll status every 60 seconds
    const statusInterval = setInterval(checkDeviceStatus, 60000)

    // Listen for online/offline events
    const handleOnline = () => checkDeviceStatus()
    const handleOffline = () => setDeviceStatus('no-network')
    window.addEventListener('online', handleOnline)
    window.addEventListener('offline', handleOffline)

    // Cleanup on unmount
    return () => {
      disconnect()
      stopPolling()
      clearInterval(statusInterval)
      window.removeEventListener('online', handleOnline)
      window.removeEventListener('offline', handleOffline)
    }
  }, [connect, disconnect, startPolling, stopPolling])

  const checkDeviceStatus = async () => {
    // Check if browser is offline first
    if (!navigator.onLine) {
      setDeviceStatus('no-network')
      return
    }

    try {
      const controller = new AbortController()
      const timeoutId = setTimeout(() => controller.abort(), 3000)

      const status = await systemAPI.getStatus()
      clearTimeout(timeoutId)

      if (status.onroad) {
        setDeviceStatus('onroad')
      } else if (status.online !== false) {
        setDeviceStatus('online')
      } else {
        setDeviceStatus('offline')
      }
    } catch (error) {
      // If status check fails, assume device offline (but network is available)
      setDeviceStatus('offline')
    }
  }

  const handleRetryConnection = () => {
    setDeviceStatus('checking')
    checkDeviceStatus()
  }

  return (
    <BrowserRouter>
      <WarningBanners />
      {(deviceStatus === 'offline' || deviceStatus === 'no-network') && (
        <StatusOverlay type={deviceStatus} onRetry={handleRetryConnection} />
      )}
      <ToastContainer toasts={toasts} onRemove={removeToast} />
      <Routes>
        <Route path="/" element={<Home deviceStatus={deviceStatus} />} />
        <Route path="/settings" element={<SettingsView deviceStatus={deviceStatus} />} />
        <Route path="/routes" element={<RoutesView deviceStatus={deviceStatus} />} />
        <Route path="/parameters" element={<ParametersView deviceStatus={deviceStatus} />} />
        <Route path="/logs" element={<LogsView deviceStatus={deviceStatus} />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Routes>
    </BrowserRouter>
  )
}

export default App
