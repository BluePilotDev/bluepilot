import { useEffect, useState } from 'react'
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom'
import { useWebSocketStore } from '@/stores/useWebSocketStore'
import { useToastStore } from '@/stores/useToastStore'
import { WarningBanners, StatusOverlay } from '@/components/common'
import { ToastContainer } from '@/components/common/Toast'
import { systemAPI } from '@/services/api'

// Views
import { Dashboard } from '@/views/Dashboard'
import { RoutesView } from '@/views/RoutesView'
import { ParametersView } from '@/views/ParametersView'
import { SettingsView } from '@/views/SettingsView'
import { LogsView } from '@/views/LogsView'

// Styles
import '@/styles/variables.css'
import '@/styles/App.css'

type DeviceStatus = 'online' | 'onroad' | 'offline' | 'checking'

function App() {
  const { connect, disconnect } = useWebSocketStore()
  const { toasts, removeToast } = useToastStore()
  const [deviceStatus, setDeviceStatus] = useState<DeviceStatus>('checking')

  useEffect(() => {
    // Connect to WebSocket on mount
    connect()

    // Check device status
    checkDeviceStatus()

    // Poll status every 30 seconds
    const statusInterval = setInterval(checkDeviceStatus, 30000)

    // Cleanup on unmount
    return () => {
      disconnect()
      clearInterval(statusInterval)
    }
  }, [connect, disconnect])

  const checkDeviceStatus = async () => {
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
      // If status check fails, assume offline
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
      {(deviceStatus === 'onroad' || deviceStatus === 'offline') && (
        <StatusOverlay type={deviceStatus} onRetry={handleRetryConnection} />
      )}
      <ToastContainer toasts={toasts} onRemove={removeToast} />
      <Routes>
        <Route path="/" element={<Dashboard deviceStatus={deviceStatus} />} />
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
