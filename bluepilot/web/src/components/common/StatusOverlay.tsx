import { useEffect, useState } from 'react'
import { systemAPI } from '@/services/api'
import { Button } from './Button'
import { Icon } from './Icon'
import './StatusOverlay.css'

interface StatusOverlayProps {
  type: 'onroad' | 'offline' | 'no-network'
  onRetry?: () => void
}

interface DetailedStatus {
  connection: string
  rateLimit: string
  lastUpdate: string
}

export const StatusOverlay = ({ type, onRetry }: StatusOverlayProps) => {
  const [details, setDetails] = useState<DetailedStatus | null>(null)
  const [loading, setLoading] = useState(false)

  useEffect(() => {
    loadDetailedStatus()
  }, [type])

  const loadDetailedStatus = async () => {
    try {
      const response = await systemAPI.getDetailedStatus()
      setDetails({
        connection: response.connection || 'Unknown',
        rateLimit: response.rate_limit || 'Normal',
        lastUpdate: response.last_update || new Date().toLocaleTimeString(),
      })
    } catch (error) {
      console.error('Failed to load detailed status:', error)
    }
  }

  const handleRetry = async () => {
    setLoading(true)
    try {
      await new Promise(resolve => setTimeout(resolve, 1000))
      if (onRetry) onRetry()
    } finally {
      setLoading(false)
    }
  }

  const configs = {
    onroad: {
      icon: <Icon name="schedule" size={80} />,
      title: 'Device Onroad',
      message: 'The device is currently driving. Web access is limited to prevent distractions.',
    },
    offline: {
      icon: <Icon name="cloud_off" size={80} />,
      title: 'Device Offline',
      message: 'Cannot connect to the device. Make sure you\'re on the same network as your comma device.',
    },
    'no-network': {
      icon: <Icon name="wifi_off" size={80} />,
      title: 'No Network Connection',
      message: 'Your device appears to be offline. Please check your internet connection.',
    },
  }
  const config = configs[type]

  return (
    <div className={`status-overlay status-${type}`}>
      <div className="status-overlay-content">
        <div className="status-icon">
          {config.icon}
        </div>
        <h2>{config.title}</h2>
        <p>{config.message}</p>

        {details && (
          <div className="status-details">
            <div className="status-detail-item">
              <span className="detail-label">Connection:</span>
              <span className="detail-value">{details.connection}</span>
            </div>
            <div className="status-detail-item">
              <span className="detail-label">Rate Limit:</span>
              <span className="detail-value">{details.rateLimit}</span>
            </div>
            <div className="status-detail-item">
              <span className="detail-label">Last Update:</span>
              <span className="detail-value">{details.lastUpdate}</span>
            </div>
          </div>
        )}

        {(type === 'offline' || type === 'no-network') && (
          <div className="status-actions">
            <Button
              variant="primary"
              size="large"
              onClick={handleRetry}
              loading={loading}
            >
              Retry Connection
            </Button>
          </div>
        )}
      </div>
    </div>
  )
}
