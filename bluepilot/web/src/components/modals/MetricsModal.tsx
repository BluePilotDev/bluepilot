import { useEffect, useState } from 'react'
import { systemAPI } from '@/services/api'
import { Modal } from '@/components/common'
import type { SystemMetrics } from '@/types'
import './MetricsModal.css'

interface MetricsModalProps {
  isOpen: boolean
  onClose: () => void
}

export const MetricsModal = ({ isOpen, onClose }: MetricsModalProps) => {
  const [metrics, setMetrics] = useState<SystemMetrics | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    if (isOpen) {
      loadMetrics()
      // Refresh metrics every 5 seconds while modal is open
      const interval = setInterval(loadMetrics, 5000)
      return () => clearInterval(interval)
    }
  }, [isOpen])

  const loadMetrics = async () => {
    try {
      const data = await systemAPI.getMetrics()
      setMetrics(data)
      setLoading(false)
    } catch (error) {
      console.error('Failed to load metrics:', error)
      setLoading(false)
    }
  }

  // Helper to determine temperature status
  const getTempStatus = (temp?: number): 'normal' | 'warning' | 'critical' => {
    if (!temp) return 'normal'
    if (temp >= 80) return 'critical'
    if (temp >= 70) return 'warning'
    return 'normal'
  }

  // Helper to convert bytes to GB
  const bytesToGB = (bytes: number): number => bytes / (1024 ** 3)

  if (!isOpen) return null

  const tempStatus = getTempStatus(metrics?.temperature)

  return (
    <Modal isOpen={isOpen} onClose={onClose} title="System Health Metrics">
      {loading && !metrics ? (
        <div className="metrics-loading">Loading metrics...</div>
      ) : (
        <div className="metrics-grid">
          {/* CPU Card */}
          <div className="metric-card">
            <h3>CPU</h3>
            <div className="metric-value">
              <span className="value-large">{metrics?.cpu_load?.toFixed(2) ?? '--'}</span>
              <span className="unit">load</span>
            </div>
            <div className="metric-details">
              <div className="metric-row">
                <span>Cores:</span>
                <span>{metrics?.cpu_cores ?? '--'}</span>
              </div>
              <div className="metric-row">
                <span>FFmpeg:</span>
                <span>{metrics?.ffmpeg_processes ?? 0} processes</span>
              </div>
            </div>
          </div>

          {/* Memory Card */}
          <div className="metric-card">
            <h3>Memory</h3>
            <div className="metric-value">
              <span className="value-large">{metrics?.memory_percent?.toFixed(1) ?? '--'}</span>
              <span className="unit">%</span>
            </div>
            <div className="metric-bar">
              <div
                className="metric-bar-fill"
                style={{ width: `${metrics?.memory_percent ?? 0}%` }}
              />
            </div>
            <div className="metric-details">
              <div className="metric-row">
                <span>Used:</span>
                <span>{metrics ? bytesToGB(metrics.memory_used).toFixed(1) : '--'} GB</span>
              </div>
              <div className="metric-row">
                <span>Total:</span>
                <span>{metrics ? bytesToGB(metrics.memory_total).toFixed(1) : '--'} GB</span>
              </div>
            </div>
          </div>

          {/* Disk Card */}
          <div className="metric-card">
            <h3>Disk (/data)</h3>
            <div className="metric-value">
              <span className="value-large">{metrics?.disk_percent?.toFixed(1) ?? '--'}</span>
              <span className="unit">%</span>
            </div>
            <div className="metric-bar">
              <div
                className="metric-bar-fill"
                style={{ width: `${metrics?.disk_percent ?? 0}%` }}
              />
            </div>
            <div className="metric-details">
              <div className="metric-row">
                <span>Used:</span>
                <span>{metrics ? bytesToGB(metrics.disk_used).toFixed(1) : '--'} GB</span>
              </div>
              <div className="metric-row">
                <span>Total:</span>
                <span>{metrics ? bytesToGB(metrics.disk_total).toFixed(1) : '--'} GB</span>
              </div>
            </div>
          </div>

          {/* Temperature Card */}
          <div className="metric-card">
            <h3>Temperature</h3>
            <div className="metric-value">
              <span className={`value-large temp-${tempStatus}`}>
                {metrics?.temperature ?? '--'}
              </span>
              <span className="unit">°C</span>
            </div>
            <div className="metric-details">
              <div className="metric-row">
                <span>Status:</span>
                <span className={`status-${tempStatus}`}>
                  {tempStatus}
                </span>
              </div>
              {metrics?.cache_size !== undefined && (
                <div className="metric-row">
                  <span>Cache:</span>
                  <span>{bytesToGB(metrics.cache_size).toFixed(1)} GB</span>
                </div>
              )}
            </div>
          </div>
        </div>
      )}
    </Modal>
  )
}
