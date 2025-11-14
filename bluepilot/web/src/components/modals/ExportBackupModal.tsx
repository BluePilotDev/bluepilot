import { useEffect, useState } from 'react'
import { Modal } from '@/components/common'
import { exportAPI } from '@/services/api'
import { useToastStore } from '@/stores/useToastStore'
import { useExportStore } from '@/stores/useExportStore'
import type { RouteDetails } from '@/types'
import './ExportBackupModal.css'

interface ExportBackupModalProps {
  isOpen: boolean
  onClose: () => void
  route: RouteDetails | null
}

interface CameraSizes {
  front?: number
  wide?: number
  driver?: number
  lq?: number
}

interface ExportStatus {
  visible: boolean
  message: string
  progress: number
  type: 'video' | 'backup'
}

export const ExportBackupModal = ({ isOpen, onClose, route }: ExportBackupModalProps) => {
  const { addToast } = useToastStore()
  const exportStore = useExportStore()
  const [selectedCameras, setSelectedCameras] = useState<Set<string>>(new Set(['front']))
  const [cameraSizes, setCameraSizes] = useState<CameraSizes>({})
  const [exportStatus, setExportStatus] = useState<ExportStatus | null>(null)
  const [loading, setLoading] = useState(false)

  useEffect(() => {
    if (isOpen && route) {
      loadCameraSizes()
      // Reset state
      setSelectedCameras(new Set(['front']))
      setExportStatus(null)
    }
  }, [isOpen, route])

  // Listen to export store for WebSocket updates
  useEffect(() => {
    if (!route || !isOpen) return

    const routeId = route.baseName || route.id || ''
    if (!routeId) return

    // Check for updates from WebSocket every 100ms
    const interval = setInterval(() => {
      const progress = exportStore.getProgress(routeId)
      if (progress) {
        // Update the export status based on WebSocket data
        const statusType: 'video' | 'backup' = progress.type === 'videos_zip' ? 'video' : 'backup'

        setExportStatus({
          visible: true,
          message: progress.message,
          progress: progress.progress * 100, // Convert to percentage
          type: statusType
        })

        // Handle completion
        if (progress.status === 'complete') {
          setLoading(false)

          // Trigger download
          if (progress.type === 'videos_zip') {
            const downloadUrl = exportAPI.downloadVideosZip(routeId)
            console.log('Downloading from:', downloadUrl)
            window.open(downloadUrl, '_blank')
          } else if (progress.type === 'backup') {
            const downloadUrl = exportAPI.downloadRouteBackup(routeId)
            console.log('Downloading backup from:', downloadUrl)
            window.open(downloadUrl, '_blank')
          }

          // Clear the export from store after a delay
          setTimeout(() => {
            exportStore.clearExport(routeId)
            setExportStatus(null)
          }, 3000)
        }

        // Handle errors
        if (progress.status === 'error') {
          setLoading(false)
          setExportStatus({
            visible: true,
            message: progress.error || 'Export failed',
            progress: 0,
            type: statusType
          })

          // Clear error after showing
          setTimeout(() => {
            exportStore.clearExport(routeId)
          }, 5000)
        }
      }
    }, 100)

    return () => clearInterval(interval)
  }, [isOpen, route, exportStore])

  const loadCameraSizes = async () => {
    if (!route) return
    try {
      const response = await fetch(`/api/route/${route.baseName}/camera-sizes`)
      if (response.ok) {
        const data = await response.json()
        setCameraSizes(data)
      }
    } catch (error) {
      console.error('Error loading camera sizes:', error)
    }
  }

  const formatSize = (bytes?: number): string => {
    if (!bytes) return '--'
    const mb = bytes / (1024 * 1024)
    if (mb < 1024) return `${mb.toFixed(1)} MB`
    return `${(mb / 1024).toFixed(2)} GB`
  }

  const toggleCamera = (camera: string) => {
    const newSet = new Set(selectedCameras)
    if (newSet.has(camera)) {
      newSet.delete(camera)
    } else {
      newSet.add(camera)
    }
    setSelectedCameras(newSet)
  }

  const downloadSingleVideo = (camera: string) => {
    if (!route) return
    const url = `/api/video-export/${route.baseName}/${camera}`
    window.open(url, '_blank')
  }

  const downloadSelectedVideos = async () => {
    if (!route || selectedCameras.size === 0) {
      addToast('Please select at least one camera', 'error')
      return
    }

    const routeId = route.baseName || route.id || ''
    if (!routeId) {
      addToast('Invalid route ID', 'error')
      return
    }

    setLoading(true)
    setExportStatus({
      visible: true,
      message: 'Creating video archive...',
      progress: 0,
      type: 'video'
    })

    try {
      const cameras = Array.from(selectedCameras)
      console.log('Creating videos ZIP for route:', routeId, 'cameras:', cameras)
      await exportAPI.createVideosZip(routeId, cameras)

      // Poll for status
      const pollInterval = setInterval(async () => {
        try {
          const status = await exportAPI.getVideosZipStatus(routeId)
          console.log('Videos ZIP status:', status)

          setExportStatus({
            visible: true,
            message: status.message || 'Processing...',
            progress: status.progress,
            type: 'video'
          })

          if (status.status === 'complete') {
            clearInterval(pollInterval)
            setExportStatus({
              visible: true,
              message: 'Download ready!',
              progress: 100,
              type: 'video'
            })
            // Trigger download
            const downloadUrl = exportAPI.downloadVideosZip(routeId)
            console.log('Downloading from:', downloadUrl)
            window.open(downloadUrl, '_blank')
            setLoading(false)
            addToast('Video export started downloading', 'success')
          } else if (status.status === 'error') {
            clearInterval(pollInterval)
            setExportStatus({
              visible: true,
              message: 'Export failed',
              progress: 0,
              type: 'video'
            })
            setLoading(false)
          }
        } catch (error: any) {
          clearInterval(pollInterval)
          console.error('Status poll error:', error)
          addToast(error?.response?.data?.error || 'Failed to check export status', 'error')
          setLoading(false)
        }
      }, 1000)

      // Timeout after 10 minutes
      setTimeout(() => {
        clearInterval(pollInterval)
        setExportStatus({
          visible: true,
          message: 'Export timeout - operation took too long',
          progress: 0,
          type: 'video'
        })
        setLoading(false)
        addToast('Video export timed out', 'error')
      }, 600000)
    } catch (error: any) {
      console.error('Export error:', error)
      const errorMessage = error?.response?.data?.error || error?.message || 'Export failed'
      setExportStatus({
        visible: true,
        message: errorMessage,
        progress: 0,
        type: 'video'
      })
      addToast(errorMessage, 'error')
      setLoading(false)
    }
  }

  const createBackup = async () => {
    if (!route) {
      addToast('No route selected', 'error')
      return
    }

    const routeId = route.baseName || route.id || ''
    if (!routeId) {
      addToast('Invalid route ID', 'error')
      return
    }

    setLoading(true)
    setExportStatus({
      visible: true,
      message: 'Creating route backup...',
      progress: 0,
      type: 'backup'
    })

    try {
      console.log('Creating backup for route:', routeId)
      await exportAPI.createRouteBackup(routeId)

      // Poll for status
      const pollInterval = setInterval(async () => {
        try {
          const status = await exportAPI.getRouteBackupStatus(routeId)
          console.log('Backup status:', status)

          setExportStatus({
            visible: true,
            message: status.message || 'Processing...',
            progress: status.progress,
            type: 'backup'
          })

          if (status.status === 'complete') {
            clearInterval(pollInterval)
            setExportStatus({
              visible: true,
              message: 'Backup ready! Download starting...',
              progress: 100,
              type: 'backup'
            })
            // Trigger download
            const downloadUrl = exportAPI.downloadRouteBackup(routeId)
            console.log('Downloading backup from:', downloadUrl)
            window.open(downloadUrl, '_blank')
            setLoading(false)
            addToast('Backup download started', 'success')
          } else if (status.status === 'error') {
            clearInterval(pollInterval)
            setExportStatus({
              visible: true,
              message: 'Backup failed',
              progress: 0,
              type: 'backup'
            })
            setLoading(false)
          }
        } catch (error: any) {
          clearInterval(pollInterval)
          console.error('Status poll error:', error)
          addToast(error?.response?.data?.error || 'Failed to check backup status', 'error')
          setLoading(false)
        }
      }, 1000)

      // Timeout after 10 minutes
      setTimeout(() => {
        clearInterval(pollInterval)
        setExportStatus({
          visible: true,
          message: 'Backup timeout - operation took too long',
          progress: 0,
          type: 'backup'
        })
        setLoading(false)
        addToast('Backup operation timed out', 'error')
      }, 600000)
    } catch (error: any) {
      console.error('Backup error:', error)
      const errorMessage = error?.response?.data?.error || error?.message || 'Backup failed'
      setExportStatus({
        visible: true,
        message: errorMessage,
        progress: 0,
        type: 'backup'
      })
      addToast(errorMessage, 'error')
      setLoading(false)
    }
  }

  if (!route) return null

  const segmentCount = route.segments?.length || 0
  const totalSize = (cameraSizes.front || 0) + (cameraSizes.wide || 0) +
                    (cameraSizes.driver || 0) + (cameraSizes.lq || 0)

  return (
    <Modal isOpen={isOpen} onClose={onClose} title="Export & Backup Route">
      <div className="export-modal-body">
        {/* Route Info */}
        <div className="export-route-info">
          <h3>Route: {route.baseName || route.date}</h3>
          <div className="export-route-meta">
            <span>{segmentCount} segments</span>
            <span>{route.size || formatSize(totalSize)}</span>
            <span>{route.duration || '--'}</span>
          </div>
        </div>

        {/* Video Downloads Section */}
        <div className="export-section">
          <h3 className="export-section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <polygon points="23 7 16 12 23 17 23 7" />
              <rect x="1" y="5" width="15" height="14" rx="2" ry="2" />
            </svg>
            Video Downloads
          </h3>
          <p className="export-section-description">
            Download individual camera videos or all cameras as a ZIP archive. Videos are exported as full-route MP4 files.
          </p>

          <div className="camera-checkboxes">
            <label className="camera-checkbox-label">
              <input
                type="checkbox"
                checked={selectedCameras.has('front')}
                onChange={() => toggleCamera('front')}
                className="camera-checkbox"
              />
              <span className="checkbox-custom"></span>
              <span className="camera-label">
                <strong>Front Camera <span className="camera-size">{formatSize(cameraSizes.front)}</span></strong>
                <span className="camera-details">HEVC, high quality</span>
              </span>
            </label>

            <label className="camera-checkbox-label">
              <input
                type="checkbox"
                checked={selectedCameras.has('wide')}
                onChange={() => toggleCamera('wide')}
                className="camera-checkbox"
              />
              <span className="checkbox-custom"></span>
              <span className="camera-label">
                <strong>Wide Camera <span className="camera-size">{formatSize(cameraSizes.wide)}</span></strong>
                <span className="camera-details">HEVC, wide angle</span>
              </span>
            </label>

            <label className="camera-checkbox-label">
              <input
                type="checkbox"
                checked={selectedCameras.has('driver')}
                onChange={() => toggleCamera('driver')}
                className="camera-checkbox"
              />
              <span className="checkbox-custom"></span>
              <span className="camera-label">
                <strong>Driver Camera <span className="camera-size">{formatSize(cameraSizes.driver)}</span></strong>
                <span className="camera-details">HEVC, driver monitoring</span>
              </span>
            </label>

            <label className="camera-checkbox-label">
              <input
                type="checkbox"
                checked={selectedCameras.has('lq')}
                onChange={() => toggleCamera('lq')}
                className="camera-checkbox"
              />
              <span className="checkbox-custom"></span>
              <span className="camera-label">
                <strong>LQ Camera <span className="camera-size">{formatSize(cameraSizes.lq)}</span></strong>
                <span className="camera-details">H.264, low quality</span>
              </span>
            </label>
          </div>

          <div className="export-actions">
            <button type="button" className="btn btn-sm btn-secondary" onClick={() => downloadSingleVideo('front')}>
              Download Front
            </button>
            <button type="button" className="btn btn-sm btn-secondary" onClick={() => downloadSingleVideo('wide')}>
              Download Wide
            </button>
            <button type="button" className="btn btn-sm btn-secondary" onClick={() => downloadSingleVideo('driver')}>
              Download Driver
            </button>
            <button type="button" className="btn btn-sm btn-secondary" onClick={() => downloadSingleVideo('lq')}>
              Download LQ
            </button>
            <button
              type="button"
              className="btn btn-primary"
              onClick={downloadSelectedVideos}
              disabled={selectedCameras.size === 0 || loading}
            >
              Download Selected Videos ({selectedCameras.size})
            </button>
          </div>

          {exportStatus?.visible && exportStatus.type === 'video' && (
            <div className="export-status">
              <div className="export-status-message">{exportStatus.message}</div>
              <div className="export-progress-bar">
                <div className="export-progress-fill" style={{ width: `${exportStatus.progress}%` }} />
              </div>
            </div>
          )}
        </div>

        {/* Full Route Backup Section */}
        <div className="export-section">
          <h3 className="export-section-title">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
              <polyline points="17 8 12 3 7 8" />
              <line x1="12" y1="3" x2="12" y2="15" />
            </svg>
            Full Route Backup
          </h3>
          <p className="export-section-description">
            Create a complete backup of this route including all videos, logs, GPS data, and metadata.
            This backup can be imported later to restore the route.
          </p>

          <div className="backup-info">
            <div className="backup-info-item">
              <span className="backup-info-label">Estimated size:</span>
              <span className="backup-info-value">{formatSize(totalSize)}</span>
            </div>
            <div className="backup-info-item">
              <span className="backup-info-label">Includes:</span>
              <span className="backup-info-value">Videos, GPS, Logs, Metadata</span>
            </div>
          </div>

          <button
            type="button"
            className="btn btn-primary btn-full-width"
            onClick={createBackup}
            disabled={loading}
          >
            Create Full Backup
          </button>

          {exportStatus?.visible && exportStatus.type === 'backup' && (
            <div className="export-status">
              <div className="export-status-message">{exportStatus.message}</div>
              <div className="export-progress-bar">
                <div className="export-progress-fill" style={{ width: `${exportStatus.progress}%` }} />
              </div>
            </div>
          )}
        </div>
      </div>
    </Modal>
  )
}
