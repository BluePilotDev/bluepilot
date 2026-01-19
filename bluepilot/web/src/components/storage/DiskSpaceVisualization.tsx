import { useEffect, useState } from 'react'
import { systemAPI } from '@/services/api'
import { useRoutesStore } from '@/stores/useRoutesStore'
import { ConfirmDialog, Icon } from '@/components/common'
import './DiskSpaceVisualization.css'

interface DiskSpaceData {
  total_bytes: number
  used_bytes: number
  free_bytes: number
  preserved_bytes: number
  non_preserved_bytes: number
  deletion_threshold_bytes: number
  formatted: {
    total: string
    used: string
    free: string
    preserved: string
    non_preserved: string
  }
}

interface CacheData {
  total_bytes: number
  formatted: {
    total: string
  }
}

interface DiskAnalysis {
  success: boolean
  disk: DiskSpaceData
  cache?: CacheData
}

export const DiskSpaceVisualization = () => {
  const [diskData, setDiskData] = useState<DiskAnalysis | null>(null)
  const [loading, setLoading] = useState(true)
  const [showClearCacheConfirm, setShowClearCacheConfirm] = useState(false)
  const { routes, clearCache, fetchRoutes } = useRoutesStore()

  useEffect(() => {
    loadDiskAnalysis()
  }, [])

  const loadDiskAnalysis = async () => {
    try {
      setLoading(true)
      const response = await systemAPI.getDiskAnalysis()
      setDiskData(response)
    } catch (error) {
      console.error('Failed to load disk analysis:', error)
    } finally {
      setLoading(false)
    }
  }

  const handleClearCache = () => {
    setShowClearCacheConfirm(true)
  }

  const handleConfirmClearCache = async () => {
    await clearCache()
    loadDiskAnalysis() // Reload disk analysis after clearing
  }

  const handleRefresh = async () => {
    await fetchRoutes(1)
    loadDiskAnalysis() // Reload disk analysis after refresh
  }

  if (loading || !diskData || !diskData.success) {
    return null
  }

  const { disk, cache } = diskData

  // Calculate percentages for gauge bar segments
  const preservedPercent = (disk.preserved_bytes / disk.total_bytes) * 100
  const routesPercent = (disk.non_preserved_bytes / disk.total_bytes) * 100
  const cachePercent = cache ? (cache.total_bytes / disk.total_bytes) * 100 : 0

  // Position threshold marker
  const thresholdPercent = (disk.deletion_threshold_bytes / disk.total_bytes) * 100
  const thresholdPosition = 100 - thresholdPercent

  // Determine warning level
  const usedPercent = (disk.used_bytes / disk.total_bytes) * 100
  let warningLevel: 'none' | 'warning' | 'critical' = 'none'
  let warningText = ''

  if (usedPercent >= 95) {
    warningLevel = 'critical'
    warningText = 'Critical'
  } else if (usedPercent >= 85) {
    warningLevel = 'warning'
    warningText = 'Low Space'
  }

  // Calculate total size (rough estimate from routes)
  const totalSize = routes.length > 0
    ? ((routes.length * 500) / 1024).toFixed(1) + ' GB'
    : '0 GB'

  return (
    <>
      <ConfirmDialog
        isOpen={showClearCacheConfirm}
        onClose={() => setShowClearCacheConfirm(false)}
        onConfirm={handleConfirmClearCache}
        title="Clear Cached Data"
        message="Are you sure you want to clear all cached data (videos, thumbnails, GPS)? This will free up disk space but cached videos will need to be re-downloaded when viewing routes."
        confirmText="Clear Cache"
        cancelText="Cancel"
        variant="warning"
      />
      <div className="storage-bar">
        <div className="storage-bar-content">
        <div className="storage-bar-left">
          <div className="storage-info">
            <span className="storage-text">
              {disk.formatted.used} / {disk.formatted.total}
            </span>
            {warningLevel !== 'none' && (
              <div className="storage-warning-icon" title={warningText}>
                <Icon name="warning" size={14} />
              </div>
            )}
          </div>
          <div className="storage-gauge">
            <div
              className="storage-segment storage-preserved"
              style={{ width: `${preservedPercent}%` }}
            />
            <div
              className="storage-segment storage-routes"
              style={{ width: `${routesPercent}%` }}
            />
            {cache && (
              <div
                className="storage-segment storage-cache"
                style={{ width: `${cachePercent}%` }}
              />
            )}
            <div
              className="storage-threshold"
              style={{ left: `${thresholdPosition}%` }}
            />
          </div>
          <div className="storage-legend">
            <div className="storage-legend-item">
              <span className="storage-dot storage-dot-preserved" />
              <span className="storage-legend-text">{disk.formatted.preserved}</span>
            </div>
            <div className="storage-legend-item">
              <span className="storage-dot storage-dot-routes" />
              <span className="storage-legend-text">{disk.formatted.non_preserved}</span>
            </div>
            {cache && (
              <div className="storage-legend-item">
                <span className="storage-dot storage-dot-cache" />
                <span className="storage-legend-text">{cache.formatted.total}</span>
              </div>
            )}
          </div>
          <div className="storage-stats">
            <span className="storage-badge">
              <Icon name="place" size={14} />
              <span>{routes.length} routes</span>
            </span>
            <span className="storage-badge">
              <Icon name="inventory_2" size={14} />
              <span>{totalSize}</span>
            </span>
          </div>
        </div>
        <div className="storage-bar-right">
          <button
            type="button"
            className="icon-btn"
            title="Clear all cached data (videos, thumbnails, GPS)"
            onClick={handleClearCache}
          >
            <Icon name="delete" size={20} />
          </button>
          <button
            type="button"
            className="icon-btn"
            title="Refresh routes"
            onClick={handleRefresh}
          >
            <Icon name="refresh" size={20} />
          </button>
        </div>
      </div>
      </div>
    </>
  )
}
