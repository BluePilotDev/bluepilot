import { useEffect, useState, useMemo } from 'react'
import { Header } from '@/components/layout/Header'
import { useRoutesStore } from '@/stores/useRoutesStore'
import { useToastStore } from '@/stores/useToastStore'
import { LoadingSpinner, ConfirmDialog, Icon, BackToTop } from '@/components/common'
import { VideoPlayer } from '@/components/video/VideoPlayer'
import { DiskSpaceVisualization } from '@/components/storage/DiskSpaceVisualization'
import { MetricsModal, RouteDownloadModal } from '@/components/modals'
import type { Route, RouteDetails, DeviceStatus } from '@/types'
import './RoutesView.css'

interface RoutesViewProps {
  deviceStatus?: DeviceStatus
}

export const RoutesView = ({ deviceStatus = 'checking' }: RoutesViewProps) => {
  const { routes, loading, fetchRoutes, fetchRouteDetails, preserveRoute, deleteRoute } = useRoutesStore()
  const { addToast } = useToastStore()
  const [selectedRoute, setSelectedRoute] = useState<RouteDetails | null>(null)
  const [showMetricsModal, setShowMetricsModal] = useState(false)
  const [showExportModal, setShowExportModal] = useState(false)
  const [exportRoute, setExportRoute] = useState<RouteDetails | null>(null)
  const [showDeleteConfirm, setShowDeleteConfirm] = useState(false)
  const [showPreserveConfirm, setShowPreserveConfirm] = useState(false)
  const [pendingActionRoute, setPendingActionRoute] = useState<string | null>(null)

  useEffect(() => {
    fetchRoutes()
  }, [fetchRoutes])

  const handleRouteClick = async (baseName: string) => {
    const routeDetails = await fetchRouteDetails(baseName)
    if (routeDetails) {
      setSelectedRoute(routeDetails)
    }
  }

  const handleCloseVideo = () => {
    setSelectedRoute(null)
  }

  const handlePreserveToggle = async (e: React.MouseEvent, baseName: string) => {
    e.stopPropagation()
    // Check if the route is currently preserved
    const route = routes.find(r => r.baseName === baseName)
    const isCurrentlyPreserved = route ? (route.preserved || (route as any).isStarred) : false

    if (isCurrentlyPreserved) {
      // If already preserved, show confirmation to unpreserve
      setPendingActionRoute(baseName)
      setShowPreserveConfirm(true)
    } else {
      // If not preserved, preserve without confirmation
      try {
        await preserveRoute(baseName)
        addToast('Route preserved successfully', 'success')
      } catch (error: any) {
        console.error('Failed to preserve route:', error)
        addToast(error?.message || 'Failed to preserve route', 'error')
      }
    }
  }

  const handleConfirmUnpreserve = async () => {
    if (!pendingActionRoute) return

    try {
      await preserveRoute(pendingActionRoute)
      addToast('Route unpreserved successfully', 'success')
    } catch (error: any) {
      console.error('Failed to unpreserve route:', error)
      addToast(error?.message || 'Failed to unpreserve route', 'error')
    } finally {
      setPendingActionRoute(null)
    }
  }

  const handleExportClick = async (e: React.MouseEvent, baseName: string) => {
    e.stopPropagation()
    const routeDetails = await fetchRouteDetails(baseName)
    if (routeDetails) {
      setExportRoute(routeDetails)
      setShowExportModal(true)
    }
  }

  const handleDeleteClick = async (e: React.MouseEvent, baseName: string) => {
    e.stopPropagation()
    setPendingActionRoute(baseName)
    setShowDeleteConfirm(true)
  }

  const handleConfirmDelete = async () => {
    if (!pendingActionRoute) return

    try {
      await deleteRoute(pendingActionRoute)
      addToast('Route deleted successfully', 'success')
    } catch (error: any) {
      console.error('Failed to delete route:', error)
      addToast(error?.message || 'Failed to delete route', 'error')
    } finally {
      setPendingActionRoute(null)
    }
  }

  // Helper function to get field value with fallback names (supports nested paths like 'driveStats.opEngagedPercent')
  const getField = (route: Route, ...fields: string[]): any => {
    for (const field of fields) {
      // Handle nested paths like 'driveStats.opEngagedPercent'
      if (field.includes('.')) {
        const parts = field.split('.')
        let value: any = route
        for (const part of parts) {
          if (value && typeof value === 'object') {
            value = (value as any)[part]
          } else {
            value = undefined
            break
          }
        }
        if (value !== undefined && value !== null) return value
      } else {
        const value = (route as any)[field]
        if (value !== undefined && value !== null) return value
      }
    }
    return undefined
  }

  // Format UTC timestamp to local time (e.g., "3:45 PM")
  const formatLocalTime = (utcTimestamp?: string): string => {
    if (!utcTimestamp) return ''
    try {
      let timestamp = utcTimestamp
      // Append Z to treat as UTC if no timezone info
      if (!timestamp.includes('+') && !timestamp.endsWith('Z')) {
        timestamp = timestamp + 'Z'
      }
      const date = new Date(timestamp)
      return date.toLocaleTimeString('en-US', {
        hour: 'numeric',
        minute: '2-digit',
        hour12: true
      })
    } catch (e) {
      console.error('Error formatting time:', e)
      return ''
    }
  }

  // Format UTC timestamp to local date (e.g., "Thursday - September 18th, 2024")
  const formatLocalDate = (utcTimestamp?: string): string => {
    if (!utcTimestamp) return 'Unknown'
    try {
      let timestamp = utcTimestamp
      // Append Z to treat as UTC if no timezone info
      if (!timestamp.includes('+') && !timestamp.endsWith('Z')) {
        timestamp = timestamp + 'Z'
      }
      const date = new Date(timestamp)

      // Get day of week
      const dayOfWeek = date.toLocaleDateString('en-US', { weekday: 'long' })

      // Get month
      const month = date.toLocaleDateString('en-US', { month: 'long' })

      // Get day with ordinal suffix (1st, 2nd, 3rd, etc.)
      const day = date.getDate()
      const ordinal = (day: number) => {
        if (day > 3 && day < 21) return 'th'
        switch (day % 10) {
          case 1: return 'st'
          case 2: return 'nd'
          case 3: return 'rd'
          default: return 'th'
        }
      }

      const year = date.getFullYear()

      return `${dayOfWeek} - ${month} ${day}${ordinal(day)}, ${year}`
    } catch (e) {
      console.error('Error formatting date:', e)
      return 'Unknown'
    }
  }

  // Group routes by date (convert UTC timestamp to local date)
  const groupedRoutes = useMemo(() => {
    const grouped: Record<string, Route[]> = {}

    for (const route of routes) {
      const dateKey = formatLocalDate(route.timestamp)
      if (!grouped[dateKey]) {
        grouped[dateKey] = []
      }
      grouped[dateKey].push(route)
    }

    return grouped
  }, [routes])

  // Format time range (start - end)
  const formatTimeRange = (route: Route): string => {
    if (!route.timestamp) {
      return route.baseName || ''
    }

    const startTime = formatLocalTime(route.timestamp)
    if (!startTime) {
      return route.baseName || ''
    }

    // If we have duration, calculate end time
    if (route.duration) {
      try {
        // Parse duration string (e.g., "1h 30m" or "45m")
        const durationMatch = route.duration.match(/(?:(\d+)h\s*)?(?:(\d+)m)?/)
        if (durationMatch) {
          const hours = parseInt(durationMatch[1] || '0')
          const minutes = parseInt(durationMatch[2] || '0')
          const totalMinutes = hours * 60 + minutes

          // Add 'Z' to treat timestamp as UTC
          let timestamp = route.timestamp
          if (!timestamp.includes('+') && !timestamp.endsWith('Z')) {
            timestamp = timestamp + 'Z'
          }

          const startDate = new Date(timestamp)
          const endDate = new Date(startDate.getTime() + totalMinutes * 60 * 1000)
          const endTime = endDate.toLocaleTimeString('en-US', {
            hour: 'numeric',
            minute: '2-digit',
            hour12: true
          })

          return `${startTime} - ${endTime}`
        }
      } catch (e) {
        console.error('Error calculating end time:', e)
      }
    }

    return startTime
  }

  if (loading && routes.length === 0) {
    return (
      <>
        <Header deviceStatus={deviceStatus} />
        <div className="loading">
          <LoadingSpinner size="large" message="Loading routes..." />
        </div>
      </>
    )
  }

  // Block routes access while driving
  if (deviceStatus === 'onroad') {
    return (
      <>
        <Header deviceStatus={deviceStatus} subtitle="Routes unavailable while driving" />
        <div className="routes-blocked">
          <Icon name="directions_car" size={80} />
          <h2>Routes Unavailable</h2>
          <p>Route browsing is disabled while driving for safety. Please return when parked.</p>
        </div>
      </>
    )
  }

  return (
    <>
      <Header
        deviceStatus={deviceStatus}
        onMetricsClick={() => setShowMetricsModal(true)}
        subtitle="Browse and review your driving recordings"
      />
      <MetricsModal isOpen={showMetricsModal} onClose={() => setShowMetricsModal(false)} />
      <RouteDownloadModal
        isOpen={showExportModal}
        onClose={() => {
          setShowExportModal(false)
          setExportRoute(null)
        }}
        route={exportRoute}
      />
      <ConfirmDialog
        isOpen={showDeleteConfirm}
        onClose={() => {
          setShowDeleteConfirm(false)
          setPendingActionRoute(null)
        }}
        onConfirm={handleConfirmDelete}
        title="Delete Route"
        message="Are you sure you want to delete this route? This action cannot be undone and all video files and logs will be permanently removed."
        confirmText="Delete"
        cancelText="Cancel"
        variant="danger"
      />
      <ConfirmDialog
        isOpen={showPreserveConfirm}
        onClose={() => {
          setShowPreserveConfirm(false)
          setPendingActionRoute(null)
        }}
        onConfirm={handleConfirmUnpreserve}
        title="Unpreserve Route"
        message="Are you sure you want to unpreserve this route? The route may be automatically deleted when disk space is needed."
        confirmText="Unpreserve"
        cancelText="Cancel"
        variant="warning"
      />
      {selectedRoute && (
        <VideoPlayer route={selectedRoute} onClose={handleCloseVideo} />
      )}
      <DiskSpaceVisualization />
      <div className="routes-container">
        {routes.length === 0 ? (
          <div className="empty">
            <Icon name="home" size={120} />
            <h2>No Routes Found</h2>
            <p>
              No driving routes available yet. Start driving to see your routes here.
            </p>
          </div>
        ) : (
          <>
            {Object.entries(groupedRoutes).map(([date, dateRoutes]) => (
              <div key={date} className="date-group">
                <div className="date-header">{date}</div>
                <div className="date-group-cards">
                  {dateRoutes.map((route) => {
                    // Get baseName (primary identifier from backend)
                    const baseName = route.baseName || route.id || ''
                    const preserved = getField(route, 'preserved', 'isStarred')
                    const startLocation = getField(route, 'start_location', 'startLocation')
                    const endLocation = getField(route, 'end_location', 'endLocation')
                    const avgSpeed = getField(route, 'avg_speed', 'avgSpeed')
                    const topSpeed = getField(route, 'top_speed', 'topSpeed')
                    const opEngagedPercent = getField(route, 'op_engaged_percent', 'driveStats.opEngagedPercent')
                    const alertCount = getField(route, 'alert_count', 'driveStats.alertCount', 'alerts')

                    return (
                      <div
                        key={baseName}
                        className="route-card"
                        onClick={() => handleRouteClick(baseName)}
                        data-base-name={baseName}
                      >
                        {/* Preserved Badge */}
                        {preserved && (
                          <div className="preserved-badge">
                            <Icon name="star" size={12} />
                            Preserved
                          </div>
                        )}

                        {/* Processing Banner */}
                        {route.processing && (
                          <div className="route-processing-banner">
                            <Icon name="schedule" size={14} />
                            <span className="banner-text">Processing drive statistics...</span>
                          </div>
                        )}

                        {/* Thumbnail */}
                        <div className="route-thumbnail">
                          <img
                            src={`/api/thumbnail/${baseName}`}
                            onError={(e) => { e.currentTarget.style.display = 'none' }}
                            alt="Route thumbnail"
                          />
                          <div className="play-overlay">
                            <Icon name="play_circle_filled" size={48} style={{ color: '#2196f3' }} />
                          </div>
                        </div>

                        {/* Route Info */}
                        <div className="route-info">
                          <div className="route-info-header">
                            <div className="route-time-range">
                              <Icon name="schedule" size={16} />
                              {formatTimeRange(route)}
                            </div>
                          </div>

                          {/* Location */}
                          {(startLocation || endLocation) && (
                            <div className="route-location">
                              <Icon name="place" size={14} />
                              {startLocation === endLocation || !endLocation
                                ? startLocation
                                : `${startLocation || 'N/A'} → ${endLocation || 'N/A'}`
                              }
                            </div>
                          )}

                          {/* Fingerprint */}
                          {route.fingerprint?.carFingerprint && (
                            <div className="route-fingerprint">
                              <Icon name="directions_car" size={14} />
                              {route.fingerprint.carFingerprint}
                            </div>
                          )}

                          {/* Stats Grid */}
                          <div className="route-stats-grid">
                            <div className="route-stat">
                              <Icon name="schedule" size={14} />
                              {route.duration}
                            </div>
                            <div className="route-stat">
                              <Icon name="place" size={14} />
                              {route.distance || getField(route, 'mileage') || '--'}
                            </div>
                            {avgSpeed && (
                              <div className="route-stat">
                                <Icon name="speed" size={14} />
                                {avgSpeed} avg
                              </div>
                            )}
                            {topSpeed && (
                              <div className="route-stat">
                                <Icon name="bolt" size={14} />
                                {topSpeed} top
                              </div>
                            )}
                            {opEngagedPercent !== undefined && (
                              <div className="route-stat route-stat-engagement">
                                <Icon name="schedule" size={14} />
                                {Math.round(opEngagedPercent)}% engaged
                              </div>
                            )}
                            {alertCount !== undefined && alertCount > 0 && (
                              <div className="route-stat route-stat-alerts">
                                <Icon name="warning" size={14} />
                                {alertCount} alerts
                              </div>
                            )}
                            <div className="route-stat">
                              <Icon name="inventory_2" size={14} />
                              {(() => {
                                const segments = getField(route, 'segments', 'totalSegments')
                                // Ensure we only render a number, not an object
                                return typeof segments === 'number' ? segments : '--'
                              })()} seg
                            </div>
                            <div className="route-stat">
                              <Icon name="folder" size={14} />
                              {route.size}
                            </div>
                          </div>

                          {/* Route Actions */}
                          <div className="route-actions">
                            <button
                              type="button"
                              className={`route-preserve-btn ${preserved ? 'active' : ''}`}
                              aria-pressed={!!preserved}
                              title={preserved ? 'Preserved' : 'Preserve'}
                              onClick={(e) => handlePreserveToggle(e, baseName)}
                            >
                              <Icon name={preserved ? 'star' : 'star_outline'} size={14} />
                              <span>{preserved ? 'Preserved' : 'Preserve'}</span>
                            </button>
                            <button
                              type="button"
                              className="route-export-btn"
                              title="Export videos and logs"
                              onClick={(e) => handleExportClick(e, baseName)}
                            >
                              <Icon name="download" size={14} />
                              <span>Export</span>
                            </button>
                            <button
                              type="button"
                              className="route-delete-btn"
                              title="Delete route"
                              onClick={(e) => handleDeleteClick(e, baseName)}
                            >
                              <Icon name="delete" size={16} />
                            </button>
                          </div>
                        </div>
                      </div>
                    )
                  })}
                </div>
              </div>
            ))}
          </>
        )}
      </div>

      <BackToTop />
    </>
  )
}
