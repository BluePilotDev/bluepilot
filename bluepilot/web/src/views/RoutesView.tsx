import { useEffect, useState, useMemo } from 'react'
import { Header } from '@/components/layout/Header'
import { useRoutesStore } from '@/stores/useRoutesStore'
import { LoadingSpinner } from '@/components/common'
import { VideoPlayer } from '@/components/video/VideoPlayer'
import { DiskSpaceVisualization } from '@/components/storage/DiskSpaceVisualization'
import { MetricsModal, ExportBackupModal } from '@/components/modals'
import type { Route, RouteDetails } from '@/types'
import './RoutesView.css'

interface RoutesViewProps {
  deviceStatus?: 'online' | 'onroad' | 'offline' | 'checking'
}

export const RoutesView = ({ deviceStatus = 'checking' }: RoutesViewProps) => {
  const { routes, loading, fetchRoutes, fetchRouteDetails, preserveRoute } = useRoutesStore()
  const [selectedRoute, setSelectedRoute] = useState<RouteDetails | null>(null)
  const [showMetricsModal, setShowMetricsModal] = useState(false)
  const [showExportModal, setShowExportModal] = useState(false)
  const [exportRoute, setExportRoute] = useState<RouteDetails | null>(null)

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
    await preserveRoute(baseName)
  }

  const handleExportClick = async (e: React.MouseEvent, baseName: string) => {
    e.stopPropagation()
    const routeDetails = await fetchRouteDetails(baseName)
    if (routeDetails) {
      setExportRoute(routeDetails)
      setShowExportModal(true)
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

  return (
    <>
      <Header deviceStatus={deviceStatus} onMetricsClick={() => setShowMetricsModal(true)} />
      <MetricsModal isOpen={showMetricsModal} onClose={() => setShowMetricsModal(false)} />
      <ExportBackupModal
        isOpen={showExportModal}
        onClose={() => {
          setShowExportModal(false)
          setExportRoute(null)
        }}
        route={exportRoute}
      />
      {selectedRoute && (
        <VideoPlayer route={selectedRoute} onClose={handleCloseVideo} />
      )}
      <DiskSpaceVisualization />
      <div className="routes-container">
        {routes.length === 0 ? (
          <div className="empty">
            <svg
              width="120"
              height="120"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="1"
            >
              <path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z" />
              <polyline points="9 22 9 12 15 12 15 22" />
            </svg>
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

                    // Debug logging for first route
                    if (routes.indexOf(route) === 0) {
                      console.log('First route debug:', {
                        baseName,
                        thumbnailUrl: `/api/thumbnail/${baseName}`
                      })
                    }

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
                            <svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor" stroke="currentColor" strokeWidth="2">
                              <polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"/>
                            </svg>
                            Preserved
                          </div>
                        )}

                        {/* Processing Banner */}
                        {route.processing && (
                          <div className="route-processing-banner">
                            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                              <circle cx="12" cy="12" r="10"/>
                              <path d="M12 6v6l4 2"/>
                            </svg>
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
                            <svg width="48" height="48" viewBox="0 0 24 24" fill="#2196f3" stroke="#2196f3" strokeWidth="2">
                              <polygon points="5 3 19 12 5 21 5 3"/>
                            </svg>
                          </div>
                        </div>

                        {/* Route Info */}
                        <div className="route-info">
                          <div className="route-info-header">
                            <div className="route-time-range">
                              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <circle cx="12" cy="12" r="10"/>
                                <polyline points="12 6 12 12 16 14"/>
                              </svg>
                              {formatTimeRange(route)}
                            </div>
                            <div className="route-actions">
                              <button
                                type="button"
                                className={`route-preserve-btn ${preserved ? 'active' : ''}`}
                                aria-pressed={preserved}
                                title={preserved ? 'Preserved' : 'Preserve'}
                                onClick={(e) => handlePreserveToggle(e, baseName)}
                              >
                                <svg width="14" height="14" viewBox="0 0 24 24" fill={preserved ? 'currentColor' : 'none'} stroke="currentColor" strokeWidth="2">
                                  <polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"/>
                                </svg>
                                <span>{preserved ? 'Preserved' : 'Preserve'}</span>
                              </button>
                              <button
                                type="button"
                                className="route-export-btn"
                                title="Export videos and backup route data"
                                onClick={(e) => handleExportClick(e, baseName)}
                              >
                                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                  <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
                                  <polyline points="7 10 12 15 17 10" />
                                  <line x1="12" y1="15" x2="12" y2="3" />
                                </svg>
                                <span>Export</span>
                              </button>
                            </div>
                          </div>

                          {/* Location */}
                          {(startLocation || endLocation) && (
                            <div className="route-location">
                              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <path d="M21 10c0 7-9 13-9 13s-9-6-9-13a9 9 0 0 1 18 0z"/>
                                <circle cx="12" cy="10" r="3"/>
                              </svg>
                              {startLocation === endLocation || !endLocation
                                ? startLocation
                                : `${startLocation || 'N/A'} → ${endLocation || 'N/A'}`
                              }
                            </div>
                          )}

                          {/* Fingerprint */}
                          {route.fingerprint?.carFingerprint && (
                            <div className="route-fingerprint">
                              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <rect x="2" y="7" width="20" height="14" rx="2" ry="2"/>
                                <path d="M16 21V5a2 2 0 0 0-2-2h-4a2 2 0 0 0-2 2v16"/>
                              </svg>
                              {route.fingerprint.carFingerprint}
                            </div>
                          )}

                          {/* Stats Grid */}
                          <div className="route-stats-grid">
                            <div className="route-stat">
                              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <circle cx="12" cy="12" r="10"/>
                                <polyline points="12 6 12 12 16 14"/>
                              </svg>
                              {route.duration}
                            </div>
                            <div className="route-stat">
                              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <path d="M21 10c0 7-9 13-9 13s-9-6-9-13a9 9 0 0 1 18 0z"/>
                                <circle cx="12" cy="10" r="3"/>
                              </svg>
                              {route.distance || getField(route, 'mileage') || '--'}
                            </div>
                            {avgSpeed && (
                              <div className="route-stat">
                                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                  <path d="M12 2v4"/>
                                  <path d="m6.8 15-3.5 2"/>
                                  <path d="m20.7 7-3.5 2"/>
                                  <path d="M6.8 9 3.3 7"/>
                                  <path d="m20.7 17-3.5-2"/>
                                  <path d="M18 18.7a9 9 0 1 0-12 0"/>
                                </svg>
                                {avgSpeed} avg
                              </div>
                            )}
                            {topSpeed && (
                              <div className="route-stat">
                                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                  <polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/>
                                </svg>
                                {topSpeed} top
                              </div>
                            )}
                            {opEngagedPercent !== undefined && (
                              <div className="route-stat route-stat-engagement">
                                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                  <circle cx="12" cy="12" r="10"/>
                                  <path d="M12 6v6l4 2"/>
                                </svg>
                                {Math.round(opEngagedPercent)}% engaged
                              </div>
                            )}
                            {alertCount !== undefined && alertCount > 0 && (
                              <div className="route-stat route-stat-alerts">
                                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                  <path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/>
                                  <line x1="12" y1="9" x2="12" y2="13"/>
                                  <line x1="12" y1="17" x2="12.01" y2="17"/>
                                </svg>
                                {alertCount} alerts
                              </div>
                            )}
                            <div className="route-stat">
                              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/>
                              </svg>
                              {(() => {
                                const segments = getField(route, 'segments', 'totalSegments')
                                // Ensure we only render a number, not an object
                                return typeof segments === 'number' ? segments : '--'
                              })()} seg
                            </div>
                            <div className="route-stat">
                              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                <line x1="22" y1="12" x2="2" y2="12"/>
                                <path d="M5.45 5.11L2 12v6a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2v-6l-3.45-6.89A2 2 0 0 0 16.76 4H7.24a2 2 0 0 0-1.79 1.11z"/>
                              </svg>
                              {route.size}
                            </div>
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
    </>
  )
}
