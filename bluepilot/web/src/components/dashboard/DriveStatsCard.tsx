/**
 * Drive Stats Card Component
 * Shows aggregate drive statistics similar to Qt widget
 */

import { useEffect, useState } from 'react'
import './DriveStatsCard.css'

interface Stats {
  routes: number
  distance: number  // in meters
  distanceMiles: number  // in miles (from API)
  duration: number  // in seconds
  durationMinutes: number  // in minutes (from API)
  averageSpeed: number  // in m/s
}

interface DriveStatsResponse {
  success: boolean
  all: Stats
  week: Stats
  source?: string
  error?: string
  cloud_error?: string
}

export function DriveStatsCard() {
  const [data, setData] = useState<DriveStatsResponse | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    fetchDriveStats()
  }, [])

  const fetchDriveStats = async () => {
    setLoading(true)
    try {
      const response = await fetch('/api/drive-stats')
      if (response.ok) {
        const responseData = await response.json()
        if (responseData.success) {
          setData(responseData)
        }
      }
    } catch (error) {
      console.error('Error fetching drive stats:', error)
    } finally {
      setLoading(false)
    }
  }

  if (loading) {
    return (
      <div className="drive-stats-card">
        <div className="stats-header">
          <h3>Drive Statistics</h3>
        </div>
        <div className="drive-stats-loading">Loading statistics...</div>
      </div>
    )
  }

  if (!data) {
    return (
      <div className="drive-stats-card">
        <div className="stats-header">
          <h3>Drive Statistics</h3>
        </div>
        <div className="drive-stats-empty">No drive data available</div>
      </div>
    )
  }

  const allHours = Math.round(data.all.duration / 3600)
  const allMiles = Math.round(data.all.distanceMiles)
  const weekHours = Math.round(data.week.duration / 3600)
  const weekMiles = Math.round(data.week.distanceMiles)

  return (
    <div className="drive-stats-card">
      <div className="stats-header">
        <h3>Drive Statistics</h3>
        {data.source === 'param_cache' && <span className="stats-badge cached">Cached</span>}
        {data.source === 'no_cache' && <span className="stats-badge info">No Data</span>}
        {data.cloud_error && <span className="stats-badge warning">Cloud Error</span>}
      </div>

      <div className="stats-grid">
        <div className="stat-item all-time">
          <span className="stat-label">All Time Drives</span>
          <span className="stat-value">{data.all.routes}</span>
        </div>
        <div className="stat-item all-time">
          <span className="stat-label">All Time Miles</span>
          <span className="stat-value">{allMiles}</span>
        </div>
        <div className="stat-item all-time">
          <span className="stat-label">All Time Hours</span>
          <span className="stat-value">{allHours}</span>
        </div>
        <div className="stat-item">
          <span className="stat-label">Week Drives</span>
          <span className="stat-value">{data.week.routes}</span>
        </div>
        <div className="stat-item">
          <span className="stat-label">Week Miles</span>
          <span className="stat-value">{weekMiles}</span>
        </div>
        <div className="stat-item">
          <span className="stat-label">Week Hours</span>
          <span className="stat-value">{weekHours}</span>
        </div>
      </div>
    </div>
  )
}
