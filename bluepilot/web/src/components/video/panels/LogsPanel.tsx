import { useState, useRef, useEffect } from 'react'
import { Icon } from '@/components/common'
import { useToastStore } from '@/stores/useToastStore'
import type { RouteDetails } from '@/types'
import { ansiToHtml } from '@/utils/ansiParser'
import './Panels.css'

interface LogsPanelProps {
  route: RouteDetails | null
  currentSegment: number
  videoCurrentTime?: number
}

interface LogMessage {
  timestamp: number
  level: string
  message: string
}

interface LogResponse {
  success: boolean
  messages: LogMessage[]
  total_count: number
  returned_count: number
  truncated: boolean
  start_time: number
  end_time: number
  error?: string
}

export const LogsPanel = ({ route, currentSegment, videoCurrentTime = 0 }: LogsPanelProps) => {
  const { addToast } = useToastStore()
  const [isOpen, setIsOpen] = useState(false)
  const [isPaused, setIsPaused] = useState(false)
  const [logType, setLogType] = useState<'qlog' | 'rlog'>('rlog')
  const [levelFilter, setLevelFilter] = useState('all')
  const [searchQuery, setSearchQuery] = useState('')
  const [loading, setLoading] = useState(false)
  const [logs, setLogs] = useState<LogMessage[]>([])
  const [logStats, setLogStats] = useState({ returned: 0, total: 0, truncated: false })
  const [logTimeRange, setLogTimeRange] = useState({ start: 0, end: 0 })
  const messagesRef = useRef<HTMLDivElement>(null)
  const syncIntervalRef = useRef<ReturnType<typeof setInterval>>()

  useEffect(() => {
    return () => {
      if (syncIntervalRef.current) {
        clearInterval(syncIntervalRef.current)
      }
    }
  }, [])

  // Sync logs to video playback time (unless paused)
  useEffect(() => {
    if (logs.length === 0 || !messagesRef.current || isPaused) return

    // Calculate time relative to current segment (each segment is 60 seconds)
    const segmentOffset = currentSegment * 60
    const segmentRelativeTime = videoCurrentTime - segmentOffset
    const segmentStartTime = logTimeRange.start

    // Find the log message closest to current video time
    const targetTimestamp = segmentStartTime + segmentRelativeTime

    let closestIndex = -1
    let closestDistance = Infinity

    const messageElements = Array.from(messagesRef.current.querySelectorAll<HTMLElement>('.log-message'))
    messageElements.forEach((el, index) => {
      const timestamp = parseFloat(el.dataset.timestamp || '0')
      const distance = Math.abs(timestamp - targetTimestamp)

      if (distance < closestDistance) {
        closestDistance = distance
        closestIndex = index
      }
    })

    // Scroll to closest message
    if (closestIndex >= 0) {
      const closestElement = messageElements[closestIndex]
      if (closestElement) {
        closestElement.scrollIntoView({ behavior: 'smooth', block: 'center' })

        // Highlight current message
        messageElements.forEach(el => el.classList.remove('log-message-active'))
        closestElement.classList.add('log-message-active')
      }
    }
  }, [videoCurrentTime, logs, currentSegment, logTimeRange.start, isPaused])

  const loadLogs = async () => {
    if (!route) {
      console.warn('No route selected')
      return
    }

    setLoading(true)
    setIsOpen(true)
    setIsPaused(false)

    try {
      // Build API URL
      const params = new URLSearchParams()
      if (levelFilter !== 'all') {
        params.append('level', levelFilter)
      }
      if (searchQuery.trim()) {
        params.append('search', searchQuery.trim())
      }
      params.append('max', '500')

      const url = `${window.location.origin}/api/logs/${route.baseName}/${currentSegment}/${logType}?${params}`

      console.log('Loading logs from:', url)

      const response = await fetch(url)
      const data: LogResponse = await response.json()

      setLoading(false)

      if (data.success) {
        setLogs(data.messages)
        setLogStats({
          returned: data.returned_count,
          total: data.total_count,
          truncated: data.truncated
        })
        setLogTimeRange({
          start: data.start_time,
          end: data.end_time
        })
      } else {
        addToast(`Error loading logs: ${data.error}`, 'error')
        setLogs([])
      }
    } catch (error) {
      console.error('Error loading logs:', error)
      setLoading(false)
      addToast(`Failed to load logs: ${error}`, 'error')
      setLogs([])
    }
  }

  const stopLogs = () => {
    setLogs([])
    setLogStats({ returned: 0, total: 0, truncated: false })
    setIsPaused(false)
  }

  const formatTimestamp = (timestamp: number): string => {
    const relativeTime = timestamp - logTimeRange.start
    const minutes = Math.floor(relativeTime / 60)
    const seconds = (relativeTime % 60).toFixed(3)
    return `${minutes}:${seconds.padStart(6, '0')}`
  }

  return (
    <div className="bottom-panel" id="log-panel">
      <div
        className="panel-header"
        id="log-panel-header"
        onClick={() => setIsOpen(!isOpen)}
        style={{ cursor: 'pointer' }}
      >
        <div className="panel-title">
          <Icon name="description" className="panel-icon" size={20} />
          <div className="panel-title-text">
            <span className="panel-title-label">Route Logs</span>
            <span className="panel-title-subtitle">Cloudlog events & diagnostics</span>
          </div>
        </div>
        <div className="panel-header-actions">
          {logs.length > 0 ? (
            <>
              <button
                type="button"
                className={`btn btn-sm panel-action-btn ${isPaused ? 'btn-success' : 'btn-secondary'}`}
                onClick={(e) => {
                  e.stopPropagation()
                  setIsPaused(!isPaused)
                }}
                title={isPaused ? 'Resume auto-scroll' : 'Pause auto-scroll'}
              >
                <Icon name={isPaused ? 'play_arrow' : 'pause'} size={16} />
              </button>
              <button
                type="button"
                className="btn btn-sm btn-danger panel-action-btn"
                onClick={(e) => {
                  e.stopPropagation()
                  stopLogs()
                }}
              >
                Stop
              </button>
            </>
          ) : (
            <button
              type="button"
              className="btn btn-sm btn-primary panel-action-btn"
              onClick={(e) => {
                e.stopPropagation()
                loadLogs()
              }}
            >
              Load Logs
            </button>
          )}
          <Icon
            name={isOpen ? "expand_more" : "chevron_right"}
            size={20}
            style={{ marginLeft: '8px', opacity: 0.7, cursor: 'pointer' }}
          />
        </div>
      </div>
      {isOpen && (
        <div className="panel-content" id="log-panel-content">
          {/* Log Controls */}
          <div className="log-controls log-controls-sticky">
            <div className="log-control-row">
              <div className="log-type-selector">
                <button
                  type="button"
                  className={`log-type-btn ${logType === 'qlog' ? 'active' : ''}`}
                  onClick={() => setLogType('qlog')}
                >
                  qlog
                </button>
                <button
                  type="button"
                  className={`log-type-btn ${logType === 'rlog' ? 'active' : ''}`}
                  onClick={() => setLogType('rlog')}
                >
                  rlog
                </button>
              </div>
              <select
                id="log-level-filter"
                className="log-filter-select"
                value={levelFilter}
                onChange={(e) => setLevelFilter(e.target.value)}
              >
                <option value="all">All Levels</option>
                <option value="error">Errors Only</option>
                <option value="warning">Warnings</option>
                <option value="info">Info</option>
              </select>
              <input
                type="text"
                id="log-search-input"
                className="log-search-input"
                placeholder="Search logs..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                onKeyPress={(e) => {
                  if (e.key === 'Enter') {
                    loadLogs()
                  }
                }}
              />
            </div>
          </div>

          {/* Log Display */}
          <div className="log-viewer-container">
            {loading ? (
              <div className="log-loading">
                <div className="spinner-small"></div>
                <span>Loading logs...</span>
              </div>
            ) : logs.length === 0 ? (
              <div className="log-viewer-empty">
                <p>Click "Load Logs" to view cloudlog messages</p>
              </div>
            ) : (
              <>
                <div ref={messagesRef} className="log-messages">
                  {logs.map((log, index) => (
                    <div
                      key={index}
                      className={`log-message log-${log.level}`}
                      data-timestamp={log.timestamp}
                    >
                      <span className="log-timestamp">{formatTimestamp(log.timestamp)}</span>
                      <span className={`log-level ${log.level}`}>{log.level}</span>
                      <span
                        className="log-text"
                        dangerouslySetInnerHTML={{ __html: ansiToHtml(log.message) }}
                      />
                    </div>
                  ))}
                </div>
                <div className="log-status">
                  <span id="log-count">
                    {logStats.returned} message{logStats.returned !== 1 ? 's' : ''}
                    {logStats.truncated && ` (showing first ${logStats.returned} of ${logStats.total})`}
                  </span>
                </div>
              </>
            )}
          </div>
        </div>
      )}
    </div>
  )
}
