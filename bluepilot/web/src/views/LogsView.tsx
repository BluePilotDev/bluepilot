import { useEffect, useState, useRef, useMemo, useCallback } from 'react'
import { Header } from '@/components/layout/Header'
import { Button, ToggleSwitch, Icon, BackToTop } from '@/components/common'
import type { DeviceStatus } from '@/types'
import './LogsView.css'

interface LogResponse {
  success: boolean
  output?: string
  error?: string
}

interface WebSocketMessage {
  type: string
  data?: {
    line?: string
    status?: string
    message?: string
  }
}

interface LogsViewProps {
  deviceStatus?: DeviceStatus
}

// ANSI color code parser
const parseAnsiColors = (text: string): JSX.Element[] => {
  const ansiRegex = /\u001b\[([0-9;]+)m/g
  const parts: JSX.Element[] = []
  let lastIndex = 0
  let currentClasses: string[] = []
  let keyCounter = 0

  const ansiCodeToClass = (code: number): string | null => {
    // Foreground colors
    if (code === 30) return 'ansi-black'
    if (code === 31) return 'ansi-red'
    if (code === 32) return 'ansi-green'
    if (code === 33) return 'ansi-yellow'
    if (code === 34) return 'ansi-blue'
    if (code === 35) return 'ansi-magenta'
    if (code === 36) return 'ansi-cyan'
    if (code === 37) return 'ansi-white'

    // Bright foreground colors
    if (code === 90) return 'ansi-bright-black'
    if (code === 91) return 'ansi-bright-red'
    if (code === 92) return 'ansi-bright-green'
    if (code === 93) return 'ansi-bright-yellow'
    if (code === 94) return 'ansi-bright-blue'
    if (code === 95) return 'ansi-bright-magenta'
    if (code === 96) return 'ansi-bright-cyan'
    if (code === 97) return 'ansi-bright-white'

    // Background colors
    if (code === 40) return 'ansi-bg-black'
    if (code === 41) return 'ansi-bg-red'
    if (code === 42) return 'ansi-bg-green'
    if (code === 43) return 'ansi-bg-yellow'
    if (code === 44) return 'ansi-bg-blue'
    if (code === 45) return 'ansi-bg-magenta'
    if (code === 46) return 'ansi-bg-cyan'
    if (code === 47) return 'ansi-bg-white'

    // Text styles
    if (code === 1) return 'ansi-bold'
    if (code === 2) return 'ansi-dim'
    if (code === 3) return 'ansi-italic'
    if (code === 4) return 'ansi-underline'

    // Reset
    if (code === 0) return null

    return null
  }

  let match: RegExpExecArray | null
  while ((match = ansiRegex.exec(text)) !== null) {
    // Add text before this ANSI code
    if (match.index > lastIndex) {
      const textContent = text.substring(lastIndex, match.index)
      if (currentClasses.length > 0) {
        parts.push(
          <span key={`span-${keyCounter++}`} className={currentClasses.join(' ')}>
            {textContent}
          </span>
        )
      } else {
        parts.push(<span key={`span-${keyCounter++}`}>{textContent}</span>)
      }
    }

    // Parse ANSI codes
    const codes = match[1].split(';').map(Number)
    for (const code of codes) {
      if (code === 0) {
        currentClasses = []
      } else {
        const className = ansiCodeToClass(code)
        if (className) {
          currentClasses.push(className)
        }
      }
    }

    lastIndex = ansiRegex.lastIndex
  }

  // Add remaining text
  if (lastIndex < text.length) {
    const textContent = text.substring(lastIndex)
    if (currentClasses.length > 0) {
      parts.push(
        <span key={`span-${keyCounter++}`} className={currentClasses.join(' ')}>
          {textContent}
        </span>
      )
    } else {
      parts.push(<span key={`span-${keyCounter++}`}>{textContent}</span>)
    }
  }

  return parts.length > 0 ? parts : [<span key="span-0">{text}</span>]
}

export function LogsView({ deviceStatus = 'checking' }: LogsViewProps) {
  const [logLines, setLogLines] = useState<string[]>([])
  const [isPaused, setIsPaused] = useState(false)
  const [searchQuery, setSearchQuery] = useState('')
  const [isConnecting, setIsConnecting] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [autoScroll, setAutoScroll] = useState(true)
  const [streamStatus, setStreamStatus] = useState<string>('stopped')

  const logContainerRef = useRef<HTMLPreElement>(null)
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null)
  const seenLinesRef = useRef<Set<string>>(new Set())

  // Load initial logs
  useEffect(() => {
    const fetchInitialLogs = async () => {
      try {
        const response = await fetch('/api/manager-logs')
        if (response.ok) {
          const data: LogResponse = await response.json()
          if (data.success && data.output) {
            const lines = data.output.split('\n').filter(line => line.trim())
            // Track seen lines for deduplication
            seenLinesRef.current = new Set(lines)
            setLogLines(lines)
          }
        }
      } catch (err) {
        console.error('Failed to fetch initial logs:', err)
      }
    }
    fetchInitialLogs()
  }, [])

  // WebSocket connection
  useEffect(() => {
    const connectWebSocket = () => {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
      const ws = new WebSocket(`${protocol}//${window.location.hostname}:8089`)

      ws.onopen = () => {
        console.log('WebSocket connected')
        setError(null)
        setIsConnecting(false)
        wsRef.current = ws
      }

      ws.onmessage = (event) => {
        try {
          const message: WebSocketMessage = JSON.parse(event.data)

          if (message.type === 'log_line' && message.data?.line && !isPaused) {
            const newLine = message.data.line
            // Skip duplicate lines
            if (seenLinesRef.current.has(newLine)) {
              return
            }
            seenLinesRef.current.add(newLine)

            setLogLines(prev => {
              const newLines = [...prev, newLine]
              // Keep only last 2000 lines to prevent memory issues
              if (newLines.length > 2000) {
                // Remove oldest lines from seen set too
                const removed = newLines.slice(0, newLines.length - 2000)
                removed.forEach(line => seenLinesRef.current.delete(line))
                return newLines.slice(-2000)
              }
              return newLines
            })
          } else if (message.type === 'log_stream_status' && message.data?.status) {
            setStreamStatus(message.data.status)
            if (message.data.status === 'error' && message.data.message) {
              setError(message.data.message)
            }
          }
        } catch (err) {
          console.error('Failed to parse WebSocket message:', err)
        }
      }

      ws.onerror = (event) => {
        console.error('WebSocket error:', event)
        setError('WebSocket connection error')
      }

      ws.onclose = () => {
        console.log('WebSocket disconnected')
        wsRef.current = null

        // Attempt to reconnect after 3 seconds
        if (!reconnectTimeoutRef.current) {
          reconnectTimeoutRef.current = setTimeout(() => {
            reconnectTimeoutRef.current = null
            setIsConnecting(true)
            connectWebSocket()
          }, 3000)
        }
      }

      return ws
    }

    const ws = connectWebSocket()

    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current)
        reconnectTimeoutRef.current = null
      }
      ws.close()
    }
  }, [isPaused])

  // Start streaming when component mounts
  useEffect(() => {
    const startStreaming = async () => {
      try {
        const response = await fetch('/api/manager-logs/stream/start', { method: 'POST' })
        if (response.ok) {
          const data = await response.json()
          if (data.success) {
            setStreamStatus('started')
          }
        }
      } catch (err) {
        console.error('Failed to start log streaming:', err)
      }
    }

    startStreaming()

    // Cleanup: stop streaming when unmounting
    return () => {
      fetch('/api/manager-logs/stream/stop', { method: 'POST' }).catch(() => {})
    }
  }, [])

  // Auto-scroll to bottom when new lines arrive
  useEffect(() => {
    if (autoScroll && logContainerRef.current && !isPaused) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight
    }
  }, [logLines, autoScroll, isPaused])

  // Detect manual scrolling
  const handleScroll = () => {
    if (logContainerRef.current) {
      const { scrollTop, scrollHeight, clientHeight } = logContainerRef.current
      const isAtBottom = scrollHeight - scrollTop - clientHeight < 50
      setAutoScroll(isAtBottom)
    }
  }

  const handlePauseToggle = async () => {
    if (!isPaused) {
      // Pausing - stop the stream
      try {
        const response = await fetch('/api/manager-logs/stream/stop', { method: 'POST' })
        if (response.ok) {
          setIsPaused(true)
          setStreamStatus('stopped')
        }
      } catch (err) {
        console.error('Failed to stop streaming:', err)
      }
    } else {
      // Resuming - start the stream
      try {
        const response = await fetch('/api/manager-logs/stream/start', { method: 'POST' })
        if (response.ok) {
          setIsPaused(false)
          setStreamStatus('started')
        }
      } catch (err) {
        console.error('Failed to start streaming:', err)
      }
    }
  }

  const handleClear = () => {
    setLogLines([])
    seenLinesRef.current.clear()
  }

  // Export logs to file
  const handleExport = useCallback(() => {
    if (logLines.length === 0) return

    // Strip ANSI codes for clean export
    const stripAnsi = (text: string) => text.replace(/\u001b\[[0-9;]+m/g, '')
    const cleanLogs = logLines.map(stripAnsi).join('\n')

    const blob = new Blob([cleanLogs], { type: 'text/plain' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = `bluepilot-logs-${new Date().toISOString().replace(/[:.]/g, '-')}.txt`
    document.body.appendChild(link)
    link.click()
    document.body.removeChild(link)
    URL.revokeObjectURL(url)
  }, [logLines])

  const handleRefresh = async () => {
    setIsConnecting(true)
    setError(null)

    // Fetch fresh logs
    try {
      const response = await fetch('/api/manager-logs')
      if (response.ok) {
        const data: LogResponse = await response.json()
        if (data.success && data.output) {
          const lines = data.output.split('\n').filter(line => line.trim())
          setLogLines(lines)
        }
      }
    } catch (err) {
      setError('Failed to fetch logs')
    } finally {
      setIsConnecting(false)
    }
  }

  // Filter lines based on search query
  const filteredLines = searchQuery
    ? logLines.filter(line => line.toLowerCase().includes(searchQuery.toLowerCase()))
    : logLines

  // Parse ANSI colors for display
  const displayContent = useMemo(() => {
    return filteredLines.map((line, index) => (
      <div key={`line-${index}`}>
        {parseAnsiColors(line)}
      </div>
    ))
  }, [filteredLines])

  return (
    <>
      <Header deviceStatus={deviceStatus} subtitle="View real-time system logs" />
      <div className="logs-view">
        <div className="logs-controls">
          <div className="search-container">
            <input
              type="text"
              className="search-input"
              placeholder="Search logs..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
            />
            {searchQuery && (
              <button
                className="search-clear"
                onClick={() => setSearchQuery('')}
                aria-label="Clear search"
              >
                <Icon name="close" size={18} />
              </button>
            )}
          </div>

          <div className="control-buttons">
            <Button
              variant={isPaused ? 'primary' : 'secondary'}
              size="small"
              onClick={handlePauseToggle}
              icon={<Icon name={isPaused ? 'play_arrow' : 'pause'} size={18} />}
            >
              {isPaused ? 'Resume' : 'Pause'}
            </Button>

            <Button
              variant="secondary"
              size="small"
              onClick={handleClear}
              icon={<Icon name="delete" size={18} />}
            >
              Clear
            </Button>

            <Button
              variant="primary"
              size="small"
              onClick={handleRefresh}
              disabled={isConnecting}
              icon={<Icon name="refresh" size={18} />}
            >
              Refresh
            </Button>

            <Button
              variant="secondary"
              size="small"
              onClick={handleExport}
              disabled={logLines.length === 0}
              icon={<Icon name="download" size={18} />}
            >
              Export
            </Button>

            <ToggleSwitch
              checked={autoScroll}
              onChange={setAutoScroll}
              label="Auto-scroll"
              size="compact"
              className="logs-toggle"
            />
          </div>
        </div>

        <div className="logs-content">
          {error && (
            <div className="console-error">
              {error}
            </div>
          )}

          <div className="console-status">
            <span className="console-status-label">Status:</span>
            <span className={`console-status-value ${streamStatus === 'started' && !isPaused ? 'streaming' : 'paused'}`}>
              {isPaused ? 'Paused' : streamStatus === 'started' ? 'Streaming' : isConnecting ? 'Connecting...' : 'Stopped'}
            </span>
            {searchQuery && (
              <>
                <span className="console-status-separator">•</span>
                <span className="console-status-label">Showing:</span>
                <span className="console-status-value">
                  {filteredLines.length} / {logLines.length} lines
                </span>
              </>
            )}
            <span className="console-status-separator">•</span>
            <span className="console-status-label">Total:</span>
            <span className="console-status-value">{logLines.length} lines</span>
          </div>

          <pre
            ref={logContainerRef}
            className="console-output"
            onScroll={handleScroll}
            aria-live={isPaused ? 'off' : 'polite'}
          >
            {displayContent.length > 0 ? displayContent : (isConnecting ? 'Connecting to log stream...' : 'No logs available')}
          </pre>
        </div>
      </div>

      <BackToTop />
    </>
  )
}
