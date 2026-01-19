import { useRef, useState } from 'react'
import './Timeline.css'

interface TimelineProps {
  currentTime: number
  duration: number
  currentSegment: number
  totalSegments: number
  onSeek: (time: number) => void
  onSegmentChange: (segment: number) => void
}

export const Timeline: React.FC<TimelineProps> = ({
  currentTime,
  duration,
  currentSegment,
  totalSegments,
  onSeek,
  onSegmentChange,
}) => {
  const timelineRef = useRef<HTMLDivElement>(null)
  const [isDragging, setIsDragging] = useState(false)
  const [hoverTime, setHoverTime] = useState<number | null>(null)

  const handleSeek = (e: React.MouseEvent<HTMLDivElement>) => {
    const timeline = timelineRef.current
    if (!timeline || !duration) return

    const rect = timeline.getBoundingClientRect()
    const x = e.clientX - rect.left
    const percent = Math.max(0, Math.min(1, x / rect.width))
    const time = percent * duration

    onSeek(time)
  }

  const handleMouseDown = (e: React.MouseEvent<HTMLDivElement>) => {
    setIsDragging(true)
    handleSeek(e)
  }

  const handleMouseMove = (e: React.MouseEvent<HTMLDivElement>) => {
    const timeline = timelineRef.current
    if (!timeline || !duration) return

    const rect = timeline.getBoundingClientRect()
    const x = e.clientX - rect.left
    const percent = Math.max(0, Math.min(1, x / rect.width))
    const time = percent * duration

    setHoverTime(time)

    if (isDragging) {
      onSeek(time)
    }
  }

  const handleMouseUp = () => {
    setIsDragging(false)
  }

  const handleMouseLeave = () => {
    setHoverTime(null)
    setIsDragging(false)
  }

  const progressPercent = duration > 0 ? (currentTime / duration) * 100 : 0

  return (
    <div className="timeline-container">
      {/* Segment selector */}
      {totalSegments > 1 && (
        <div className="segment-selector">
          {Array.from({ length: totalSegments }, (_, i) => (
            <button
              key={i}
              className={`segment-btn ${i === currentSegment ? 'active' : ''}`}
              onClick={() => onSegmentChange(i)}
              title={`Segment ${i + 1}`}
            >
              {i + 1}
            </button>
          ))}
        </div>
      )}

      {/* Timeline scrubber */}
      <div
        ref={timelineRef}
        className="timeline"
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseLeave}
      >
        <div className="timeline-background">
          <div className="timeline-progress" style={{ width: `${progressPercent}%` }} />

          {hoverTime !== null && (
            <div
              className="timeline-hover-indicator"
              style={{
                left: `${(hoverTime / duration) * 100}%`,
              }}
            >
              <div className="timeline-hover-time">{formatTime(hoverTime)}</div>
            </div>
          )}
        </div>

        <div
          className="timeline-handle"
          style={{
            left: `${progressPercent}%`,
          }}
        />
      </div>
    </div>
  )
}

function formatTime(seconds: number): string {
  if (!seconds || !isFinite(seconds)) return '0:00'

  const mins = Math.floor(seconds / 60)
  const secs = Math.floor(seconds % 60)
  return `${mins}:${secs.toString().padStart(2, '0')}`
}
