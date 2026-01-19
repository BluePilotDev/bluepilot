import { useState } from 'react'
import type { RecentChangesControl as RecentChangesControlType } from '@/types/panels'
import { Button, ControlCard, Modal } from '@/components/common'
import './RecentChangesControl.css'

type ChangeSource = {
  id: 'bluepilot' | 'sunnypilot'
  label: string
  subtitle: string
  path: string
}

const CHANGE_SOURCES: ChangeSource[] = [
  {
    id: 'bluepilot',
    label: 'BluePilot Changes',
    subtitle: 'Latest updates from BluePilot',
    path: 'CHANGELOG.md',
  },
  {
    id: 'sunnypilot',
    label: 'SunnyPilot Changes',
    subtitle: 'Upstream SunnyPilot updates',
    path: 'CHANGELOG_SP.md',
  },
]

interface Props {
  control: RecentChangesControlType
  disabled?: boolean
  disabledReason?: string | null
}

export function RecentChangesControl({ control, disabled, disabledReason }: Props) {
  const [activeSource, setActiveSource] = useState<ChangeSource | null>(null)
  const [content, setContent] = useState('')
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)

  const openSource = async (source: ChangeSource) => {
    if (disabled) return

    setActiveSource(source)
    setLoading(true)
    setError(null)
    setContent('')

    try {
      const response = await fetch(`/api/file-content?path=${encodeURIComponent(source.path)}`)

      if (!response.ok) {
        throw new Error('Failed to load change log')
      }

      const data = await response.json()
      if (data.success) {
        setContent(data.content || 'File is empty')
      } else {
        throw new Error(data.error || 'Failed to load change log')
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load change log')
    } finally {
      setLoading(false)
    }
  }

  const closeModal = () => {
    setActiveSource(null)
    setContent('')
    setError(null)
  }

  return (
    <>
      <ControlCard
        title={control.title}
        description={control.desc}
        disabled={disabled}
        disabledReason={disabledReason}
        className="recent-changes-control"
      >
        <div className="recent-changes-buttons">
          {CHANGE_SOURCES.map((source) => (
            <Button
              key={source.id}
              variant="secondary"
              size="large"
              className={`recent-changes-btn recent-changes-btn--${source.id}`}
              onClick={() => openSource(source)}
              disabled={disabled}
            >
              <span className="recent-changes-btn__label">{source.label}</span>
              <span className="recent-changes-btn__subtitle">{source.subtitle}</span>
            </Button>
          ))}
        </div>
      </ControlCard>

      <Modal
        isOpen={activeSource !== null}
        title={activeSource?.label || 'Changes'}
        onClose={closeModal}
        maxWidth="900px"
      >
        <div className="recent-changes-modal">
          {loading && <div className="recent-changes-loading">Loading...</div>}
          {error && !loading && (
            <div className="recent-changes-error">
              <strong>Error:</strong> {error}
            </div>
          )}
          {!loading && !error && <pre className="recent-changes-content">{content}</pre>}
        </div>
      </Modal>
    </>
  )
}
