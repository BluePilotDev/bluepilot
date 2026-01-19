/**
 * File Viewer Control Component
 * Displays file contents in a modal (logs, changelogs, etc.)
 */

import { useState } from 'react'
import type { FileViewerControl } from '@/types/panels'
import { Button, ControlCard, Modal } from '@/components/common'
import './FileViewer.css'

interface FileViewerProps {
  control: FileViewerControl
  disabled?: boolean
}

export function FileViewer({ control, disabled }: FileViewerProps) {
  const [showModal, setShowModal] = useState(false)
  const [content, setContent] = useState<string>('')
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)

  const handleOpen = async () => {
    setShowModal(true)
    setLoading(true)
    setError(null)

    try {
      // Fetch file content from backend
      const response = await fetch(`/api/file-content?path=${encodeURIComponent(control.path)}`)

      if (!response.ok) {
        throw new Error('Failed to load file')
      }

      const data = await response.json()

      if (data.success) {
        setContent(data.content || 'File is empty')
      } else {
        setError(data.error || 'Failed to load file')
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load file')
    } finally {
      setLoading(false)
    }
  }

  const handleClose = () => {
    setShowModal(false)
    setContent('')
    setError(null)
  }

  return (
    <>
      <ControlCard
        title={control.title}
        description={control.desc}
        className="file-viewer-control"
        disabled={disabled}
        footer={
          <Button
            variant="secondary"
            onClick={handleOpen}
            disabled={disabled}
            className="file-viewer-btn"
          >
            {control.button_text || 'View'}
          </Button>
        }
      />

      {showModal && (
        <Modal
          isOpen={showModal}
          title={control.header || control.title}
          onClose={handleClose}
          maxWidth="800px"
        >
          <div className="file-viewer-modal-content">
            {loading && <div className="file-viewer-loading">Loading...</div>}

            {error && (
              <div className="file-viewer-error">
                <strong>Error:</strong> {error}
              </div>
            )}

            {!loading && !error && (
              <pre className="file-viewer-pre">{content}</pre>
            )}
          </div>
        </Modal>
      )}
    </>
  )
}
