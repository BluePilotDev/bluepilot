/**
 * Floating Changes Indicator
 * Shows pending changes count and provides save/discard actions
 */

import { useState } from 'react'
import { useChangeTrackingStore } from '@/stores/useChangeTrackingStore'
import { useParamsStore } from '@/stores/useParamsStore'
import { Icon, Button } from '@/components/common'
import { ChangesSummaryModal } from './ChangesSummaryModal'
import './FloatingChangesIndicator.css'

export function FloatingChangesIndicator() {
  const { hasChanges, getChangeCount, isSaving, commitAllChanges, discardAllChanges, autoSaveEnabled, setAutoSave } =
    useChangeTrackingStore()
  const { applyStagedParams, clearStagedParams } = useParamsStore()
  const [showSummary, setShowSummary] = useState(false)

  const changeCount = getChangeCount()

  const handleSaveAll = async () => {
    const result = await commitAllChanges()
    if (result.success) {
      applyStagedParams()
    }
  }

  const handleDiscardAll = () => {
    discardAllChanges()
    clearStagedParams()
  }

  // Show auto-save indicator when enabled (even with no pending changes)
  if (autoSaveEnabled) {
    return (
      <>
        <div className="floating-changes-indicator floating-changes-indicator--autosave">
          <div className="floating-changes-content">
            <div className="floating-changes-info">
              <div className="floating-changes-badge floating-changes-badge--autosave">
                <Icon name="bolt" size={18} />
              </div>
              <span className="floating-changes-text">Real-time tuning mode</span>
            </div>

            <div className="floating-changes-actions">
              <Button
                variant="ghost"
                size="small"
                onClick={() => setAutoSave(false)}
              >
                Disable
              </Button>
            </div>
          </div>
        </div>

        <ChangesSummaryModal
          isOpen={showSummary}
          onClose={() => setShowSummary(false)}
          onSave={handleSaveAll}
          onDiscard={handleDiscardAll}
        />
      </>
    )
  }

  if (!hasChanges()) {
    return null
  }

  return (
    <>
      <div className="floating-changes-indicator">
        <div className="floating-changes-content">
          <div className="floating-changes-info">
            <div className="floating-changes-badge">
              <Icon name="edit" size={18} />
              <span className="floating-changes-count">{changeCount}</span>
            </div>
            <span className="floating-changes-text">
              {changeCount === 1 ? 'unsaved change' : 'unsaved changes'}
            </span>
          </div>

          <div className="floating-changes-actions">
            <button
              type="button"
              className="floating-changes-review-btn"
              onClick={() => setShowSummary(true)}
              aria-label="Review changes"
            >
              Review
            </button>

            <Button
              variant="ghost"
              size="small"
              onClick={handleDiscardAll}
              disabled={isSaving}
            >
              Discard
            </Button>

            <Button
              variant="primary"
              size="small"
              onClick={handleSaveAll}
              loading={isSaving}
            >
              Save All
            </Button>
          </div>
        </div>
      </div>

      <ChangesSummaryModal
        isOpen={showSummary}
        onClose={() => setShowSummary(false)}
        onSave={handleSaveAll}
        onDiscard={handleDiscardAll}
      />
    </>
  )
}
