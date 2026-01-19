/**
 * Changes Summary Modal
 * Displays all pending changes grouped by panel for review before saving
 */

import { useMemo } from 'react'
import { useChangeTrackingStore, type PendingChange } from '@/stores/useChangeTrackingStore'
import { Modal, Button, Icon, ToggleSwitch } from '@/components/common'
import './ChangesSummaryModal.css'

interface ChangesSummaryModalProps {
  isOpen: boolean
  onClose: () => void
  onSave: () => Promise<void>
  onDiscard: () => void
}

interface GroupedChanges {
  panelName: string
  panelId: string
  changes: PendingChange[]
}

function formatValue(value: string | number | boolean | null, controlType: string): string {
  if (value === null || value === undefined) return 'Not set'

  // Handle toggle values
  if (controlType === 'toggle') {
    return value === '1' || value === 1 || value === true ? 'ON' : 'OFF'
  }

  // Handle numeric values
  if (typeof value === 'number') {
    return String(value)
  }

  return String(value)
}

export function ChangesSummaryModal({ isOpen, onClose, onSave, onDiscard }: ChangesSummaryModalProps) {
  const { getChangeSummary, discardChange, isSaving, saveError, clearSaveError, autoSaveEnabled, setAutoSave } =
    useChangeTrackingStore()

  const changes = getChangeSummary()

  // Group changes by panel
  const groupedChanges = useMemo(() => {
    const groups = new Map<string, GroupedChanges>()

    for (const change of changes) {
      const existing = groups.get(change.panelId)
      if (existing) {
        existing.changes.push(change)
      } else {
        groups.set(change.panelId, {
          panelName: change.panelName,
          panelId: change.panelId,
          changes: [change],
        })
      }
    }

    return Array.from(groups.values())
  }, [changes])

  const handleSave = async () => {
    clearSaveError()
    await onSave()
    if (!saveError) {
      onClose()
    }
  }

  const handleDiscard = () => {
    onDiscard()
    onClose()
  }

  const handleRevertChange = (param: string) => {
    discardChange(param)
    // If no more changes, close modal
    if (changes.length <= 1) {
      onClose()
    }
  }

  return (
    <Modal
      isOpen={isOpen}
      onClose={onClose}
      title="Review Changes"
      size="medium"
      maxWidth="600px"
    >
      <div className="changes-summary">
        {saveError && (
          <div className="changes-summary-error">
            <Icon name="error" size={18} />
            <span>{saveError}</span>
          </div>
        )}

        {groupedChanges.length === 0 ? (
          <div className="changes-summary-empty">
            <Icon name="check_circle" size={48} />
            <p>No pending changes</p>
          </div>
        ) : (
          <div className="changes-summary-list">
            {groupedChanges.map((group) => (
              <div key={group.panelId} className="changes-summary-group">
                <h3 className="changes-summary-group-title">{group.panelName}</h3>
                <div className="changes-summary-items">
                  {group.changes.map((change) => (
                    <div key={change.param} className="changes-summary-item">
                      <div className="changes-summary-item-info">
                        <span className="changes-summary-item-title">{change.controlTitle}</span>
                        <div className="changes-summary-item-values">
                          <span className="changes-summary-value changes-summary-value--old">
                            {formatValue(change.oldValue, change.controlType)}
                          </span>
                          <Icon name="arrow_forward" size={14} className="changes-summary-arrow" />
                          <span className="changes-summary-value changes-summary-value--new">
                            {formatValue(change.newValue, change.controlType)}
                          </span>
                        </div>
                      </div>
                      <button
                        type="button"
                        className="changes-summary-revert-btn"
                        onClick={() => handleRevertChange(change.param)}
                        aria-label={`Revert ${change.controlTitle}`}
                        title="Revert this change"
                      >
                        <Icon name="undo" size={16} />
                      </button>
                    </div>
                  ))}
                </div>
              </div>
            ))}
          </div>
        )}

        <div className="changes-summary-autosave">
          <div className="changes-summary-autosave-info">
            <span className="changes-summary-autosave-label">Real-time Tuning Mode</span>
            <span className="changes-summary-autosave-desc">
              When enabled, changes save immediately for real-time feedback
            </span>
          </div>
          <ToggleSwitch
            checked={autoSaveEnabled}
            onChange={(checked) => setAutoSave(checked)}
            aria-label="Enable real-time tuning mode"
          />
        </div>

        <div className="changes-summary-footer">
          <span className="changes-summary-count">
            {changes.length} {changes.length === 1 ? 'change' : 'changes'}
          </span>
          <div className="changes-summary-actions">
            <Button variant="ghost" onClick={handleDiscard} disabled={isSaving}>
              Discard All
            </Button>
            <Button variant="primary" onClick={handleSave} loading={isSaving}>
              Save {changes.length} {changes.length === 1 ? 'Change' : 'Changes'}
            </Button>
          </div>
        </div>
      </div>
    </Modal>
  )
}
