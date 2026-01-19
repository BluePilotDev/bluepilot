import { useState } from 'react'
import type { RestartUIControl as RestartUIControlType } from '@/types/panels'
import { panelAPI } from '@/services/panelAPI'
import { Button, ControlCard, Modal } from '@/components/common'
import './RestartUIControl.css'

interface Props {
  control: RestartUIControlType
  disabled?: boolean
  disabledReason?: string | null
}

export function RestartUIControl({ control, disabled, disabledReason }: Props) {
  const [showConfirm, setShowConfirm] = useState(false)
  const [loading, setLoading] = useState(false)
  const [result, setResult] = useState<{ success: boolean; message: string } | null>(null)

  const buttonLabel = control.button_text || 'Restart UI'
  const confirmRequired = control.confirm !== undefined ? control.confirm : true

  const triggerRestart = async () => {
    setShowConfirm(false)
    setLoading(true)
    setResult(null)

    try {
      const response = await panelAPI.executePanelCommand({ action: 'restart_ui' })
      setResult({
        success: response.success,
        message: response.error ||
          (response.success
            ? 'UI restart requested. The interface will reload in a few seconds.'
            : 'Failed to restart the UI.'),
      })
    } catch (error) {
      setResult({
        success: false,
        message: error instanceof Error ? error.message : 'Failed to restart the UI.',
      })
    } finally {
      setLoading(false)
    }
  }

  const handleClick = () => {
    if (disabled) return
    if (confirmRequired) {
      setShowConfirm(true)
    } else {
      triggerRestart()
    }
  }

  return (
    <>
      <ControlCard
        title={control.title}
        description={control.desc}
        disabled={disabled}
        disabledReason={disabledReason}
        className="restart-ui-control"
      >
        <div className="restart-ui-actions">
          <Button
            className="restart-ui-btn"
            variant="primary"
            onClick={handleClick}
            loading={loading}
            disabled={disabled}
          >
            {buttonLabel}
          </Button>
          <p className="restart-ui-hint">
            The UI process will briefly stop and automatically relaunch. Vehicle controls are unaffected.
          </p>
        </div>
        {result && (
          <div className={`restart-ui-result ${result.success ? 'success' : 'error'}`}>
            {result.message}
          </div>
        )}
      </ControlCard>

      {showConfirm && (
        <Modal
          isOpen={showConfirm}
          title={control.title || 'Restart Interface'}
          onClose={() => setShowConfirm(false)}
          actions={[
            {
              label: control.confirm_no_text || 'Cancel',
              variant: 'secondary',
              onClick: () => setShowConfirm(false),
            },
            {
              label: control.confirm_yes_text || 'Restart',
              variant: 'primary',
              onClick: triggerRestart,
            },
          ]}
        >
          <p>{control.confirm_text || 'Are you sure you want to restart the UI? The screen will refresh shortly.'}</p>
        </Modal>
      )}
    </>
  )
}
