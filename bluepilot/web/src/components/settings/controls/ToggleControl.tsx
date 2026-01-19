/**
 * Toggle Control Component
 * Renders a boolean toggle switch for panel settings
 */

import { useState } from 'react'
import type { ToggleControl as ToggleControlType } from '@/types/panels'
import { useParamsStore } from '@/stores/useParamsStore'
import { usePanelStateStore } from '@/stores/usePanelStateStore'
import { useSettingsContext } from '@/contexts/SettingsContext'
import { useStagedParam } from '@/hooks/useStagedParam'
import { getDynamicDescription, getDynamicTitle, getDynamicStyle } from '@/utils/conditionalEvaluator'
import { ControlCard, ConfirmDialog, ToggleSwitch } from '@/components/common'
import './ToggleControl.css'

interface ToggleControlProps {
  control: ToggleControlType
  disabled?: boolean
  disabledReason?: string | null
}

export function ToggleControl({ control, disabled, disabledReason }: ToggleControlProps) {
  const { params } = useParamsStore()
  const panelState = usePanelStateStore((state) => state.state)
  const settingsContext = useSettingsContext()
  const [showConfirm, setShowConfirm] = useState(false)
  const [pendingToggleValue, setPendingToggleValue] = useState<boolean | null>(null)

  // Use staged param hook for change tracking
  const { value, stageValue } = useStagedParam({
    param: control.param,
    controlTitle: control.title,
    controlType: 'toggle',
    panelId: settingsContext?.panelId || '',
    panelName: settingsContext?.panelName || '',
    groupName: settingsContext?.groupName || '',
  })

  // Get current value (use staged value if available)
  const isEnabled = value === true || value === '1' || value === 1

  // Get dynamic content
  const title = getDynamicTitle(control, panelState, params)
  const description = getDynamicDescription(control, panelState, params)
  const style = getDynamicStyle(control, panelState, params)

  const handleToggle = () => {
    const newValue = !isEnabled

    // Show confirmation if required
    if (control.confirm || control.confirmation) {
      setPendingToggleValue(newValue)
      setShowConfirm(true)
      return
    }

    // Otherwise, stage the change
    stageValue(newValue ? '1' : '0')
  }

  const handleConfirm = () => {
    if (pendingToggleValue !== null) {
      stageValue(pendingToggleValue ? '1' : '0')
      setPendingToggleValue(null)
    }
    setShowConfirm(false)
  }

  const handleCancel = () => {
    setPendingToggleValue(null)
    setShowConfirm(false)
  }

  return (
    <>
      <ControlCard
        title={title}
        description={description}
        disabled={disabled}
        disabledReason={disabledReason}
        layout="inline"
        className="toggle-control-card"
        style={style}
        aside={
          <ToggleSwitch
            checked={isEnabled}
            onChange={() => handleToggle()}
            disabled={disabled}
            aria-label={title}
          />
        }
      />

      <ConfirmDialog
        isOpen={showConfirm}
        onClose={handleCancel}
        onConfirm={handleConfirm}
        title={`${pendingToggleValue ? 'Enable' : 'Disable'} ${title}`}
        message={
          control.confirm_text ? (
            <div dangerouslySetInnerHTML={{ __html: control.confirm_text }} />
          ) : (
            `Are you sure you want to ${pendingToggleValue ? 'enable' : 'disable'} ${title}?`
          )
        }
        confirmText={control.confirm_yes_text || (pendingToggleValue ? 'Enable' : 'Disable')}
        cancelText={control.confirm_no_text || 'Cancel'}
        variant={style?.backgroundColor?.toLowerCase().includes('e22c2c') ? 'danger' : 'warning'}
      />
    </>
  )
}
