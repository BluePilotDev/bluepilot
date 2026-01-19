/**
 * Panel Group Component
 * Renders a group of controls within a panel
 */

import { useMemo, useState, useCallback } from 'react'
import type { PanelGroup as PanelGroupType, PanelState, PanelControl, SelectionOption } from '@/types/panels'
import { useParamsStore } from '@/stores/useParamsStore'
import { isControlVisible } from '@/utils/conditionalEvaluator'
import { Button, Modal, Icon } from '@/components/common'
import { DynamicControl } from './DynamicControl'
import './PanelGroup.css'

interface PanelGroupProps {
  group: PanelGroupType
  state: PanelState
  panelId?: string
}

/**
 * Get the fallback default value for a control based on its type.
 * Used when no {param}_default value exists in the params system.
 */
function getControlFallbackDefault(control: PanelControl): string | null {
  switch (control.type) {
    case 'toggle':
      // Toggles default to false (stored as "0")
      return '0'

    case 'selection':
    case 'segmented_control': {
      // Find the option marked as default
      const options = 'options' in control ? control.options as SelectionOption[] : []
      const defaultOption = options.find(opt => opt.default === true)
      return defaultOption?.value ?? options[0]?.value ?? null
    }

    case 'integer':
    case 'float': {
      // Use the min value as default for numeric controls
      if ('min' in control && control.min !== undefined) {
        return String(control.min)
      }
      return null
    }

    default:
      return null
  }
}

/**
 * Get the default value for a control.
 * First checks for {param}_default in the params store (Qt-style defaults),
 * then falls back to control-type-based defaults.
 */
function getControlDefaultValue(
  control: PanelControl,
  params: Record<string, { value: unknown }>
): string | null {
  // Only controls with params can be reset
  if (!('param' in control) || !control.param) {
    return null
  }

  // Check for {param}_default in the params store (matches Qt implementation)
  const defaultKey = `${control.param}_default`
  const defaultParam = params[defaultKey]
  if (defaultParam?.value !== undefined && defaultParam.value !== null && defaultParam.value !== '') {
    return String(defaultParam.value)
  }

  // Fall back to control-type-based defaults
  return getControlFallbackDefault(control)
}

export function PanelGroup({ group, state, panelId }: PanelGroupProps) {
  const { params, getEffectiveParams, updateParam } = useParamsStore()
  const [showResetConfirm, setShowResetConfirm] = useState(false)
  const [isResetting, setIsResetting] = useState(false)

  // Use effective params (with staged changes) for visibility checks
  const effectiveParams = getEffectiveParams()

  // Skip hidden groups
  if (group.hidden) {
    return null
  }

  // Check if any controls in this group are visible (using effective params for staged changes)
  const hasVisibleControls = useMemo(() => {
    return group.controls.some((control) => {
      // Check if control is hidden in web UI
      if ('webSupported' in control && control.webSupported === false) {
        return false
      }
      // Check visibility conditions
      return isControlVisible(control, state, effectiveParams)
    })
  }, [group.controls, state, effectiveParams])

  // Get all resettable controls in the group
  const resettableControls = useMemo(() => {
    return group.controls.filter((control) => {
      if ('webSupported' in control && control.webSupported === false) {
        return false
      }
      if (!('param' in control) || !control.param) {
        return false
      }
      return getControlDefaultValue(control, params) !== null
    })
  }, [group.controls, params])

  // Reset all controls to defaults
  const handleReset = useCallback(async () => {
    setIsResetting(true)
    try {
      const resetPromises = resettableControls.map((control) => {
        if ('param' in control && control.param) {
          const defaultValue = getControlDefaultValue(control, params)
          if (defaultValue !== null) {
            return updateParam(control.param, defaultValue)
          }
        }
        return Promise.resolve()
      })
      await Promise.all(resetPromises)
    } finally {
      setIsResetting(false)
      setShowResetConfirm(false)
    }
  }, [resettableControls, params, updateParam])

  // Hide the entire group if no controls are visible
  if (!hasVisibleControls) {
    return null
  }

  return (
    <div className="panel-group">
      <div className="panel-group-header">
        <h3 className="panel-group-title">{group.title}</h3>
        {group.enableResetButton && resettableControls.length > 0 && (
          <Button
            variant="secondary"
            size="small"
            className="panel-group-reset"
            title="Reset to defaults"
            type="button"
            onClick={() => setShowResetConfirm(true)}
            icon={<Icon name="refresh" size={16} />}
          >
            Reset
          </Button>
        )}
      </div>

      <div className="panel-group-controls">
        {group.controls.map((control, index) => (
          <DynamicControl
            key={`${group.groupName}-${index}`}
            control={control}
            state={state}
            panelId={panelId}
            groupName={group.groupName}
          />
        ))}
      </div>

      {/* Reset Confirmation Modal */}
      <Modal
        isOpen={showResetConfirm}
        title="Reset Group Settings"
        onClose={() => setShowResetConfirm(false)}
      >
        <div className="panel-group-reset-modal">
          <p>
            Are you sure you want to reset all <strong>{resettableControls.length}</strong> settings
            in "{group.title}" to their default values?
          </p>
          <p className="reset-warning">This action cannot be undone.</p>
          <div className="reset-modal-actions">
            <Button
              variant="secondary"
              onClick={() => setShowResetConfirm(false)}
              disabled={isResetting}
            >
              Cancel
            </Button>
            <Button
              variant="danger"
              onClick={handleReset}
              disabled={isResetting}
            >
              {isResetting ? 'Resetting...' : 'Reset to Defaults'}
            </Button>
          </div>
        </div>
      </Modal>
    </div>
  )
}
