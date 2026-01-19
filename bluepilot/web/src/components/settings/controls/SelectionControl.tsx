/**
 * Selection Control Component
 * Renders a dropdown/select for choosing from multiple options
 */

import type { SelectionControl as SelectionControlType } from '@/types/panels'
import { useParamsStore } from '@/stores/useParamsStore'
import { usePanelStateStore } from '@/stores/usePanelStateStore'
import { useSettingsContext } from '@/contexts/SettingsContext'
import { useStagedParam } from '@/hooks/useStagedParam'
import { getDynamicDescription, evaluateConditions } from '@/utils/conditionalEvaluator'
import { ControlCard } from '@/components/common'
import './SelectionControl.css'

interface SelectionControlProps {
  control: SelectionControlType
  disabled?: boolean
  disabledReason?: string | null
}

export function SelectionControl({ control, disabled, disabledReason }: SelectionControlProps) {
  const { params } = useParamsStore()
  const panelState = usePanelStateStore((state) => state.state)
  const settingsContext = useSettingsContext()

  // Use staged param hook for change tracking
  const { value, stageValue } = useStagedParam({
    param: control.param,
    controlTitle: control.title,
    controlType: 'selection',
    panelId: settingsContext?.panelId || '',
    panelName: settingsContext?.panelName || '',
    groupName: settingsContext?.groupName || '',
  })

  // Get current value and normalize to string for comparison (use staged value)
  const currentValue = value !== null && value !== undefined ? String(value) : ''

  // Get dynamic description
  const description = getDynamicDescription(control, panelState, params)

  // Filter options based on enableConditions
  const availableOptions = control.options.filter((option) => {
    if (!option.enableConditions) return true
    return evaluateConditions(option.enableConditions, panelState, params)
  })

  // Get unit based on metric setting
  const isMetric = params['IsMetric']?.value === true
  const unit = isMetric && control.unitMetric ? control.unitMetric : control.unit

  const handleChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    stageValue(e.target.value)
  }

  return (
    <ControlCard
      title={control.title}
      description={description}
      disabled={disabled}
      disabledReason={disabledReason}
      className="selection-control"
      footer={
        <select
          className="selection-control__select"
          value={currentValue}
          onChange={handleChange}
          disabled={disabled || availableOptions.length === 0}
        >
          {availableOptions.map((option) => {
            const displayName = unit ? option.name.replace('{unit}', unit) : option.name
            const optionValue = String(option.value)
            return (
              <option key={option.value} value={optionValue}>
                {displayName}
              </option>
            )
          })}
        </select>
      }
    />
  )
}
