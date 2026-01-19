/**
 * Number Control Component
 * Renders +/- buttons with a slider for integer and float values
 */

import { useState, useEffect, useCallback } from 'react'
import type { IntegerControl, FloatControl } from '@/types/panels'
import { useParamsStore } from '@/stores/useParamsStore'
import { usePanelStateStore } from '@/stores/usePanelStateStore'
import { useSettingsContext } from '@/contexts/SettingsContext'
import { useStagedParam } from '@/hooks/useStagedParam'
import { getDynamicDescription } from '@/utils/conditionalEvaluator'
import { ControlCard, Icon } from '@/components/common'
import './NumberControl.css'

interface NumberControlProps {
  control: IntegerControl | FloatControl
  disabled?: boolean
  disabledReason?: string | null
}

export function NumberControl({ control, disabled, disabledReason }: NumberControlProps) {
  const { params } = useParamsStore()
  const panelState = usePanelStateStore((state) => state.state)
  const settingsContext = useSettingsContext()

  const isFloat = control.type === 'float'
  const division = isFloat ? 1 : (control.division || 1)

  // Use staged param hook for change tracking
  const { value: stagedValue, stageValue } = useStagedParam({
    param: control.param,
    controlTitle: control.title,
    controlType: isFloat ? 'float' : 'integer',
    panelId: settingsContext?.panelId || '',
    panelName: settingsContext?.panelName || '',
    groupName: settingsContext?.groupName || '',
  })

  // Get current value (use staged value)
  // For floats: store the actual decimal value (0.70), no division scaling needed
  // For integers: use division to convert stored integer to display decimal (e.g., stored 70 / 100 = display 0.70)
  const currentValue = stagedValue !== null && stagedValue !== undefined
    ? Number(stagedValue) / division
    : control.min

  const [localValue, setLocalValue] = useState(currentValue)

  useEffect(() => {
    setLocalValue(currentValue)
  }, [currentValue])

  // Get dynamic description
  const description = getDynamicDescription(control, panelState, params)

  // Get unit based on metric setting
  const isMetric = params['IsMetric']?.value === true
  const unit = isMetric && control.unitMetric ? control.unitMetric : control.unit

  // Decimal places for display
  const decimalPlaces = isFloat ? 2 : 0

  // Stage value (instead of saving to backend)
  const stageNumberValue = useCallback((value: number) => {
    const clamped = Math.max(control.min, Math.min(control.max, value))
    setLocalValue(clamped)

    // For floats: store the display value directly (0.70)
    // For integers: multiply by division and round (display 0.70 * 100 = stored 70)
    const valueToStore = isFloat ? clamped : Math.round(clamped * division)
    stageValue(String(valueToStore))
  }, [control.min, control.max, division, isFloat, stageValue])

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value)
    if (!isNaN(value)) {
      setLocalValue(value)
    }
  }

  const handleInputBlur = () => {
    stageNumberValue(localValue)
  }

  // Simple increment/decrement on click
  const increment = useCallback(() => {
    const newValue = Math.min(control.max, localValue + control.increment)
    setLocalValue(newValue)
    const valueToStore = isFloat ? newValue : Math.round(newValue * division)
    stageValue(String(valueToStore))
  }, [control.max, control.increment, division, isFloat, localValue, stageValue])

  const decrement = useCallback(() => {
    const newValue = Math.max(control.min, localValue - control.increment)
    setLocalValue(newValue)
    const valueToStore = isFloat ? newValue : Math.round(newValue * division)
    stageValue(String(valueToStore))
  }, [control.min, control.increment, division, isFloat, localValue, stageValue])

  // Slider change handler
  const handleSliderChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = parseFloat(e.target.value)
    setLocalValue(value)
  }

  // Stage on slider release
  const handleSliderRelease = () => {
    stageNumberValue(localValue)
  }

  return (
    <ControlCard
      title={control.title}
      description={description}
      disabled={disabled}
      disabledReason={disabledReason}
      className="number-control"
    >
      <div className="number-control__controls">
        <button
          type="button"
          className="number-control__btn number-control__btn--minus"
          onClick={decrement}
          disabled={disabled || localValue <= control.min}
          aria-label="Decrease value"
        >
          <Icon name="remove" size={24} />
        </button>

        <div className="number-control__value">
          <input
            type="number"
            className="number-control__input"
            value={localValue.toFixed(decimalPlaces)}
            onChange={handleInputChange}
            onBlur={handleInputBlur}
            disabled={disabled}
            min={control.min}
            max={control.max}
            step={control.increment}
            aria-label={`${control.title} value`}
          />
          {unit && <span className="number-control__unit">{unit}</span>}
        </div>

        <button
          type="button"
          className="number-control__btn number-control__btn--plus"
          onClick={increment}
          disabled={disabled || localValue >= control.max}
          aria-label="Increase value"
        >
          <Icon name="add" size={24} />
        </button>
      </div>

      {/* Slider for quick adjustments */}
      <div className="number-control__slider-container">
        <span className="number-control__slider-label">{control.min}</span>
        <input
          type="range"
          className="number-control__slider"
          min={control.min}
          max={control.max}
          step={control.increment}
          value={localValue}
          onChange={handleSliderChange}
          onMouseUp={handleSliderRelease}
          onTouchEnd={handleSliderRelease}
          disabled={disabled}
          aria-label={`${control.title} slider`}
        />
        <span className="number-control__slider-label">{control.max}</span>
      </div>
    </ControlCard>
  )
}
