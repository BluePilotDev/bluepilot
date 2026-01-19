import type { ChangeEvent, InputHTMLAttributes } from 'react'
import './ToggleSwitch.css'

type ToggleSize = 'default' | 'compact'

interface ToggleSwitchProps
  extends Omit<InputHTMLAttributes<HTMLInputElement>, 'type' | 'onChange' | 'size'> {
  label?: string
  size?: ToggleSize
  alignLabel?: 'start' | 'end'
  onChange?: (checked: boolean, event: ChangeEvent<HTMLInputElement>) => void
  className?: string
}

export const ToggleSwitch = ({
  label,
  size = 'default',
  alignLabel = 'end',
  onChange,
  className = '',
  checked,
  disabled,
  ...inputProps
}: ToggleSwitchProps) => {
  const classes = [
    'bp-toggle-switch',
    `bp-toggle-switch--${size}`,
    label ? `bp-toggle-switch--label-${alignLabel}` : '',
    className,
  ]
    .filter(Boolean)
    .join(' ')

  const handleChange = (event: ChangeEvent<HTMLInputElement>) => {
    onChange?.(event.target.checked, event)
  }

  return (
    <label className={classes}>
      <input
        type="checkbox"
        checked={checked}
        disabled={disabled}
        onChange={handleChange}
        {...inputProps}
      />
      <span className="bp-toggle-switch__slider" aria-hidden="true" />
      {label && <span className="bp-toggle-switch__label">{label}</span>}
    </label>
  )
}
