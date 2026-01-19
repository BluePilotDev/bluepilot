/**
 * Platform Display Control
 * Displays platform/vehicle information with custom styling
 */

import { useParamsStore } from '@/stores/useParamsStore'
import type { PlatformDisplayControl as PlatformDisplayControlType } from '@/types/panels'
import { ControlCard } from '@/components/common'
import './PlatformDisplayControl.css'

interface PlatformDisplayControlProps {
  control: PlatformDisplayControlType
}

export function PlatformDisplayControl({ control }: PlatformDisplayControlProps) {
  const params = useParamsStore((store) => store.params)
  const paramValue = params[control.value_param]?.value

  // Format the value for display
  const displayValue = paramValue !== undefined && paramValue !== null
    ? String(paramValue)
    : 'Not Detected'

  // Determine status color based on indicators in the value
  const getStatusClass = () => {
    const value = String(displayValue)
    if (value.includes('🟢')) return 'status-auto'
    if (value.includes('🔵')) return 'status-manual'
    if (value.includes('🟡')) return 'status-unknown'
    return ''
  }

  return (
    <ControlCard
      title={control.title}
      description={control.desc}
      className={`platform-display-control ${getStatusClass()}`}
    >
      <div
        className="platform-display-value"
        style={{ color: control.value_color || undefined }}
      >
        {displayValue}
      </div>
    </ControlCard>
  )
}
