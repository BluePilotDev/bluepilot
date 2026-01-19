/**
 * Static Param Display Control
 * Displays a parameter value in read-only mode
 */

import { useParamsStore } from '@/stores/useParamsStore'
import type { StaticParamDisplayControl as StaticParamDisplayControlType } from '@/types/panels'
import { ControlCard } from '@/components/common'
import './StaticParamDisplayControl.css'

interface StaticParamDisplayControlProps {
  control: StaticParamDisplayControlType
}

export function StaticParamDisplayControl({ control }: StaticParamDisplayControlProps) {
  const params = useParamsStore((store) => store.params)
  const paramValue = params[control.param]?.value

  // Format the value for display
  const displayValue = paramValue !== undefined && paramValue !== null
    ? String(paramValue)
    : 'Not Set'

  return (
    <ControlCard
      title={control.title}
      description={control.desc}
      className="static-param-display-control"
    >
      <div className="static-param-display-value">
        <span className="static-param-display-key">{control.param}</span>
        <span className="static-param-display-text">{displayValue}</span>
      </div>
    </ControlCard>
  )
}
