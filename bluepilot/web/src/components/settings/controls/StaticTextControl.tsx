/**
 * Static Text Control
 * Displays static informational text (no interaction)
 */

import type { StaticTextControl as StaticTextControlType } from '@/types/panels'
import { ControlCard } from '@/components/common'
import './StaticTextControl.css'

interface StaticTextControlProps {
  control: StaticTextControlType
}

export function StaticTextControl({ control }: StaticTextControlProps) {
  const description = control.desc
    ? control.desc.split('\n').map((line, index) => (
        <p key={`${control.title}-${index}`}>{line}</p>
      ))
    : null

  return (
    <ControlCard
      title={control.title}
      description={description && <div className="static-text-desc">{description}</div>}
      className="static-text-control"
    />
  )
}
