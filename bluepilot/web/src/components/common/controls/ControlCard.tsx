import type { CSSProperties, ReactNode } from 'react'
import './ControlCard.css'

interface ControlCardProps {
  title: string
  description?: string | ReactNode | null
  disabled?: boolean
  disabledReason?: string | null
  layout?: 'stacked' | 'inline'
  aside?: ReactNode
  footer?: ReactNode
  meta?: ReactNode
  children?: ReactNode
  className?: string
  style?: CSSProperties
}

/**
 * ControlCard provides a shared visual shell for panel controls so they
 * match the Qt UI layout/spacing.
 */
export const ControlCard = ({
  title,
  description,
  disabled,
  disabledReason,
  layout = 'stacked',
  aside,
  footer,
  meta,
  children,
  className = '',
  style,
}: ControlCardProps) => {
  const classes = [
    'control-card',
    `control-card--${layout}`,
    disabled ? 'control-card--disabled' : '',
    className,
  ]
    .filter(Boolean)
    .join(' ')

  const renderDescription = () => {
    if (!description) return null
    if (typeof description === 'string') {
      return (
        <p
          className="control-card__description"
          dangerouslySetInnerHTML={{ __html: description }}
        />
      )
    }
    return <div className="control-card__description">{description}</div>
  }

  return (
    <div className={classes} style={style}>
      <div className="control-card__main">
        <div className="control-card__header">
          <div className="control-card__title-row">
            <h4 className="control-card__title">{title}</h4>
            {disabled && disabledReason && (
              <span className="control-card__badge">{disabledReason}</span>
            )}
          </div>
          {meta && <div className="control-card__meta">{meta}</div>}
        </div>
        {renderDescription()}
        {children}
      </div>
      {aside && <div className="control-card__aside">{aside}</div>}
      {footer && <div className="control-card__footer">{footer}</div>}
    </div>
  )
}
