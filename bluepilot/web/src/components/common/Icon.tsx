import React from 'react'
import './Icon.css'

interface IconProps {
  name: string
  size?: number | string
  className?: string
  style?: React.CSSProperties
  onClick?: () => void
  title?: string
}

export const Icon: React.FC<IconProps> = ({
  name,
  size = 24,
  className = '',
  style,
  onClick,
  title,
}) => {
  const iconStyle: React.CSSProperties = {
    fontSize: typeof size === 'number' ? `${size}px` : size,
    ...style,
  }

  return (
    <span
      className={`material-icons ${className}`}
      style={iconStyle}
      onClick={onClick}
      title={title}
    >
      {name}
    </span>
  )
}
