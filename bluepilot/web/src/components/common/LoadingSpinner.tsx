import React from 'react'
import './LoadingSpinner.css'

interface LoadingSpinnerProps {
  size?: 'small' | 'medium' | 'large'
  message?: string
  fullPage?: boolean
}

export const LoadingSpinner: React.FC<LoadingSpinnerProps> = ({
  size = 'medium',
  message,
  fullPage = false,
}) => {
  const content = (
    <div className={`loading-spinner ${fullPage ? 'loading-spinner--fullpage' : ''}`}>
      <div className={`spinner spinner--${size}`}></div>
      {message && <p className="loading-message">{message}</p>}
    </div>
  )

  if (fullPage) {
    return (
      <div className="loading-overlay">
        {content}
      </div>
    )
  }

  return content
}
