import React, { useEffect } from 'react'
import { Icon } from './Icon'
import './Toast.css'

export interface ToastProps {
  message: string
  type?: 'success' | 'error' | 'info'
  duration?: number
  onClose: () => void
}

export const Toast: React.FC<ToastProps> = ({ message, type = 'success', duration, onClose }) => {
  useEffect(() => {
    // Only auto-dismiss if duration is provided
    if (duration !== undefined) {
      const timer = setTimeout(() => {
        onClose()
      }, duration)

      return () => clearTimeout(timer)
    }
  }, [duration, onClose])

  const getIconName = () => {
    switch (type) {
      case 'success':
        return 'check_circle'
      case 'error':
        return 'error'
      case 'info':
        return 'info'
      default:
        return 'check_circle'
    }
  }

  return (
    <div className={`toast toast--${type}`}>
      <span className="toast__icon">
        <Icon name={getIconName()} size={20} />
      </span>
      <span className="toast__message">{message}</span>
      <button
        className="toast__close"
        onClick={onClose}
        aria-label="Close notification"
        type="button"
      >
        <Icon name="close" size={16} />
      </button>
    </div>
  )
}

interface ToastContainerProps {
  toasts: Array<{ id: string; message: string; type?: 'success' | 'error' | 'info'; duration?: number }>
  onRemove: (id: string) => void
}

export const ToastContainer: React.FC<ToastContainerProps> = ({ toasts, onRemove }) => {
  return (
    <div className="toast-container">
      {toasts.map((toast) => (
        <Toast
          key={toast.id}
          message={toast.message}
          type={toast.type}
          duration={toast.duration}
          onClose={() => onRemove(toast.id)}
        />
      ))}
    </div>
  )
}
