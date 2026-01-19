import React from 'react'
import { Modal } from './Modal'
import { Icon } from './Icon'
import './ConfirmDialog.css'

interface ConfirmDialogProps {
  isOpen: boolean
  onClose: () => void
  onConfirm: () => void
  title: string
  message: string | React.ReactNode
  confirmText?: string
  cancelText?: string
  variant?: 'danger' | 'warning' | 'info'
}

export const ConfirmDialog: React.FC<ConfirmDialogProps> = ({
  isOpen,
  onClose,
  onConfirm,
  title,
  message,
  confirmText = 'Confirm',
  cancelText = 'Cancel',
  variant = 'info',
}) => {
  const handleConfirm = () => {
    onConfirm()
    onClose()
  }

  const getIcon = () => {
    switch (variant) {
      case 'danger':
        return 'error'
      case 'warning':
        return 'warning'
      case 'info':
        return 'info'
      default:
        return 'info'
    }
  }

  return (
    <Modal
      isOpen={isOpen}
      onClose={onClose}
      title={title}
      size="small"
      actions={[
        {
          label: cancelText,
          onClick: onClose,
          variant: 'secondary',
        },
        {
          label: confirmText,
          onClick: handleConfirm,
          variant: variant === 'danger' ? 'danger' : 'primary',
        },
      ]}
    >
      <div className={`confirm-dialog-message confirm-dialog-message--${variant}`}>
        <div className="confirm-dialog-icon">
          <Icon name={getIcon()} size={48} />
        </div>
        {typeof message === 'string' ? <p>{message}</p> : message}
      </div>
    </Modal>
  )
}
