import React, { useState, useEffect, useRef } from 'react'
import { Modal } from './Modal'
import './InputDialog.css'

interface InputDialogProps {
  isOpen: boolean
  onClose: () => void
  onSubmit: (value: string) => void
  title: string
  message?: string
  placeholder?: string
  defaultValue?: string
  isPassword?: boolean
  confirmText?: string
  cancelText?: string
}

export const InputDialog: React.FC<InputDialogProps> = ({
  isOpen,
  onClose,
  onSubmit,
  title,
  message,
  placeholder,
  defaultValue = '',
  isPassword = false,
  confirmText = 'OK',
  cancelText = 'Cancel',
}) => {
  const [value, setValue] = useState(defaultValue)
  const inputRef = useRef<HTMLInputElement>(null)

  // Reset value when dialog opens/closes or defaultValue changes
  useEffect(() => {
    setValue(defaultValue)
  }, [defaultValue, isOpen])

  // Focus input when dialog opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => {
        inputRef.current?.focus()
      }, 100)
    }
  }, [isOpen])

  const handleSubmit = () => {
    onSubmit(value)
    onClose()
  }

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      handleSubmit()
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
          onClick: handleSubmit,
          variant: 'primary',
        },
      ]}
    >
      {message && (
        <div className="input-dialog-message" dangerouslySetInnerHTML={{ __html: message }} />
      )}
      <input
        ref={inputRef}
        type={isPassword ? 'password' : 'text'}
        className="input-dialog-input"
        value={value}
        onChange={(e) => setValue(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder={placeholder}
      />
    </Modal>
  )
}
