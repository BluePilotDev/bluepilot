/**
 * Command Button Control Component
 * Renders a button that executes commands or actions
 */

import { useMemo, useState, useEffect } from 'react'
import type { CommandButtonControl } from '@/types/panels'
import { usePanelStateStore } from '@/stores/usePanelStateStore'
import { panelAPI } from '@/services/panelAPI'
import { getDynamicDescription } from '@/utils/conditionalEvaluator'
import { Button, ControlCard, Modal, InputDialog, ConfirmDialog } from '@/components/common'
import './CommandButton.css'

const DEVICE_UI_ACTIONS = new Set([
  'show_driver_camera',
  'show_training_guide',
  'show_language_selector',
  'show_regulatory',
])

interface CommandButtonProps {
  control: CommandButtonControl
  disabled?: boolean
  disabledReason?: string | null
}

export function CommandButton({ control, disabled, disabledReason }: CommandButtonProps) {
  const panelState = usePanelStateStore((state) => state.state)
  const [showConfirm, setShowConfirm] = useState(false)
  const [showInput, setShowInput] = useState(false)
  const [showContent, setShowContent] = useState(false)
  const [executing, setExecuting] = useState(false)
  const [result, setResult] = useState<{ success: boolean; message?: string } | null>(null)
  const [sshKeysState, setSshKeysState] = useState<{ has_keys: boolean; username?: string } | null>(null)
  const [contentData, setContentData] = useState<{ content: string; modified?: string } | null>(null)
  const [inputConfig, setInputConfig] = useState<{
    title: string
    message: string
    placeholder?: string
    defaultValue?: string
    isPassword?: boolean
  } | null>(null)

  // Get dynamic description - pass empty params since buttons don't depend on param values
  const description = getDynamicDescription(control, panelState, {})
  const requiresDeviceUI = useMemo(() => control.action !== undefined && DEVICE_UI_ACTIONS.has(control.action), [control.action])
  const deviceOnlyMessage = requiresDeviceUI
    ? control.device_only_message ||
      'This action requires the device UI. Please make this change directly on your comma device.'
    : null

  // Load SSH keys state on mount if this is the manage_ssh_keys action
  useEffect(() => {
    if (control.action === 'manage_ssh_keys') {
      loadSshKeysState()
    }
  }, [control.action])

  const loadSshKeysState = async () => {
    try {
      const response = await panelAPI.executePanelCommand({
        action: 'manage_ssh_keys',
      })
      if (response.success) {
        setSshKeysState({
          has_keys: response.has_keys || false,
          username: response.username,
        })
      }
    } catch (error) {
      console.error('Failed to load SSH keys state:', error)
    }
  }

  const handleClick = async () => {
    if (requiresDeviceUI) {
      return
    }

    // Handle special interactive actions
    if (control.action === 'manage_ssh_keys') {
      handleManageSshKeys()
    } else if (control.action === 'set_copyparty_password') {
      handleSetCopypartyPassword()
    } else if (control.action === 'view_error_log') {
      handleViewErrorLog()
    } else if (control.confirm) {
      setShowConfirm(true)
    } else {
      executeCommand()
    }
  }

  const handleManageSshKeys = async () => {
    if (sshKeysState?.has_keys) {
      // Show confirmation to remove
      setInputConfig({
        title: 'Remove SSH Keys',
        message: `Current GitHub username: <b>${sshKeysState.username}</b><br><br>Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username other than your own. A comma employee will NEVER ask you to add their GitHub username.<br><br>Do you want to remove these SSH keys?`,
      })
      setShowConfirm(true)
    } else {
      // Show input dialog for GitHub username
      setInputConfig({
        title: 'Enter your GitHub username',
        message: 'Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username other than your own. A comma employee will NEVER ask you to add their GitHub username.',
        placeholder: 'GitHub username',
      })
      setShowInput(true)
    }
  }

  const handleSetCopypartyPassword = () => {
    setInputConfig({
      title: 'Set Copyparty Password',
      message: 'Enter a password to protect your Copyparty server.<br>Leave empty to disable password protection.',
      placeholder: 'Password',
      isPassword: true,
    })
    setShowInput(true)
  }

  const handleViewErrorLog = async () => {
    setExecuting(true)
    setResult(null)

    try {
      const response = await panelAPI.executePanelCommand({
        action: 'view_error_log',
      })

      if (response.success) {
        setContentData({
          content: response.content || 'No error log found',
          modified: response.modified,
        })
        setShowContent(true)
      } else {
        setResult({
          success: false,
          message: response.error || 'Failed to load error log',
        })
      }
    } catch (error) {
      setResult({
        success: false,
        message: error instanceof Error ? error.message : 'Failed to load error log',
      })
    } finally {
      setExecuting(false)
    }
  }

  const handleInputSubmit = async (value: string) => {
    setShowInput(false)
    setExecuting(true)
    setResult(null)

    try {
      let response

      if (control.action === 'manage_ssh_keys') {
        // Add SSH keys with GitHub username
        response = await panelAPI.executePanelCommand({
          action: 'manage_ssh_keys',
          username: value,
        })
        if (response.success) {
          await loadSshKeysState() // Reload state
        }
      } else if (control.action === 'set_copyparty_password') {
        // Set Copyparty password
        response = await panelAPI.executePanelCommand({
          action: 'set_copyparty_password',
          password: value,
        })
      } else {
        response = { success: false, error: 'Unknown action' }
      }

      setResult({
        success: response.success,
        message: response.message || response.error || (response.success ? 'Command executed successfully' : 'Command failed'),
      })
    } catch (error) {
      setResult({
        success: false,
        message: error instanceof Error ? error.message : 'Command failed',
      })
    } finally {
      setExecuting(false)
    }
  }

  const executeCommand = async () => {
    setShowConfirm(false)
    setExecuting(true)
    setResult(null)

    try {
      let response

      if (control.action === 'manage_ssh_keys' && sshKeysState?.has_keys) {
        // Remove SSH keys
        response = await panelAPI.executePanelCommand({
          action: 'manage_ssh_keys',
          remove: true,
        })
        if (response.success) {
          await loadSshKeysState() // Reload state
        }
      } else if (control.action) {
        // Execute panel command
        response = await panelAPI.executePanelCommand({
          action: control.action,
          param: control.param,
          value: control.value,
          params: control.params,
        })
      } else {
        // Unsupported - requires device UI
        response = {
          success: false,
          error: 'This command requires the device UI. Please use the settings panel on your Comma device.',
        }
      }

      setResult({
        success: response.success,
        message: response.message || response.error || (response.success ? 'Command executed successfully' : 'Command failed'),
      })
    } catch (error) {
      setResult({
        success: false,
        message: error instanceof Error ? error.message : 'Command failed',
      })
    } finally {
      setExecuting(false)
    }
  }

  const getButtonStyle = () => {
    if (control.button_style) {
      return {
        backgroundColor: control.button_style.background_color,
        color: control.button_style.text_color,
      }
    }
    return {}
  }

  const getButtonText = () => {
    if (control.action === 'manage_ssh_keys' && sshKeysState?.has_keys) {
      return 'REMOVE'
    }
    return control.button_text
  }

  // Determine if this is a danger action for confirmation dialog styling
  const isDangerAction = useMemo(() => {
    // Check if button has red background color
    const bgColor = control.button_style?.background_color?.toLowerCase()
    if (bgColor && (bgColor.includes('e22c2c') || bgColor.includes('ff2424') || bgColor.includes('dc3545'))) {
      return true
    }
    // Check for destructive actions
    const dangerActions = ['reset_settings', 'remove_params', 'remove_platform']
    if (control.action && dangerActions.includes(control.action)) {
      return true
    }
    // Check for destructive button text
    const dangerButtonTexts = ['RESET', 'DELETE', 'REMOVE', 'POWER OFF', 'SHUTDOWN']
    if (dangerButtonTexts.includes(control.button_text?.toUpperCase())) {
      return true
    }
    // SSH key removal
    if (sshKeysState?.has_keys) {
      return true
    }
    return false
  }, [control.button_style, control.action, control.button_text, sshKeysState?.has_keys])

  return (
    <>
      <ControlCard
        title={control.title}
        description={description}
        disabled={disabled || requiresDeviceUI}
        disabledReason={disabledReason}
        className="command-button-control"
        footer={
          <Button
            className="settings-btn"
            onClick={handleClick}
            disabled={disabled || requiresDeviceUI}
            loading={executing}
            style={getButtonStyle()}
          >
            {getButtonText()}
          </Button>
        }
      >
        {deviceOnlyMessage && <div className="command-button-device-notice">{deviceOnlyMessage}</div>}
        {result && (
          <div className={`command-button-result ${result.success ? 'success' : 'error'}`}>
            {result.message}
          </div>
        )}
      </ControlCard>

      {/* Confirmation Dialog */}
      <ConfirmDialog
        isOpen={showConfirm}
        onClose={() => setShowConfirm(false)}
        onConfirm={executeCommand}
        title={inputConfig?.title || control.title || 'Confirm Action'}
        message={<div dangerouslySetInnerHTML={{ __html: inputConfig?.message || control.confirm_text || `Are you sure you want to ${control.title}?` }} />}
        confirmText={control.confirm_button_text || control.confirm_yes_text || (isDangerAction ? control.button_text : 'Confirm')}
        cancelText={control.cancel_button_text || control.confirm_no_text || 'Cancel'}
        variant={isDangerAction ? 'danger' : 'warning'}
      />

      {/* Input Dialog */}
      {showInput && inputConfig && (
        <InputDialog
          isOpen={showInput}
          onClose={() => setShowInput(false)}
          onSubmit={handleInputSubmit}
          title={inputConfig.title}
          message={inputConfig.message}
          placeholder={inputConfig.placeholder}
          defaultValue={inputConfig.defaultValue}
          isPassword={inputConfig.isPassword}
        />
      )}

      {/* Content Modal (for error log) */}
      {showContent && contentData && (
        <Modal
          isOpen={showContent}
          title="Error Log"
          onClose={() => setShowContent(false)}
          size="large"
          actions={[
            {
              label: 'Close',
              onClick: () => setShowContent(false),
              variant: 'secondary',
            },
          ]}
        >
          {contentData.modified && (
            <div style={{ marginBottom: '1rem', fontWeight: 'bold' }}>{contentData.modified}</div>
          )}
          <pre style={{
            whiteSpace: 'pre-wrap',
            wordWrap: 'break-word',
            maxHeight: '60vh',
            overflow: 'auto',
            backgroundColor: '#f5f5f5',
            padding: '1rem',
            borderRadius: '4px',
            fontSize: '0.9rem'
          }}>
            {contentData.content}
          </pre>
        </Modal>
      )}
    </>
  )
}
