/**
 * Panel API Service
 * Handles all API calls related to panel configurations and commands
 */

import type {
  PanelsListResponse,
  PanelResponse,
  PanelStateResponse,
  PanelCommandRequest,
  PanelCommandResponse,
} from '@/types/panels'

const API_BASE = '/api'

/**
 * Get list of all available panels
 */
export async function getPanels(): Promise<PanelsListResponse> {
  const response = await fetch(`${API_BASE}/panels`)
  if (!response.ok) {
    throw new Error(`Failed to fetch panels: ${response.statusText}`)
  }
  return response.json()
}

/**
 * Get a specific panel configuration
 */
export async function getPanel(panelId: string): Promise<PanelResponse> {
  const response = await fetch(`${API_BASE}/panels/${panelId}`)
  if (!response.ok) {
    throw new Error(`Failed to fetch panel ${panelId}: ${response.statusText}`)
  }
  return response.json()
}

/**
 * Get current panel state (device state for conditionals)
 */
export async function getPanelState(): Promise<PanelStateResponse> {
  const response = await fetch(`${API_BASE}/panel-state`)
  if (!response.ok) {
    throw new Error(`Failed to fetch panel state: ${response.statusText}`)
  }
  return response.json()
}

/**
 * Execute a panel command
 */
export async function executePanelCommand(
  request: PanelCommandRequest
): Promise<PanelCommandResponse> {
  const response = await fetch(`${API_BASE}/panel-command`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  })

  if (!response.ok) {
    const error = await response.json()
    throw new Error(error.error || `Command failed: ${response.statusText}`)
  }

  return response.json()
}

/**
 * Set a parameter value (convenience wrapper)
 */
export async function setParameter(key: string, value: any): Promise<PanelCommandResponse> {
  return executePanelCommand({
    action: 'set_param',
    param: key,
    value,
  })
}

/**
 * Remove parameters (convenience wrapper)
 */
export async function removeParameters(params: string[]): Promise<PanelCommandResponse> {
  return executePanelCommand({
    action: 'remove_params',
    params,
  })
}

// Export all panel API methods
export const panelAPI = {
  getPanels,
  getPanel,
  getPanelState,
  executePanelCommand,
  setParameter,
  removeParameters,
}
