/**
 * useStagedParam Hook
 * Provides a unified interface for controls to stage parameter changes
 * instead of saving immediately.
 *
 * When autoSave is enabled (session toggle), changes are also immediately
 * sent to the device for real-time tuning feedback.
 */

import { useCallback } from 'react'
import { useParamsStore } from '@/stores/useParamsStore'
import { useChangeTrackingStore, type PendingChange } from '@/stores/useChangeTrackingStore'

interface UseStagedParamOptions {
  param: string
  controlTitle: string
  controlType: PendingChange['controlType']
  panelId: string
  panelName: string
  groupName: string
}

interface UseStagedParamReturn {
  /** The effective value (staged if exists, otherwise actual) */
  value: string | number | boolean | null
  /** The actual saved value from backend */
  actualValue: string | number | boolean | null
  /** Whether this param has a pending (unsaved) change */
  hasChange: boolean
  /** Stage a new value (records change but doesn't save to backend unless autoSave enabled) */
  stageValue: (newValue: string | number | boolean) => void
  /** Discard the staged change for this param */
  discardChange: () => void
}

export function useStagedParam(options: UseStagedParamOptions): UseStagedParamReturn {
  const { param, controlTitle, controlType, panelId, panelName, groupName } = options

  const { params, stagedParams, stageParam, unstageParam, updateParam } = useParamsStore()
  const { recordChange, discardChange: discardTrackedChange, isAutoSaveEnabled } = useChangeTrackingStore()

  const actualValue = params[param]?.value ?? null
  const stagedValue = stagedParams[param]
  const hasChange = param in stagedParams
  const value = hasChange ? stagedValue : actualValue

  const stageValue = useCallback(
    (newValue: string | number | boolean) => {
      // Check if auto-save is enabled for this session
      const autoSave = isAutoSaveEnabled()

      if (autoSave) {
        // Auto-save mode: save immediately to backend (no staging)
        updateParam(param, newValue)
      } else {
        // Staged mode: update local state only
        stageParam(param, newValue)

        // Record in change tracking store (for summary/save)
        recordChange({
          param,
          oldValue: actualValue,
          newValue,
          controlTitle,
          controlType,
          panelId,
          panelName,
          groupName,
        })
      }
    },
    [param, actualValue, controlTitle, controlType, panelId, panelName, groupName, stageParam, recordChange, isAutoSaveEnabled, updateParam]
  )

  const discardChange = useCallback(() => {
    unstageParam(param)
    discardTrackedChange(param)
  }, [param, unstageParam, discardTrackedChange])

  return {
    value,
    actualValue,
    hasChange,
    stageValue,
    discardChange,
  }
}
