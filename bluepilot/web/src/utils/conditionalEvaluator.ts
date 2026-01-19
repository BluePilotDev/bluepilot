/**
 * Conditional Evaluator
 * Evaluates conditional logic for panel controls (visibility, enable/disable, dynamic content)
 */

import type { Conditions, Condition, PanelState } from '@/types/panels'

/**
 * Evaluate a set of conditions
 * Returns true if all conditions are met, false otherwise
 */
export function evaluateConditions(
  conditions: Conditions | undefined,
  state: PanelState,
  params: Record<string, any>
): boolean {
  if (!conditions) {
    return true // No conditions means always true
  }

  // Handle allConditionsTrue (AND logic)
  if (conditions.allConditionsTrue) {
    return conditions.allConditionsTrue.every((cond) =>
      evaluateSingleCondition(cond, state, params)
    )
  }

  // Handle anyConditionsTrue (OR logic)
  if (conditions.anyConditionsTrue) {
    return conditions.anyConditionsTrue.some((cond) =>
      evaluateSingleCondition(cond, state, params)
    )
  }

  // Otherwise, evaluate as a single condition
  return evaluateSingleCondition(conditions, state, params)
}

/**
 * Evaluate a single condition
 */
function evaluateSingleCondition(
  condition: Condition | Conditions,
  state: PanelState,
  params: Record<string, any>
): boolean {
  // Recursively handle nested allConditionsTrue/anyConditionsTrue
  if (condition.allConditionsTrue) {
    return condition.allConditionsTrue.every((cond: Condition) =>
      evaluateSingleCondition(cond, state, params)
    )
  }

  if (condition.anyConditionsTrue) {
    return condition.anyConditionsTrue.some((cond: Condition) =>
      evaluateSingleCondition(cond, state, params)
    )
  }

  // Evaluate parameter conditions
  if (condition.paramIsTrue !== undefined) {
    const value = params[condition.paramIsTrue]?.value
    return value === true || value === '1' || value === 1
  }

  if (condition.paramIsFalse !== undefined) {
    const value = params[condition.paramIsFalse]?.value
    return value === false || value === '0' || value === 0 || value === null || value === undefined
  }

  if (condition.paramValueEquals !== undefined) {
    for (const [paramKey, expectedValue] of Object.entries(condition.paramValueEquals)) {
      const actualValue = params[paramKey]?.value
      // Convert both to strings for comparison
      if (String(actualValue) !== String(expectedValue)) {
        return false
      }
    }
    return true
  }

  if (condition.paramValueGreaterThan !== undefined) {
    for (const [paramKey, threshold] of Object.entries(condition.paramValueGreaterThan)) {
      const value = params[paramKey]?.value
      const numValue = typeof value === 'number' ? value : parseFloat(value)
      const thresholdNum = typeof threshold === 'number' ? threshold : parseFloat(String(threshold))
      if (isNaN(numValue) || numValue <= thresholdNum) {
        return false
      }
    }
    return true
  }

  if (condition.paramValueLessThan !== undefined) {
    for (const [paramKey, threshold] of Object.entries(condition.paramValueLessThan)) {
      const value = params[paramKey]?.value
      const numValue = typeof value === 'number' ? value : parseFloat(value)
      const thresholdNum = typeof threshold === 'number' ? threshold : parseFloat(String(threshold))
      if (isNaN(numValue) || numValue >= thresholdNum) {
        return false
      }
    }
    return true
  }

  if (condition.paramExists !== undefined) {
    return params[condition.paramExists] !== undefined && params[condition.paramExists] !== null
  }

  // Evaluate device state conditions
  if (condition.isOnroad !== undefined) {
    return state.isOnroad === condition.isOnroad
  }

  if (condition.isOffroad !== undefined) {
    return state.isOffroad === condition.isOffroad
  }

  if (condition.hasCarParams !== undefined) {
    return state.hasCarParams === condition.hasCarParams
  }

  if (condition.hasLongitudinalControl !== undefined) {
    return state.hasLongitudinalControl === condition.hasLongitudinalControl
  }

  if (condition.hasBlindSpotMonitoring !== undefined) {
    return state.hasBlindSpotMonitoring === condition.hasBlindSpotMonitoring
  }

  if (condition.hasIntelligentCruiseButtonManagement !== undefined) {
    return (
      state.hasIntelligentCruiseButtonManagement === condition.hasIntelligentCruiseButtonManagement
    )
  }

  if (condition.hasAlphaLongitudinalAvailable !== undefined) {
    return state.hasAlphaLongitudinalAvailable === condition.hasAlphaLongitudinalAvailable
  }

  if (condition.isAngleSteering !== undefined) {
    return state.isAngleSteering === condition.isAngleSteering
  }

  if (condition.isMadsLimitedBrand !== undefined) {
    return state.isMadsLimitedBrand === condition.isMadsLimitedBrand
  }

  if (condition.isPcmCruise !== undefined) {
    return state.isPcmCruise === condition.isPcmCruise
  }

  if (condition.isICBMAvailable !== undefined) {
    return state.isICBMAvailable === condition.isICBMAvailable
  }

  // Evaluate branch conditions
  if (condition.isReleaseBranch !== undefined) {
    return state.isReleaseBranch === condition.isReleaseBranch
  }

  if (condition.isTestedBranch !== undefined) {
    return state.isTestedBranch === condition.isTestedBranch
  }

  if (condition.isDevelopmentBranch !== undefined) {
    return state.isDevelopmentBranch === condition.isDevelopmentBranch
  }

  // Evaluate brand conditions
  if (condition.brandEquals !== undefined) {
    const brand = condition.brandEquals
    return state.brandEquals?.[brand] === true
  }

  // If no recognized condition was found, default to true
  // This allows for unknown/future conditions to not break the UI
  return true
}

/**
 * Get the appropriate description based on condition matching
 */
export function getDynamicDescription(
  control: any,
  state: PanelState,
  params: Record<string, any>
): string {
  if (!control.dynamic_desc || !control.descriptions || !control.description_conditions) {
    return control.desc || ''
  }

  // Check each description condition
  for (const [key, conditions] of Object.entries(control.description_conditions)) {
    if (evaluateConditions(conditions as Conditions, state, params)) {
      return control.descriptions[key] || control.desc || ''
    }
  }

  // Fallback to default description
  return control.descriptions.default || control.desc || ''
}

/**
 * Get the appropriate title based on condition matching
 */
export function getDynamicTitle(
  control: any,
  _state: PanelState,
  params: Record<string, any>
): string {
  if (!control.dynamic_title || !control.titles) {
    return control.title || ''
  }

  // Check if toggle is enabled/disabled
  if (control.type === 'toggle' && control.param) {
    const value = params[control.param]?.value
    const isEnabled = value === true || value === '1' || value === 1

    if (isEnabled && control.titles.enabled) {
      return control.titles.enabled
    }
    if (!isEnabled && control.titles.disabled) {
      return control.titles.disabled
    }
  }

  // Fallback to default title
  return control.title || ''
}

/**
 * Get the appropriate style based on condition matching
 */
export function getDynamicStyle(
  control: any,
  _state: PanelState,
  params: Record<string, any>
): any {
  if (!control.dynamic_styling || !control.styles) {
    return control.button_style || {}
  }

  // Check if toggle is enabled/disabled
  if (control.type === 'toggle' && control.param) {
    const value = params[control.param]?.value
    const isEnabled = value === true || value === '1' || value === 1

    if (isEnabled && control.styles.enabled) {
      return control.styles.enabled
    }
    if (!isEnabled && control.styles.disabled) {
      return control.styles.disabled
    }
  }

  // Fallback to default style
  return control.button_style || {}
}

/**
 * Get the reason why a control is disabled
 */
export function getDisabledReason(
  enableConditions: Conditions | undefined,
  state: PanelState,
  params: Record<string, any>
): string | null {
  if (!enableConditions) {
    return null
  }

  // Check allConditionsTrue
  if (enableConditions.allConditionsTrue) {
    for (const cond of enableConditions.allConditionsTrue) {
      if (!evaluateSingleCondition(cond, state, params)) {
        return cond.reason || 'Condition not met'
      }
    }
    return null
  }

  // Check anyConditionsTrue
  if (enableConditions.anyConditionsTrue) {
    const anyMet = enableConditions.anyConditionsTrue.some((cond) =>
      evaluateSingleCondition(cond, state, params)
    )
    if (!anyMet) {
      // Return first reason found
      for (const cond of enableConditions.anyConditionsTrue) {
        if (cond.reason) {
          return cond.reason
        }
      }
      return 'None of the required conditions are met'
    }
    return null
  }

  // Single condition
  if (!evaluateSingleCondition(enableConditions, state, params)) {
    return enableConditions.reason || 'Condition not met'
  }

  return null
}

/**
 * Check if a control should be visible
 */
export function isControlVisible(
  control: any,
  state: PanelState,
  params: Record<string, any>
): boolean {
  if (control.hidden === true) {
    return false
  }

  if (control.visibleConditions) {
    return evaluateConditions(control.visibleConditions, state, params)
  }

  return true
}

/**
 * Check if a control should be enabled
 */
export function isControlEnabled(
  control: any,
  state: PanelState,
  params: Record<string, any>
): boolean {
  if (control.enableConditions) {
    return evaluateConditions(control.enableConditions, state, params)
  }

  return true
}
