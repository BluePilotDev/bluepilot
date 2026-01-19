/**
 * Panel Configuration Types
 * TypeScript interfaces for BluePilot panel JSON configurations
 */

// ============================================================================
// Panel Metadata
// ============================================================================

export interface PanelMetadata {
  id: string
  name: string
  description: string
  icon?: string
}

export interface PanelConfig {
  menuName: string
  menuIcon?: string
  menuDescription: string
  type?: string
  groups: PanelGroup[]
  persistentParams?: string[]
  clearOnManagerStartParams?: string[]
  clearOnOnroadTransitionParams?: string[]
  clearOnOffroadTransitionParams?: string[]
}

export interface PanelGroup {
  groupName: string
  title: string
  controls: PanelControl[]
  enableResetButton?: boolean
  hidden?: boolean
}

// ============================================================================
// Panel Controls
// ============================================================================

export type PanelControl =
  | ToggleControl
  | SelectionControl
  | SegmentedControl
  | IntegerControl
  | FloatControl
  | CommandButtonControl
  | FileViewerControl
  | StaticParamDisplayControl
  | FileParamDisplayControl
  | PlatformDisplayControl
  | ParamViewerControl
  | ParamListViewerControl
  | StaticTextControl
  | RecentChangesControl
  | RestartUIControl

// Base control interface with common properties
interface BaseControl {
  type: string
  title: string
  desc?: string
  icon?: string
  hidden?: boolean
  confirm?: boolean
  confirm_text?: string
  confirm_yes_text?: string
  confirm_no_text?: string
  requiresReboot?: boolean

  // Web UI support flag
  webSupported?: boolean // Set to false to hide control in web UI (defaults to true if omitted)

  // Conditional visibility and enabling
  visibleConditions?: Conditions
  enableConditions?: Conditions

  // Dynamic descriptions
  dynamic_desc?: boolean
  descriptions?: Record<string, string>
  description_conditions?: Record<string, Conditions>

  // Dynamic titles
  dynamic_title?: boolean
  titles?: Record<string, string>

  // Dynamic styling
  dynamic_styling?: boolean
  styles?: Record<string, ControlStyle>
}

export interface ControlStyle {
  background_color?: string
  background_color_pressed?: string
  text_color?: string
}

// ============================================================================
// Specific Control Types
// ============================================================================

export interface ToggleControl extends BaseControl {
  type: 'toggle'
  param: string
  confirmation?: boolean
}

export interface SelectionControl extends BaseControl {
  type: 'selection'
  param: string
  options: SelectionOption[]
  unit?: string
  unitMetric?: string
}

export interface SelectionOption {
  name: string
  label?: string
  value: string
  default?: boolean
  desc?: string
  enableConditions?: Conditions
}

export interface SegmentedControl extends BaseControl {
  type: 'segmented_control'
  param: string
  options: SelectionOption[]
  showDescBottom?: boolean
}

export interface IntegerControl extends BaseControl {
  type: 'integer'
  param: string
  min: number
  max: number
  increment: number
  unit?: string
  unitMetric?: string
  division?: number
}

export interface FloatControl extends BaseControl {
  type: 'float'
  param: string
  min: number
  max: number
  increment: number
  unit?: string
  unitMetric?: string
  division?: number
}

export interface CommandButtonControl extends BaseControl {
  type: 'command_button'
  button_text: string
  action?: string
  command?: string
  working_dir?: string
  param?: string
  value?: string
  params?: string[]
  button_style?: ControlStyle
  actionButtons?: ActionButton[]
  command_timeout_ms?: number
  confirm_button_text?: string
  cancel_button_text?: string
  OnlyOnCommaDevice?: boolean
  connect_signal?: string
  device_only_message?: string
}

export interface ActionButton {
  text: string
  action: string
  showWhen: string
  confirm?: boolean
  confirm_text?: string
  confirm_yes_text?: string
  confirm_no_text?: string
}

export interface FileViewerControl extends BaseControl {
  type: 'file_viewer'
  path: string
  header?: string
  button_text?: string
  conditions?: GitConditions
}

export interface StaticParamDisplayControl extends BaseControl {
  type: 'static_param_display'
  param: string
}

export interface FileParamDisplayControl extends BaseControl {
  type: 'file_param_display'
  file: string
  prefix?: string
}

export interface PlatformDisplayControl extends BaseControl {
  type: 'platform_display'
  value_param: string
  value_color?: string
}

export interface ParamViewerControl extends BaseControl {
  type: 'param_viewer'
  param: string
}

export interface ParamListViewerControl extends BaseControl {
  type: 'param_list_viewer'
}

export interface StaticTextControl extends BaseControl {
  type: 'static_text'
}

export interface RecentChangesControl extends BaseControl {
  type: 'recent_changes'
  conditions?: GitConditions
}

export interface RestartUIControl extends BaseControl {
  type: 'restart_ui'
  button_text: string
}

// ============================================================================
// Conditional System
// ============================================================================

export interface Conditions {
  // Logical operators
  allConditionsTrue?: Condition[]
  anyConditionsTrue?: Condition[]

  // Parameter conditions
  paramIsTrue?: string
  paramIsFalse?: string
  paramValueEquals?: Record<string, string>
  paramValueGreaterThan?: Record<string, number>
  paramValueLessThan?: Record<string, number>
  paramExists?: string

  // Device state conditions
  isOnroad?: boolean
  isOffroad?: boolean
  hasCarParams?: boolean
  hasLongitudinalControl?: boolean
  hasIntelligentCruiseButtonManagement?: boolean
  hasBlindSpotMonitoring?: boolean
  hasAlphaLongitudinalAvailable?: boolean
  isAngleSteering?: boolean
  isMadsLimitedBrand?: boolean
  isPcmCruise?: boolean
  isICBMAvailable?: boolean

  // Branch conditions
  isReleaseBranch?: boolean
  isTestedBranch?: boolean
  isDevelopmentBranch?: boolean

  // Brand conditions
  brandEquals?: string

  // Reason for condition (for error messages)
  reason?: string
}

export interface Condition {
  // Can contain any of the Conditions properties
  [key: string]: any
}

export interface GitConditions {
  git_remote?: string[]
  git_branch?: string[]
}

// ============================================================================
// Panel State
// ============================================================================

export interface PanelState {
  // Basic state
  isOnroad: boolean
  isOffroad: boolean
  hasCarParams: boolean

  // Vehicle capabilities
  hasLongitudinalControl?: boolean
  hasBlindSpotMonitoring?: boolean
  hasIntelligentCruiseButtonManagement?: boolean
  hasAlphaLongitudinalAvailable?: boolean
  isAngleSteering?: boolean
  isMadsLimitedBrand?: boolean
  isPcmCruise?: boolean
  isICBMAvailable?: boolean

  // Branch info
  isReleaseBranch?: boolean
  isTestedBranch?: boolean
  isDevelopmentBranch?: boolean

  // Brand detection
  brandEquals?: Record<string, boolean>

  // Git info
  gitRemote?: string
  gitBranch?: string
}

// ============================================================================
// API Response Types
// ============================================================================

export interface PanelsListResponse {
  success: boolean
  panels: PanelMetadata[]
}

export interface PanelResponse {
  success: boolean
  panel: PanelConfig
  error?: string
}

export interface PanelStateResponse {
  success: boolean
  state: PanelState
  error?: string
}

export interface PanelCommandRequest {
  action: string
  param?: string
  value?: any
  params?: string[]
  // manage_ssh_keys action
  username?: string
  remove?: boolean
  // set_copyparty_password action
  password?: string
}

export interface PanelCommandResponse {
  success: boolean
  error?: string
  hint?: string
  message?: string
  removed?: string[]
  failed?: string[]
  // manage_ssh_keys action
  has_keys?: boolean
  username?: string
  // set_copyparty_password action
  requires_reboot?: boolean
  // view_error_log action
  content?: string
  modified?: string
}
