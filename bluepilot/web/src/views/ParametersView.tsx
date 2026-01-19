import { useEffect, useState, useMemo } from 'react'
import { Header } from '@/components/layout/Header'
import { useParamsStore } from '@/stores/useParamsStore'
import { LoadingSpinner, Button, Modal, ToastContainer, ToggleSwitch, BackToTop } from '@/components/common'
import type { Parameter, DeviceStatus } from '@/types'
import { formatParamValueForDisplay } from '@/utils/params'
import './ParametersView.css'

interface ParametersViewProps {
  deviceStatus?: DeviceStatus
}

type SortColumn = 'key' | 'value' | 'type' | 'category' | 'last_modified'
type SortDirection = 'asc' | 'desc'

const SORT_OPTIONS: { label: string; value: SortColumn }[] = [
  { label: 'Parameter', value: 'key' },
  { label: 'Value', value: 'value' },
  { label: 'Type', value: 'type' },
  { label: 'Category', value: 'category' },
  { label: 'Last Modified', value: 'last_modified' },
]

const NUMERIC_TYPES = new Set(['number', 'int', 'float'])
const BOOLEAN_TYPES = new Set(['boolean', 'bool'])

const getSortValue = (param: Parameter, column: SortColumn): string | number | boolean | null | undefined => {
  switch (column) {
    case 'key':
      return param.key
    case 'value':
      return param.value
    case 'type':
      return param.type
    case 'category':
      return param.category ?? ''
    case 'last_modified':
      return param.last_modified ?? 0
    default:
      return ''
  }
}

export const ParametersView = ({ deviceStatus = 'checking' }: ParametersViewProps) => {
  const { params, loading, fetchParams, updateParam, searchQuery, setSearchQuery, getFilteredParams } =
    useParamsStore()
  const [editingParam, setEditingParam] = useState<Parameter | null>(null)
  const [editValue, setEditValue] = useState<string>('')
  const [editMode, setEditMode] = useState(false)
  const [sortColumn, setSortColumn] = useState<SortColumn>('key')
  const [sortDirection, setSortDirection] = useState<SortDirection>('asc')
  const [viewValueModal, setViewValueModal] = useState<Parameter | null>(null)
  const [toasts, setToasts] = useState<Array<{ id: string; message: string; type?: 'success' | 'error' | 'info' }>>([])

  useEffect(() => {
    fetchParams()
  }, [fetchParams])

  const formatLastModified = (timestamp?: number): string => {
    if (!timestamp) return 'Never'

    const date = new Date(timestamp * 1000)
    const now = new Date()
    const diffMs = now.getTime() - date.getTime()
    const diffMins = Math.floor(diffMs / 60000)
    const diffHours = Math.floor(diffMs / 3600000)
    const diffDays = Math.floor(diffMs / 86400000)

    if (diffMins < 1) return 'Just now'
    if (diffMins < 60) return `${diffMins}m ago`
    if (diffHours < 24) return `${diffHours}h ago`
    if (diffDays < 7) return `${diffDays}d ago`
    return date.toLocaleDateString()
  }

  const sortedParams = useMemo(() => {
    const filtered = getFilteredParams()
      .filter((param) => param.key && param.key !== 'null' && param.key !== 'undefined')

    return filtered.sort((a, b) => {
      let aVal = getSortValue(a, sortColumn)
      let bVal = getSortValue(b, sortColumn)

      if (aVal === undefined || aVal === null) aVal = ''
      if (bVal === undefined || bVal === null) bVal = ''

      if (typeof aVal === 'string') {
        aVal = aVal.toLowerCase()
      }
      if (typeof bVal === 'string') {
        bVal = bVal.toLowerCase()
      }

      if (aVal < bVal) return sortDirection === 'asc' ? -1 : 1
      if (aVal > bVal) return sortDirection === 'asc' ? 1 : -1
      return 0
    })
  }, [params, searchQuery, getFilteredParams, sortColumn, sortDirection])

  const toggleSortDirection = () => {
    setSortDirection((prev) => (prev === 'asc' ? 'desc' : 'asc'))
  }

  const handleEdit = (param: Parameter) => {
    setEditingParam(param)
    setEditValue(String(param.value))
  }

  const handleSave = async () => {
    if (!editingParam) return

    let value: string | number | boolean = editValue
    const type = editingParam.type?.toLowerCase()

    if (type && NUMERIC_TYPES.has(type)) {
      const parsed = Number(editValue)
      value = Number.isNaN(parsed) ? 0 : parsed
    } else if (type && BOOLEAN_TYPES.has(type)) {
      value = editValue === 'true'
    }

    await updateParam(editingParam.key, value)
    setEditingParam(null)
  }

  const handleViewValue = (param: Parameter) => {
    setViewValueModal(param)
  }

  const addToast = (message: string, type: 'success' | 'error' | 'info' = 'success') => {
    const id = `${Date.now()}-${Math.random()}`
    setToasts((prev) => [...prev, { id, message, type }])
  }

  const removeToast = (id: string) => {
    setToasts((prev) => prev.filter((toast) => toast.id !== id))
  }

  const copyToClipboard = (text: string, label: string = 'Value') => {
    navigator.clipboard
      .writeText(text)
      .then(() => {
        addToast(`${label} copied to clipboard`, 'success')
      })
      .catch(() => {
        addToast('Failed to copy to clipboard', 'error')
      })
  }

  const formattedModalValue = viewValueModal ? formatParamValueForDisplay(viewValueModal) : null
  const modalValueDisplay = (() => {
    if (!formattedModalValue) return '—'
    if (formattedModalValue.isBinary) {
      if (formattedModalValue.decodedString) return formattedModalValue.decodedString
      if (viewValueModal?.raw_value) return viewValueModal.raw_value
    }
    return formattedModalValue.display || formattedModalValue.raw || '—'
  })()
  const modalCopyValue = formattedModalValue?.raw ?? modalValueDisplay

  if (loading && Object.keys(params).length === 0) {
    return (
      <>
        <Header deviceStatus={deviceStatus} />
        <div className="loading">
          <LoadingSpinner size="large" message="Loading parameters..." />
        </div>
      </>
    )
  }

  return (
    <>
      <Header deviceStatus={deviceStatus} subtitle="Manage system parameters" />
      <ToastContainer toasts={toasts} onRemove={removeToast} />
      <div className="params-manager">
        <div className="params-header">
          <div className="params-controls">
            <div className="params-sort-controls">
              <label htmlFor="params-sort">Sort</label>
              <select
                id="params-sort"
                value={sortColumn}
                onChange={(e) => setSortColumn(e.target.value as SortColumn)}
              >
                {SORT_OPTIONS.map((option) => (
                  <option key={option.value} value={option.value}>
                    {option.label}
                  </option>
                ))}
              </select>
              <button
                type="button"
                className="sort-direction-btn"
                onClick={toggleSortDirection}
                title={`Switch to ${sortDirection === 'asc' ? 'descending' : 'ascending'} order`}
              >
                {sortDirection === 'asc' ? '↑ Asc' : '↓ Desc'}
              </button>
            </div>
            <ToggleSwitch
              checked={editMode}
              onChange={setEditMode}
              label="Edit Mode"
              size="compact"
              alignLabel="start"
              className={`params-edit-toggle ${editMode ? 'active' : ''}`}
              title="Enable parameter editing (use with caution)"
            />
            <input
              type="text"
              id="params-search"
              placeholder="Search parameters..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
            />
          </div>
        </div>
        <div className="params-content">
          {sortedParams.length === 0 ? (
            <div className="empty-state">
              <p>No parameters found</p>
            </div>
          ) : (
            <div className="params-list">
              {sortedParams.map((param) => {
                const formattedValue = formatParamValueForDisplay(param)
                const trimmed = formattedValue.display.trim()
                const preview =
                  trimmed.length > 260 ? `${trimmed.substring(0, 257).trimEnd()}…` : trimmed || '—'
                const formatBadge = formattedValue.formatLabel?.toUpperCase()
                const typeBadge = param.type ? param.type.toUpperCase() : null
                const attributeBadges = param.attributes || []

                return (
                  <div className="param-row" key={param.key}>
                    <div className="param-row__header">
                      <div className="param-row__title-block">
                        <div className="param-row__title-line">
                          <span className="param-key" title={param.key}>
                            {param.key}
                          </span>
                          <div className="param-row__status-chips">
                            {param.category && (
                              <span className={`param-badge category ${param.category.toLowerCase()}`}>
                                {param.category}
                              </span>
                            )}
                            {param.readonly && <span className="param-badge readonly">readonly</span>}
                            {param.critical && <span className="param-badge critical">critical</span>}
                          </div>
                        </div>
                        <div className="param-last-modified">Last modified: {formatLastModified(param.last_modified)}</div>
                      </div>
                      <div className="param-row__actions">
                        {param.readonly ? (
                          <Button size="small" variant="ghost" className="param-edit-btn" disabled>
                            Read-Only
                          </Button>
                        ) : (
                          <Button
                            size="small"
                            variant="primary"
                            className="param-edit-btn"
                            onClick={() => handleEdit(param)}
                            disabled={!editMode || param.type === 'bytes'}
                            title={param.type === 'bytes' ? 'Binary parameters are view-only' : undefined}
                          >
                            Edit
                          </Button>
                        )}
                      </div>
                    </div>
                    <div
                      className="param-row__value"
                      title="Click to view full value"
                      onClick={() => handleViewValue(param)}
                    >
                      <div className="param-row__value-header">
                        <span className="value-label">Value</span>
                        <div className="value-pill-group">
                          {typeBadge && <span className="value-pill value-pill--type">{typeBadge}</span>}
                          {attributeBadges.map((attr) => (
                            <span key={attr} className="value-pill value-pill--attribute">{attr}</span>
                          ))}
                          {formatBadge && formatBadge !== typeBadge && (
                            <span className="value-pill">{formatBadge}</span>
                          )}
                          {param.type === 'bytes' && param.byte_length !== undefined && (
                            <span className="value-pill value-pill--outline">{param.byte_length} bytes</span>
                          )}
                        </div>
                      </div>
                      <pre className="value-code-block">
                        <code>{preview}</code>
                      </pre>
                      <span className="value-footer-hint">Click to inspect full value</span>
                    </div>
                    {param.description && (
                      <div className="param-description">
                        <span className="description-label">Description</span>
                        <p>{param.description}</p>
                      </div>
                    )}
                    {param.critical && (
                      <div className="critical-flag">
                        <span>⚠️ Critical parameter</span>
                      </div>
                    )}
                  </div>
                )
              })}
            </div>
          )}
        </div>
      </div>

      <Modal
        isOpen={editingParam !== null}
        onClose={() => setEditingParam(null)}
        title="Edit Parameter"
        size="small"
      >
        {editingParam && (
          <div className="edit-param-modal-new">
            <div className="edit-param-header">
              <div className="edit-param-key-section">
                <span className="param-key-label">Parameter Key</span>
                <code className="param-key-value-edit">{editingParam.key}</code>
              </div>
              <div className="edit-param-type">
                <span className={`param-badge ${editingParam.type}`}>{editingParam.type}</span>
              </div>
            </div>

            {editingParam.description && (
              <div className="edit-param-description">
                <span className="description-label">Description</span>
                <p>{editingParam.description}</p>
              </div>
            )}

            <div className="edit-param-input-section">
              <label htmlFor="edit-param-value" className="input-label">
                New Value
              </label>
              {editingParam.type && BOOLEAN_TYPES.has(editingParam.type.toLowerCase()) ? (
                <select
                  id="edit-param-value"
                  value={editValue}
                  onChange={(e) => setEditValue(e.target.value)}
                  className="edit-input-new"
                  autoFocus
                >
                  <option value="true">true</option>
                  <option value="false">false</option>
                </select>
              ) : (
                <input
                  id="edit-param-value"
                  type={editingParam.type && NUMERIC_TYPES.has(editingParam.type.toLowerCase()) ? 'number' : 'text'}
                  value={editValue}
                  onChange={(e) => setEditValue(e.target.value)}
                  className="edit-input-new"
                  placeholder={`Enter ${editingParam.type} value...`}
                  autoFocus
                />
              )}
            </div>

            {editingParam.critical && (
              <div className="edit-warning-banner">
                <span className="warning-icon">⚠️</span>
                <div className="warning-content">
                  <strong>Caution: Critical Parameter</strong>
                  <p>Changes to this parameter may affect system stability.</p>
                </div>
              </div>
            )}

            <div className="modal-actions">
              <Button variant="secondary" onClick={() => setEditingParam(null)}>
                Cancel
              </Button>
              <Button variant="primary" onClick={handleSave}>
                Save Changes
              </Button>
            </div>
          </div>
        )}
      </Modal>

      <Modal
        isOpen={viewValueModal !== null}
        onClose={() => setViewValueModal(null)}
        title="Parameter Details"
        size="large"
      >
        {viewValueModal && (
          <div className="param-modal-simple">
            <div className="param-modal-simple__header">
              <span className="param-key-label">Parameter Key</span>
              <div className="param-key-value-wrapper">
                <code className="param-key-value">{viewValueModal.key}</code>
                <button
                  className="icon-btn"
                  onClick={() => copyToClipboard(viewValueModal.key, 'Key')}
                  title="Copy key"
                >
                  📋
                </button>
              </div>
            </div>

            <div className="param-modal-simple__value">
              <div className="value-pill-group">
                {viewValueModal.type && (
                  <span className="value-pill value-pill--type">{viewValueModal.type.toUpperCase()}</span>
                )}
                {viewValueModal.attributes?.map((attr) => (
                  <span key={attr} className="value-pill value-pill--attribute">{attr}</span>
                ))}
                {formattedModalValue?.formatLabel && (
                  <span className="value-pill">{formattedModalValue.formatLabel.toUpperCase()}</span>
                )}
                {viewValueModal.type === 'bytes' && viewValueModal.byte_length !== undefined && (
                  <span className="value-pill value-pill--outline">{viewValueModal.byte_length} bytes</span>
                )}
              </div>
              <pre
                className={`value-code-block value-code-block--modal ${
                  formattedModalValue?.isBinary ? 'binary-value' : 'text-value'
                }`}
              >
                <code>{modalValueDisplay}</code>
              </pre>
            </div>

            <div className="param-modal-simple__actions">
              <Button
                variant="secondary"
                onClick={() => {
                  if (formattedModalValue) {
                    copyToClipboard(modalCopyValue, 'Value')
                  }
                }}
              >
                Copy Value
              </Button>
              <Button variant="secondary" onClick={() => copyToClipboard(viewValueModal.key, 'Key')}>
                Copy Key
              </Button>
              <Button variant="primary" onClick={() => setViewValueModal(null)}>
                Close
              </Button>
            </div>
          </div>
        )}
      </Modal>

      <BackToTop />
    </>
  )
}
