/**
 * Settings View
 * Main settings page with tabbed panel interface
 */

import { useEffect, useState, useRef, useCallback } from 'react'
import { Header } from '@/components/layout/Header'
import { LoadingSpinner, Icon, Modal, Button, BackToTop } from '@/components/common'
import { PanelGroup } from '@/components/settings/PanelGroup'
import { FavoritesPanel } from '@/components/settings/FavoritesPanel'
import { FloatingChangesIndicator } from '@/components/settings/FloatingChangesIndicator'
import { usePanelsStore } from '@/stores/usePanelsStore'
import { usePanelStateStore } from '@/stores/usePanelStateStore'
import { useParamsStore } from '@/stores/useParamsStore'
import { useUnsavedChangesWarning } from '@/hooks/useUnsavedChangesWarning'
import type { DeviceStatus } from '@/types'
import './SettingsView.css'

interface SettingsViewProps {
  deviceStatus: DeviceStatus
}

type PanelIconKey =
  | 'favorites'
  | 'bp_device_panel'
  | 'bp_display_panel'
  | 'bp_visuals_panel'
  | 'bp_vehicle_panel'
  | 'bp_cruise_panel'
  | 'bp_toggles_panel'
  | 'bp_steering_panel'
  | 'bp_developer_panel'
  | 'default'

const panelIcons: Record<PanelIconKey, () => JSX.Element> = {
  favorites: () => <Icon name="star" />,
  bp_device_panel: () => <Icon name="devices" />,
  bp_display_panel: () => <Icon name="monitor" />,
  bp_visuals_panel: () => <Icon name="visibility" />,
  bp_vehicle_panel: () => <Icon name="directions_car" />,
  bp_cruise_panel: () => <Icon name="speed" />,
  bp_toggles_panel: () => <Icon name="toggle_on" />,
  bp_steering_panel: () => <Icon name="trip_origin" />,
  bp_developer_panel: () => <Icon name="code" />,
  default: () => <Icon name="dashboard" />,
}

const getPanelIcon = (panelId?: string) => {
  if (!panelId) return panelIcons.default()
  const icon = panelIcons[panelId as PanelIconKey]
  return icon ? icon() : panelIcons.default()
}

export function SettingsView({ deviceStatus: _deviceStatus }: SettingsViewProps) {
  const { panels, loadedPanels, loading, error, fetchPanels, fetchPanel } = usePanelsStore()
  const { state, fetchState } = usePanelStateStore()
  const { fetchParams } = useParamsStore()
  const [selectedPanelId, setSelectedPanelId] = useState<string | null>(null)
  const [searchQuery, setSearchQuery] = useState('')

  // Warn user about unsaved changes when leaving page
  useUnsavedChangesWarning()

  // Backup/Restore state
  const [exporting, setExporting] = useState(false)
  const [importing, setImporting] = useState(false)
  const [backupResult, setBackupResult] = useState<{ success: boolean; message: string; details?: any } | null>(null)
  const [showBackupModal, setShowBackupModal] = useState(false)
  const fileInputRef = useRef<HTMLInputElement>(null)

  // Mobile dropdown state
  const [mobileNavOpen, setMobileNavOpen] = useState(false)
  const mobileNavRef = useRef<HTMLDivElement>(null)

  // Close mobile nav when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (mobileNavRef.current && !mobileNavRef.current.contains(event.target as Node)) {
        setMobileNavOpen(false)
      }
    }
    if (mobileNavOpen) {
      document.addEventListener('mousedown', handleClickOutside)
    }
    return () => document.removeEventListener('mousedown', handleClickOutside)
  }, [mobileNavOpen])

  const handleMobileNavSelect = useCallback((panelId: string) => {
    setSelectedPanelId(panelId)
    setMobileNavOpen(false)
  }, [])

  // Fetch panels and state on mount
  useEffect(() => {
    fetchPanels()
    fetchState()
    fetchParams()
  }, [fetchPanels, fetchState, fetchParams])

  // Export settings handler
  const handleExport = async () => {
    setExporting(true)
    setBackupResult(null)

    try {
      const response = await fetch('/api/params/backup')
      const data = await response.json()

      if (data.success) {
        const backup = {
          version: '1.0',
          timestamp: new Date().toISOString(),
          device: 'BluePilot',
          params: data.params,
          count: data.count,
        }

        const blob = new Blob([JSON.stringify(backup, null, 2)], { type: 'application/json' })
        const url = URL.createObjectURL(blob)
        const link = document.createElement('a')
        link.href = url
        link.download = `bluepilot-backup-${new Date().toISOString().split('T')[0]}.json`
        document.body.appendChild(link)
        link.click()
        document.body.removeChild(link)
        URL.revokeObjectURL(url)

        setBackupResult({ success: true, message: `Exported ${data.count} settings` })
        setShowBackupModal(true)
      } else {
        setBackupResult({ success: false, message: data.error || 'Export failed' })
        setShowBackupModal(true)
      }
    } catch (err) {
      setBackupResult({ success: false, message: err instanceof Error ? err.message : 'Export failed' })
      setShowBackupModal(true)
    } finally {
      setExporting(false)
    }
  }

  // Import settings handler
  const handleImport = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0]
    if (!file) return

    setImporting(true)
    setBackupResult(null)

    try {
      const text = await file.text()
      const backup = JSON.parse(text)

      if (!backup.params || !backup.version) {
        throw new Error('Invalid backup file format')
      }

      const response = await fetch('/api/params/restore', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ params: backup.params }),
      })

      const data = await response.json()

      if (data.success || data.restored?.length > 0) {
        setBackupResult({
          success: true,
          message: `Restored ${data.count || data.restored?.length || 0} settings`,
          details: { restored: data.restored?.length || 0, failed: data.failed?.length || 0 },
        })
        // Refresh params after restore
        fetchParams()
      } else {
        setBackupResult({ success: false, message: data.error || 'Restore failed' })
      }
      setShowBackupModal(true)
    } catch (err) {
      setBackupResult({ success: false, message: err instanceof Error ? err.message : 'Import failed' })
      setShowBackupModal(true)
    } finally {
      setImporting(false)
      event.target.value = ''
    }
  }

  // Auto-select Favorites panel on mount
  useEffect(() => {
    if (!selectedPanelId) {
      setSelectedPanelId('favorites')
    }
  }, [selectedPanelId])

  // Load selected panel configuration
  useEffect(() => {
    // Skip fetching for special built-in panels
    if (selectedPanelId && selectedPanelId !== 'favorites' && !loadedPanels[selectedPanelId]) {
      fetchPanel(selectedPanelId)
    }
  }, [selectedPanelId, loadedPanels, fetchPanel])

  const selectedPanel = selectedPanelId ? loadedPanels[selectedPanelId] : null

  // Filter panel groups based on search query
  const filteredGroups = selectedPanel?.groups.map((group) => {
    if (!searchQuery.trim()) {
      return group // No filtering
    }

    const query = searchQuery.toLowerCase()
    const filteredControls = group.controls.filter((control) => {
      // Skip controls that are not web-supported
      if ('webSupported' in control && control.webSupported === false) {
        return false
      }

      const title = control.title?.toLowerCase() || ''
      const desc = control.desc?.toLowerCase() || ''
      return title.includes(query) || desc.includes(query)
    })

    return {
      ...group,
      controls: filteredControls,
    }
  }).filter((group) => group.controls.length > 0) // Remove empty groups

  if (loading && panels.length === 0) {
    return (
      <>
        <Header deviceStatus={_deviceStatus} subtitle="Configure BluePilot settings and behavior" />
        <div className="settings-view settings-view-centered">
          <LoadingSpinner message="Loading settings..." />
        </div>
      </>
    )
  }

  if (error) {
    return (
      <>
        <Header deviceStatus={_deviceStatus} subtitle="Configure BluePilot settings and behavior" />
        <div className="settings-view settings-view-centered">
          <div className="settings-error-card">
            <h2>Error Loading Settings</h2>
            <p>{error}</p>
          </div>
        </div>
      </>
    )
  }

  const favoritesMeta = {
    id: 'favorites',
    name: 'Favorites',
    description: 'Pinned controls from every panel for quick access',
  }

  const activeMetadata =
    selectedPanelId === 'favorites'
      ? favoritesMeta
      : panels.find((panel) => panel.id === selectedPanelId) || null

  const headerDescription =
    selectedPanelId === 'favorites'
      ? favoritesMeta.description
      : selectedPanel?.menuDescription || activeMetadata?.description || 'Configure BluePilot behavior offroad'

  return (
    <>
      <Header deviceStatus={_deviceStatus} subtitle="Configure BluePilot settings and behavior" />
      {/* Hidden file input for import */}
      <input
        type="file"
        accept=".json"
        onChange={handleImport}
        disabled={importing}
        ref={fileInputRef}
        className="settings-file-input-hidden"
        aria-label="Import settings file"
      />
      <div className="settings-view">
        <div className="settings-layout">
          <aside className="settings-sidebar">
            {/* Mobile dropdown navigation */}
            <div className="settings-nav-mobile" ref={mobileNavRef}>
              <div className="settings-backup-actions">
                <button
                  type="button"
                  className="settings-backup-btn"
                  onClick={() => fileInputRef.current?.click()}
                  disabled={importing}
                  title="Import settings"
                >
                  <Icon name="upload" size={18} />
                  <span>{importing ? '...' : 'Import'}</span>
                </button>
                <button
                  type="button"
                  className="settings-backup-btn"
                  onClick={handleExport}
                  disabled={exporting}
                  title="Export settings"
                >
                  <Icon name="download" size={18} />
                  <span>{exporting ? '...' : 'Export'}</span>
                </button>
              </div>
              <button
                type="button"
                className={`settings-nav-dropdown-trigger ${mobileNavOpen ? 'open' : ''}`}
                onClick={() => setMobileNavOpen(!mobileNavOpen)}
                aria-label="Select settings panel"
              >
                <div className="settings-nav-dropdown-selected">
                  <div className="settings-nav-icon">{getPanelIcon(selectedPanelId || 'favorites')}</div>
                  <div className="settings-nav-copy">
                    <span className="settings-nav-label">
                      {selectedPanelId === 'favorites'
                        ? 'Favorites'
                        : panels.find((p) => p.id === selectedPanelId)?.name || 'Select Panel'}
                    </span>
                    <span className="settings-nav-desc">
                      {selectedPanelId === 'favorites'
                        ? 'Starred controls'
                        : panels.find((p) => p.id === selectedPanelId)?.description || ''}
                    </span>
                  </div>
                </div>
                <Icon name="expand_more" size={20} className="settings-nav-dropdown-chevron" />
              </button>
              {mobileNavOpen && (
                <div className="settings-nav-dropdown-menu">
                  <button
                    type="button"
                    className={`settings-nav-dropdown-item ${selectedPanelId === 'favorites' ? 'active' : ''}`}
                    onClick={() => handleMobileNavSelect('favorites')}
                  >
                    <div className="settings-nav-icon">{getPanelIcon('favorites')}</div>
                    <div className="settings-nav-copy">
                      <span className="settings-nav-label">Favorites</span>
                      <span className="settings-nav-desc">Starred controls</span>
                    </div>
                  </button>
                  {panels.map((panel) => (
                    <button
                      type="button"
                      key={panel.id}
                      className={`settings-nav-dropdown-item ${selectedPanelId === panel.id ? 'active' : ''}`}
                      onClick={() => handleMobileNavSelect(panel.id)}
                    >
                      <div className="settings-nav-icon">{getPanelIcon(panel.id)}</div>
                      <div className="settings-nav-copy">
                        <span className="settings-nav-label">{panel.name}</span>
                        <span className="settings-nav-desc">{panel.description || 'Panel controls'}</span>
                      </div>
                    </button>
                  ))}
                </div>
              )}
            </div>

            {/* Desktop button navigation */}
            <div className="settings-nav">
              {/* Import/Export buttons inline with nav */}
              <div className="settings-backup-actions">
                <button
                  type="button"
                  className="settings-backup-btn"
                  onClick={() => fileInputRef.current?.click()}
                  disabled={importing}
                  title="Import settings"
                >
                  <Icon name="upload" size={18} />
                  <span>{importing ? '...' : 'Import'}</span>
                </button>
                <button
                  type="button"
                  className="settings-backup-btn"
                  onClick={handleExport}
                  disabled={exporting}
                  title="Export settings"
                >
                  <Icon name="download" size={18} />
                  <span>{exporting ? '...' : 'Export'}</span>
                </button>
              </div>

              <button
                type="button"
                className={`settings-nav-btn ${selectedPanelId === 'favorites' ? 'active' : ''}`}
                onClick={() => setSelectedPanelId('favorites')}
              >
                <div className="settings-nav-icon">{getPanelIcon('favorites')}</div>
                <div className="settings-nav-copy">
                  <span className="settings-nav-label">Favorites</span>
                  <span className="settings-nav-desc">Starred controls</span>
                </div>
              </button>

              {panels.map((panel) => (
                <button
                  type="button"
                  key={panel.id}
                  className={`settings-nav-btn ${selectedPanelId === panel.id ? 'active' : ''}`}
                  onClick={() => setSelectedPanelId(panel.id)}
                  data-panel-id={panel.id}
                >
                  <div className="settings-nav-icon">{getPanelIcon(panel.id)}</div>
                  <div className="settings-nav-copy">
                    <span className="settings-nav-label">{panel.name}</span>
                    <span className="settings-nav-desc">{panel.description || 'Panel controls'}</span>
                  </div>
                </button>
              ))}
            </div>
          </aside>

          <section className="settings-main">
            <div className="settings-panel-header">
              <div className="settings-panel-heading">
                <div className="settings-panel-icon">{getPanelIcon(selectedPanelId || activeMetadata?.id)}</div>
                <div>
                  <h1>{activeMetadata?.name || 'Settings'}</h1>
                  <p>{headerDescription}</p>
                </div>
              </div>
              <div className="settings-search">
                <input
                  type="text"
                  className="settings-search-input"
                  placeholder="Search settings..."
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  aria-label="Search settings"
                />
                {searchQuery && (
                  <button
                    type="button"
                    className="settings-search-clear"
                    onClick={() => setSearchQuery('')}
                    aria-label="Clear search"
                  >
                    <Icon name="close" size={18} />
                  </button>
                )}
              </div>
            </div>

            <div className="settings-panel-content">
              {selectedPanelId === 'favorites' ? (
                <FavoritesPanel />
              ) : selectedPanel ? (
                <>
                  {!searchQuery && headerDescription && (
                    <div className="settings-panel-description">{headerDescription}</div>
                  )}

                  {filteredGroups && filteredGroups.length > 0 ? (
                    filteredGroups.map((group) => (
                      <PanelGroup
                        key={group.groupName}
                        group={group}
                        state={state}
                        panelId={selectedPanelId || undefined}
                      />
                    ))
                  ) : searchQuery ? (
                    <div className="settings-no-results">
                      <p>No settings found for "{searchQuery}"</p>
                      <button type="button" onClick={() => setSearchQuery('')}>Clear search</button>
                    </div>
                  ) : null}
                </>
              ) : (
                <div className="settings-panel-loading">
                  <LoadingSpinner message="Loading panel..." />
                </div>
              )}
            </div>
          </section>
        </div>
      </div>

      {/* Backup/Restore Result Modal */}
      {showBackupModal && backupResult && (
        <Modal
          isOpen={showBackupModal}
          title={backupResult.success ? 'Success' : 'Error'}
          onClose={() => setShowBackupModal(false)}
        >
          <p>{backupResult.message}</p>
          {backupResult.details && backupResult.details.failed > 0 && (
            <p className="settings-backup-detail">
              <strong>Failed:</strong> {backupResult.details.failed}
            </p>
          )}
          <div className="settings-backup-modal-actions">
            <Button variant="primary" onClick={() => setShowBackupModal(false)}>
              OK
            </Button>
          </div>
        </Modal>
      )}

      {/* Floating indicator for pending changes */}
      <FloatingChangesIndicator />

      <BackToTop />
    </>
  )
}
