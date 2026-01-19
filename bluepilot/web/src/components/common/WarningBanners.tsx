import { useState, useEffect } from 'react'
import { Icon } from './Icon'
import './WarningBanners.css'

export const WarningBanners = () => {
  const [showCellular, setShowCellular] = useState(false)
  const [showFirefox, setShowFirefox] = useState(false)

  useEffect(() => {
    // Check if Firefox
    const isFirefox = navigator.userAgent.toLowerCase().includes('firefox')
    if (isFirefox) {
      setShowFirefox(true)
    }

    // Check for cellular connection (placeholder - would need backend support)
    // setShowCellular(checkCellularStatus())
  }, [])

  if (!showCellular && !showFirefox) {
    return null
  }

  return (
    <>
      {showCellular && (
        <div className="cellular-warning">
          <div className="cellular-warning-content">
            <Icon name="warning" size={24} />
            <div className="cellular-warning-text">
              <strong>Cellular Access Enabled</strong>
              <span>Server accessible over cellular network</span>
            </div>
            <button className="cellular-warning-close" onClick={() => setShowCellular(false)} title="Dismiss">
              <Icon name="close" size={20} />
            </button>
          </div>
        </div>
      )}

      {showFirefox && (
        <div className="firefox-warning">
          <div className="firefox-warning-content">
            <Icon name="info" size={24} />
            <div className="firefox-warning-text">
              <strong>Limited Functionality on Firefox</strong>
              <span>
                HEVC video playback is not supported. Only the LQ (H.264) camera will be available for route viewing. For
                full functionality, please use Safari, Chrome, or Edge.
              </span>
            </div>
            <button className="firefox-warning-close" onClick={() => setShowFirefox(false)} title="Dismiss">
              <Icon name="close" size={20} />
            </button>
          </div>
        </div>
      )}
    </>
  )
}
