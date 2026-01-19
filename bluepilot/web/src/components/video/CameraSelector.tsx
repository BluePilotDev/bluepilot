import type { CameraType } from '@/types'
import './CameraSelector.css'

interface CameraSelectorProps {
  cameras: CameraType[]
  currentCamera: CameraType
  onCameraChange: (camera: CameraType) => void
}

const CAMERA_LABELS: Record<CameraType, string> = {
  front: 'Front',
  wide: 'Wide',
  driver: 'Driver',
  lq: 'LQ',
}

export const CameraSelector: React.FC<CameraSelectorProps> = ({
  cameras,
  currentCamera,
  onCameraChange,
}) => {
  if (cameras.length <= 1) {
    return null
  }

  return (
    <div className="camera-selector">
      {cameras.map((camera) => (
        <button
          key={camera}
          className={`camera-btn ${camera === currentCamera ? 'active' : ''}`}
          onClick={() => onCameraChange(camera)}
          title={`Switch to ${CAMERA_LABELS[camera]} camera`}
        >
          {CAMERA_LABELS[camera]}
        </button>
      ))}
    </div>
  )
}
