import { useCallback, useEffect, useRef, useState } from 'react'
import { Header } from '@/components/layout/Header'
import type { DeviceStatus } from '@/types'
import './VASMView.css'

type Side = 'left' | 'right'
type Point = [number, number]

interface VASMConfig {
  width: number
  height: number
  poly_left: Point[]
  poly_right: Point[]
}

interface VASMViewProps {
  deviceStatus?: DeviceStatus
}

const emptyConfig = (): VASMConfig => ({ width: 0, height: 0, poly_left: [], poly_right: [] })

export const VASMView = ({ deviceStatus = 'checking' }: VASMViewProps) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const imageRef = useRef<HTMLImageElement | null>(null)
  const [config, setConfig] = useState<VASMConfig>(emptyConfig)
  const [editing, setEditing] = useState<Side | null>(null)
  const [loading, setLoading] = useState(true)
  const [saving, setSaving] = useState(false)
  const [message, setMessage] = useState('')
  const [error, setError] = useState('')

  const draw = useCallback(() => {
    const canvas = canvasRef.current
    const image = imageRef.current
    if (!canvas) return
    const context = canvas.getContext('2d')
    if (!context) return

    context.clearRect(0, 0, canvas.width, canvas.height)
    if (image) context.drawImage(image, 0, 0, canvas.width, canvas.height)
    else {
      context.fillStyle = '#121826'
      context.fillRect(0, 0, canvas.width, canvas.height)
      context.fillStyle = '#94a3b8'
      context.font = '18px sans-serif'
      context.textAlign = 'center'
      context.fillText('Load a parked driver-camera snapshot to begin', canvas.width / 2, canvas.height / 2)
    }

    const polygons: Array<{ side: Side; points: Point[]; stroke: string; fill: string }> = [
      { side: 'left', points: config.poly_left, stroke: '#38bdf8', fill: 'rgba(56, 189, 248, 0.28)' },
      { side: 'right', points: config.poly_right, stroke: '#fb923c', fill: 'rgba(251, 146, 60, 0.28)' },
    ]
    for (const polygon of polygons) {
      if (!polygon.points.length) continue
      context.beginPath()
      context.moveTo(...polygon.points[0])
      polygon.points.slice(1).forEach(point => context.lineTo(...point))
      if (polygon.points.length >= 3) {
        context.closePath()
        context.fillStyle = polygon.fill
        context.fill()
      }
      context.strokeStyle = polygon.stroke
      context.lineWidth = editing === polygon.side ? 4 : 2
      context.stroke()
      polygon.points.forEach(([x, y]) => {
        context.beginPath()
        context.arc(x, y, 6, 0, Math.PI * 2)
        context.fillStyle = '#ffffff'
        context.fill()
        context.strokeStyle = polygon.stroke
        context.stroke()
      })
    }
  }, [config, editing])

  const applyNativeConfig = useCallback((saved: VASMConfig, image: HTMLImageElement) => {
    const width = Math.min(image.naturalWidth, 1280)
    const height = Math.round(width * image.naturalHeight / image.naturalWidth)
    const scaleX = width / (saved.width || image.naturalWidth)
    const scaleY = height / (saved.height || image.naturalHeight)
    setConfig({
      width,
      height,
      poly_left: (saved.poly_left || []).map(([x, y]) => [x * scaleX, y * scaleY]),
      poly_right: (saved.poly_right || []).map(([x, y]) => [x * scaleX, y * scaleY]),
    })
  }, [])

  const load = useCallback(async () => {
    setLoading(true)
    setError('')
    setMessage('')
    try {
      const [configResponse, snapshotResponse] = await Promise.all([
        fetch('/api/vasm/config'),
        fetch('/api/vasm/snapshot', { cache: 'no-store' }),
      ])
      if (!snapshotResponse.ok) {
        const payload = await snapshotResponse.json().catch(() => ({}))
        throw new Error(payload.error || 'No driver-camera snapshot is available')
      }
      const saved = configResponse.ok ? await configResponse.json() as VASMConfig : emptyConfig()
      const objectUrl = URL.createObjectURL(await snapshotResponse.blob())
      const image = new Image()
      image.onload = () => {
        imageRef.current = image
        applyNativeConfig(saved, image)
        setMessage('Choose a side, then tap around the matching window region.')
        setLoading(false)
        URL.revokeObjectURL(objectUrl)
      }
      image.onerror = () => {
        setError('The driver-camera snapshot could not be decoded.')
        setLoading(false)
        URL.revokeObjectURL(objectUrl)
      }
      image.src = objectUrl
    } catch (loadError) {
      setError(loadError instanceof Error ? loadError.message : 'V-ASM setup could not be loaded')
      setLoading(false)
    }
  }, [applyNativeConfig])

  useEffect(() => { load() }, [load])
  useEffect(() => { draw() }, [draw])

  const addPoint = (event: React.MouseEvent<HTMLCanvasElement>) => {
    if (!editing || !imageRef.current) return
    const canvas = canvasRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const point: Point = [
      Math.round((event.clientX - rect.left) * canvas.width / rect.width),
      Math.round((event.clientY - rect.top) * canvas.height / rect.height),
    ]
    const key = editing === 'left' ? 'poly_left' : 'poly_right'
    setConfig(current => ({ ...current, [key]: [...current[key], point] }))
  }

  const updateSide = (side: Side, operation: 'undo' | 'clear') => {
    const key = side === 'left' ? 'poly_left' : 'poly_right'
    setConfig(current => ({
      ...current,
      [key]: operation === 'clear' ? [] : current[key].slice(0, -1),
    }))
  }

  const save = async () => {
    const image = imageRef.current
    if (!image || (config.poly_left.length < 3 && config.poly_right.length < 3)) {
      setError('Draw at least one region with three or more points.')
      return
    }
    if ((config.poly_left.length > 0 && config.poly_left.length < 3) ||
        (config.poly_right.length > 0 && config.poly_right.length < 3)) {
      setError('Each drawn region needs at least three points, or it must be cleared.')
      return
    }

    setSaving(true)
    setError('')
    try {
      const scaleX = image.naturalWidth / config.width
      const scaleY = image.naturalHeight / config.height
      const scale = (points: Point[]) => points.map(([x, y]): Point => [Math.round(x * scaleX), Math.round(y * scaleY)])
      const response = await fetch('/api/vasm/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          width: image.naturalWidth,
          height: image.naturalHeight,
          poly_left: scale(config.poly_left),
          poly_right: scale(config.poly_right),
        }),
      })
      const payload = await response.json()
      if (!response.ok) throw new Error(payload.error || 'Could not save camera regions')
      setEditing(null)
      setMessage('Camera regions saved. V-ASM is enabled.')
    } catch (saveError) {
      setError(saveError instanceof Error ? saveError.message : 'Could not save camera regions')
    } finally {
      setSaving(false)
    }
  }

  const clearSavedConfig = async () => {
    if (!window.confirm('Clear both camera regions and disable V-ASM?')) return
    const response = await fetch('/api/vasm/config', { method: 'DELETE' })
    if (response.ok) {
      setConfig(current => ({ ...current, poly_left: [], poly_right: [] }))
      setEditing(null)
      setMessage('Camera regions cleared. V-ASM is disabled.')
      setError('')
    } else {
      setError('Could not clear the saved camera regions.')
    }
  }

  return (
    <>
      <Header deviceStatus={deviceStatus} subtitle="Parked setup using the driver camera" />
      <main className="vasm-page">
        <section className="vasm-card">
          <div className="vasm-intro">
            <h2>Window monitoring regions</h2>
            <p>Mark only the portions of the driver-camera image that look through the left and right windows. V-ASM supplements the vehicle’s OEM blind-spot signal.</p>
          </div>

          <div className="vasm-toolbar">
            <button className={editing === 'left' ? 'active left' : ''} onClick={() => setEditing('left')} disabled={loading}>Draw left</button>
            <button className={editing === 'right' ? 'active right' : ''} onClick={() => setEditing('right')} disabled={loading}>Draw right</button>
            <button onClick={() => editing && updateSide(editing, 'undo')} disabled={!editing}>Undo point</button>
            <button onClick={() => editing && updateSide(editing, 'clear')} disabled={!editing}>Clear side</button>
            <button onClick={load} disabled={loading}>New snapshot</button>
          </div>

          <div className="vasm-canvas-wrap">
            <canvas
              ref={canvasRef}
              width={config.width || 640}
              height={config.height || 480}
              onClick={addPoint}
              aria-label="Driver camera V-ASM annotation canvas"
            />
            {loading && <div className="vasm-loading">Loading snapshot…</div>}
          </div>

          {error && <div className="vasm-notice error">{error}</div>}
          {message && !error && <div className="vasm-notice">{message}</div>}

          <div className="vasm-actions">
            <button className="danger" onClick={clearSavedConfig} disabled={saving}>Clear saved setup</button>
            <button className="primary" onClick={save} disabled={loading || saving}>{saving ? 'Saving…' : 'Save and enable V-ASM'}</button>
          </div>
          <p className="vasm-safety">Experimental driver aid only. False positives and missed vehicles are possible—always check mirrors and look over your shoulder.</p>
        </section>
      </main>
    </>
  )
}
