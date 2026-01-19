import { useState, useEffect, useRef, useCallback } from 'react'
import { useNavigate } from 'react-router-dom'
import { useRoutesStore } from '@/stores/useRoutesStore'
import { useToastStore } from '@/stores/useToastStore'
import { ConfirmDialog, Icon } from '@/components/common'
import { RouteDownloadModal } from '@/components/modals'
import type { RouteDetails, CameraType } from '@/types'
import { VehicleInfoPanel } from './panels/VehicleInfoPanel'
import { LogsPanel } from './panels/LogsPanel'
import { CerealDataPanel } from './panels/CerealDataPanel'
import { FFmpegDebugPanel } from './panels/FFmpegDebugPanel'
import {
  detectBrowserCapabilities,
  loadHLSScript,
  getHLSConfig,
  canUseNativeHLS,
  type BrowserCapabilities
} from '@/utils/videoPlayback'
import './VideoPlayer.css'

interface VideoPlayerProps {
  route: RouteDetails
  onClose: () => void
}

export const VideoPlayer: React.FC<VideoPlayerProps> = ({ route, onClose }) => {
  const navigate = useNavigate()
  const { deleteRoute, preserveRoute } = useRoutesStore()
  const { addToast } = useToastStore()
  const [showExportModal, setShowExportModal] = useState(false)
  const [showDeleteConfirm, setShowDeleteConfirm] = useState(false)
  const videoRef = useRef<HTMLVideoElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)
  const videoWrapperRef = useRef<HTMLDivElement>(null)

  const [currentCamera, setCurrentCamera] = useState<CameraType>('front')
  const [currentSegment, setCurrentSegment] = useState(0)
  const [currentTime, setCurrentTime] = useState(0)
  const [buffering, setBuffering] = useState(false)
  const [showDebugPanel, setShowDebugPanel] = useState(false)
  const [showReplayOverlay, setShowReplayOverlay] = useState(false)
  const [browserCaps, setBrowserCaps] = useState<BrowserCapabilities | null>(null)
  const [availableCameras, setAvailableCameras] = useState<CameraType[]>(['front'])
  const [useCanvas, setUseCanvas] = useState(true) // Start with canvas for h265-web-player

  // HLS instance (will be initialized if needed)
  const hlsRef = useRef<any>(null)
  const h265PlayerRef = useRef<any>(null)
  const retryCountRef = useRef(0)
  const maxRetries = 3
  const lastPlaybackTimeRef = useRef(0)
  const bufferErrorCountRef = useRef(0)
  const MAX_BUFFER_ERRORS = 5
  const canvasTimeIntervalRef = useRef<ReturnType<typeof setInterval> | null>(null)
  const canvasStartTimeRef = useRef<number>(0)
  const currentSegmentRef = useRef(0)

  // Detect browser capabilities on mount
  useEffect(() => {
    const init = async () => {
      // Load HLS.js script
      try {
        await loadHLSScript()
      } catch (e) {
        console.error('Failed to load HLS.js:', e)
      }

      // Detect capabilities
      const caps = await detectBrowserCapabilities()
      setBrowserCaps(caps)
    }

    init()
  }, [])

  // Detect available cameras from route segments
  useEffect(() => {
    if (route.segments && route.segments.length > 0) {
      const cameras = new Set<CameraType>()
      route.segments.forEach(segment => {
        Object.keys(segment.videos).forEach(camera => {
          cameras.add(camera as CameraType)
        })
      })
      const cameraList = Array.from(cameras).sort((a, b) => {
        const order: CameraType[] = ['front', 'wide', 'driver', 'lq']
        return order.indexOf(a) - order.indexOf(b)
      })
      setAvailableCameras(cameraList)

      // Set initial camera based on HEVC support (matching old logic)
      if (cameraList.length > 0 && browserCaps) {
        const canPlayHEVC = browserCaps.hevcSupported
        let cameraPriority: CameraType[]

        if (canPlayHEVC) {
          // HEVC supported: prioritize HEVC cameras (front -> wide -> driver -> lq)
          cameraPriority = ['front', 'wide', 'driver', 'lq']
        } else {
          // No HEVC support: only use LQ camera
          cameraPriority = ['lq']
        }

        // Select first available camera from priority list
        let selectedCamera: CameraType | null = null
        for (const camera of cameraPriority) {
          if (cameraList.includes(camera)) {
            selectedCamera = camera
            break
          }
        }

        if (selectedCamera) {
          setCurrentCamera(selectedCamera)
        } else if (cameraList.length > 0) {
          // Fallback to first available camera
          setCurrentCamera(cameraList[0])
        }
      } else if (cameraList.length > 0) {
        // Fallback if browserCaps not yet loaded
        setCurrentCamera(cameraList.includes('front') ? 'front' : cameraList[0])
      }
    }
  }, [route, browserCaps])

  // Check if camera is available in current segment
  const isCameraAvailableInSegment = useCallback((camera: CameraType, segmentIndex: number): boolean => {
    if (!route.segments || segmentIndex >= route.segments.length) return false
    const segment = route.segments[segmentIndex]
    return !!segment.videos?.[camera]
  }, [route])

  // Check if HEVC camera can be played
  const canPlayHEVCCamera = useCallback((camera: CameraType): boolean => {
    if (camera === 'lq') return true // LQ is always playable (H.264)
    return browserCaps?.hevcSupported ?? false
  }, [browserCaps])

  // Handle camera switching with playback time sync and availability checks
  const handleCameraChange = useCallback((newCamera: CameraType) => {
    const video = videoRef.current
    if (!video || !browserCaps) return

    // Check if camera is available for current segment
    if (!isCameraAvailableInSegment(newCamera, currentSegment)) {
      console.warn(`Camera ${newCamera} not available in segment ${currentSegment + 1}`)
      return
    }

    // Prevent switching to HEVC cameras if not supported
    const isHEVCCamera = newCamera !== 'lq'
    if (isHEVCCamera && !canPlayHEVCCamera(newCamera)) {
      console.warn(`Cannot switch to ${newCamera} camera - HEVC not supported`)
      return
    }

    // Store current playback time before switching
    lastPlaybackTimeRef.current = video.currentTime
    console.log(`Storing playback time for camera sync: ${lastPlaybackTimeRef.current}s`)
    setCurrentCamera(newCamera)
  }, [currentSegment, browserCaps, isCameraAvailableInSegment, canPlayHEVCCamera])

  // Switch to video element (from canvas) when using HLS.js or native HLS
  const switchToVideoElement = useCallback(() => {
    if (!useCanvas || videoRef.current) return // Already switched

    console.log('Switching from canvas to video element for HLS playback')

    // Clear canvas time tracking interval when switching away from canvas
    if (canvasTimeIntervalRef.current) {
      clearInterval(canvasTimeIntervalRef.current)
      canvasTimeIntervalRef.current = null
    }

    setUseCanvas(false)
  }, [useCanvas])

  // Load h265-web-player for HEVC playback on browsers without native support
  const loadH265WebPlayer = useCallback(() => {
    const canvas = canvasRef.current
    const segmentIndex = currentSegmentRef.current
    if (!canvas || !route.segments || segmentIndex >= route.segments.length) {
      console.error('Canvas element not available for h265-web-player')
      setBuffering(false)
      return
    }

    const segment = route.segments[segmentIndex]
    const videoUrl = `/api/video/${route.baseName}/${segment.number}/${currentCamera}`

    try {
      console.log('Initializing h265-web-player for:', videoUrl)
      setBuffering(true)

      // Ensure canvas is visible and has proper dimensions
      const canvasWidth = canvas.clientWidth || 1280
      const canvasHeight = canvas.clientHeight || 720

      canvas.width = canvasWidth
      canvas.height = canvasHeight
      canvas.style.width = canvasWidth + 'px'
      canvas.style.height = canvasHeight + 'px'

      console.log('Canvas dimensions:', canvasWidth, 'x', canvasHeight)

      // Create WebGL context FIRST before initializing player
      const glCtx = canvas.getContext('webgl') || canvas.getContext('experimental-webgl')

      if (!glCtx) {
        console.warn('WebGL context not available, falling back to HLS')
        switchToVideoElement()
        loadHLSPlayback()
        return
      }

      console.log('WebGL context created successfully')

      // Set global canvas variable for h265-web-player library
      window.canvas = canvas

      const player = new window.Player()
      player.init({
        width: canvasWidth,
        height: canvasHeight,
        canvas: canvas,
        url: videoUrl,
      })

      h265PlayerRef.current = player
      console.log('h265-web-player initialized successfully')

      // Hide loading spinner after a short delay
      setTimeout(() => {
        setBuffering(false)
      }, 1000)

      // Start time tracking for canvas player (h265-web-player doesn't fire timeupdate events)
      // Clear any existing interval
      if (canvasTimeIntervalRef.current) {
        clearInterval(canvasTimeIntervalRef.current)
      }

      // Start time tracking for this segment
      canvasStartTimeRef.current = Date.now()

      // Update currentTime every 100ms
      canvasTimeIntervalRef.current = setInterval(() => {
        const elapsedMs = Date.now() - canvasStartTimeRef.current
        const elapsedSeconds = elapsedMs / 1000
        const currentSeg = currentSegmentRef.current
        const newTime = currentSeg * 60 + elapsedSeconds

        setCurrentTime(newTime)

        // Auto-advance to next segment after ~60 seconds
        if (elapsedSeconds >= 60) {
          clearInterval(canvasTimeIntervalRef.current!)
          canvasTimeIntervalRef.current = null

          const nextSegment = currentSeg + 1
          if (nextSegment < route.segments.length) {
            setCurrentSegment(nextSegment)
          } else {
            setShowReplayOverlay(true)
          }
        }
      }, 100)
    } catch (error) {
      console.error('Error initializing h265-web-player:', error)
      setBuffering(false)

      // Fallback to LQ camera
      console.log('Falling back to LQ camera due to h265-web-player error')
      if (currentCamera !== 'lq') {
        setCurrentCamera('lq')
      } else {
        // Last resort: switch to video element for HLS playback
        switchToVideoElement()
        setTimeout(() => {
          const video = videoRef.current
          if (video) {
            const hlsUrl = `/api/hls/${route.baseName}/${currentCamera}/playlist.m3u8`
            video.src = hlsUrl
            video.load()
          }
        }, 100)
      }
    }
  }, [route, currentCamera, switchToVideoElement])

  // Load native HLS playback (Safari)
  const loadNativeHLS = useCallback(() => {
    const video = videoRef.current
    if (!video) return

    const hlsUrl = `/api/hls/${route.baseName}/${currentCamera}/playlist.m3u8`

    console.log('Using Safari native HLS')
    console.log('Loading HLS URL:', hlsUrl)

    // Set muted attribute for programmatic play (matching old code)
    video.muted = true
    video.setAttribute('playsinline', '')
    video.setAttribute('preload', 'metadata')

    // Add handlers for native HLS (matching old code structure)
    const handleLoadedMetadata = () => {
      console.log('Native HLS loaded, total duration:', video.duration)
      setBuffering(false)

      // Sync to stored playback time when switching cameras
      if (lastPlaybackTimeRef.current > 0) {
        console.log(`Seeking to synced time: ${lastPlaybackTimeRef.current}s`)
        video.currentTime = lastPlaybackTimeRef.current
        lastPlaybackTimeRef.current = 0
      }
    }

    const handleTimeUpdate = () => {
      const t = video.currentTime
      // Calculate segment based on time (with small epsilon to handle rounding)
      const newSegment = Math.min(
        Math.floor(t / 60 + 0.0001),
        (route.segments?.length || 1) - 1
      )

      // Use functional update to avoid stale closure
      setCurrentSegment((prevSegment) => {
        if (newSegment !== prevSegment && newSegment >= 0 && newSegment < route.segments.length) {
          return newSegment
        }
        return prevSegment
      })
    }

    const handleError = () => {
      console.error('Native HLS video element error event')
      const err = video.error
      if (err) {
        console.error('Video Error Code:', err.code)
        console.error('Video Error Message:', err.message)
      } else {
        console.error('Video error object not available.')
      }
      setBuffering(false)
    }

    const handleEnded = () => {
      console.log('Native HLS: Playback ended')
      setShowReplayOverlay(true)
      if (!video.paused) {
        video.pause()
      }
    }

    // Use direct property assignment (matching old code)
    video.onloadedmetadata = handleLoadedMetadata
    video.onerror = handleError
    video.onended = handleEnded
    video.ontimeupdate = handleTimeUpdate

    // Set the HLS playlist URL
    video.src = hlsUrl
    video.load()

    // Show loading state
    setBuffering(true)

    // Attempt programmatic play (matching old code)
    // iOS often needs muted+playsinline set (done above)
    setTimeout(() => {
      video
        .play()
        .then(() => {
          console.log('Native HLS playback started successfully')
          setBuffering(false)
        })
        .catch((e) => {
          console.warn('Autoplay failed (may require user gesture):', e)
          setBuffering(false)
        })
    }, 100)
  }, [route, currentCamera])

  // Load HLS.js playback for full route
  const loadHLSPlayback = useCallback(() => {
    const video = videoRef.current
    if (!video) return

    // Use HLS.js for full route streaming (allows scrubbing across all segments)
    const hlsUrl = `/api/hls/${route.baseName}/${currentCamera}/playlist.m3u8`

    console.log('Loading HLS route playlist:', hlsUrl)

    setBuffering(true)

    // All other browsers - use HLS.js for consistent behavior
    if (typeof window.Hls !== 'undefined' && window.Hls.isSupported()) {
      console.log('Using HLS.js for route playback')

      const hls = new window.Hls(getHLSConfig())

      hls.loadSource(hlsUrl)
      hls.attachMedia(video)

      hls.on(window.Hls.Events.MANIFEST_PARSED, () => {
        console.log('HLS manifest parsed - full route available for scrubbing')
        setBuffering(false)

        // Sync to stored playback time when switching cameras
        if (lastPlaybackTimeRef.current > 0) {
          console.log(`Seeking to synced time: ${lastPlaybackTimeRef.current}s`)
          video.currentTime = lastPlaybackTimeRef.current
          lastPlaybackTimeRef.current = 0 // Reset after use
        }

        // Autoplay disabled - user must click play (matching old code)
        // video.play().catch((e) => {
        //   console.warn('Autoplay failed:', e)
        // })
      })

      hls.on(window.Hls.Events.ERROR, (_event: any, data: any) => {
        // Log error details for debugging
        if (data.fatal) {
          console.error('HLS fatal error:', data)
        } else {
          // Non-fatal errors - log as warning (matching old code behavior)
          console.warn('Non-fatal HLS error:', data.details, data)
        }

        if (data.fatal) {
          switch (data.type) {
            case window.Hls.ErrorTypes.NETWORK_ERROR:
              console.error('Fatal network error, trying to recover...')
              if (retryCountRef.current < maxRetries) {
                retryCountRef.current++
                setTimeout(() => {
                  console.log(`Retry attempt ${retryCountRef.current}`)
                  hls.startLoad()
                }, 1000)
              } else {
                console.error('Max retries reached')
                hls.destroy()
                hlsRef.current = null
                setBuffering(false)
              }
              break

            case window.Hls.ErrorTypes.MEDIA_ERROR:
              console.error('Fatal media error, trying to recover...')
              try {
                hls.recoverMediaError()
              } catch (e) {
                console.error('Media recovery failed:', e)
                hls.destroy()
                hlsRef.current = null
                setBuffering(false)
              }
              break

            default:
              console.error('Unrecoverable HLS error')
              hls.destroy()
              hlsRef.current = null
              setBuffering(false)
              break
          }
        } else {
          // Non-fatal errors - let HLS.js handle retries automatically
          // fragParsingError is handled automatically by HLS.js with retry logic
          // We just need to track buffer errors for iOS/Safari

          // Handle bufferFullError specifically for iOS/Safari
          if (data.details === 'bufferFullError') {
            bufferErrorCountRef.current++
            console.warn(
              `Buffer full error (${bufferErrorCountRef.current}/${MAX_BUFFER_ERRORS})`
            )

            if (bufferErrorCountRef.current >= MAX_BUFFER_ERRORS) {
              console.error(
                'Too many consecutive buffer errors - destroying HLS instance'
              )
              hls.destroy()
              hlsRef.current = null
              setBuffering(false)
            }
          } else {
            // Reset counter on other error types (including fragParsingError)
            // HLS.js handles fragParsingError retries automatically
            bufferErrorCountRef.current = 0
          }
        }
      })

      // Reset buffer error counter on successful fragment loads
      hls.on(window.Hls.Events.FRAG_LOADED, () => {
        if (bufferErrorCountRef.current > 0) {
          console.log('Fragment loaded successfully - resetting buffer error count')
          bufferErrorCountRef.current = 0
        }
      })

      // Additional event handlers for debugging (matching old code)
      hls.on(window.Hls.Events.BUFFER_APPENDING, (_event: any, data: any) => {
        // Log buffer events for debugging
        if (data.type === 'video') {
          console.debug('Appending video buffer:', data.data?.length, 'bytes')
        }
      })

      hls.on(window.Hls.Events.BUFFER_EOS, () => {
        console.debug('Buffer reached end of stream')
      })

      hls.on(window.Hls.Events.BUFFER_RESET, () => {
        console.debug('Buffer reset')
      })

      hlsRef.current = hls
    } else if (canUseNativeHLS()) {
      // Fallback to native HLS for browsers that support it
      console.log('Using native HLS (fallback)')

      const handleError = () => {
        console.error('Native HLS fallback error')
        const err = video.error
        if (err) {
          console.error('Error Code:', err.code, 'Message:', err.message)
        }
        setBuffering(false)
      }

      const handleEnded = () => {
        console.log('Native HLS fallback: Playback ended')
        setShowReplayOverlay(true)
        if (!video.paused) {
          video.pause()
        }
      }

      video.addEventListener('error', handleError, { once: true })
      video.addEventListener('ended', handleEnded)
      video.src = hlsUrl
      video.load()
      setBuffering(true)
    } else {
      console.error('No HLS support available')
      setBuffering(false)
    }
  }, [route, currentCamera])

  // Wrapper functions to call from video initialization effect
  const callLoadH265WebPlayer = useCallback(() => {
    loadH265WebPlayer()
  }, [loadH265WebPlayer])

  const callLoadHLSPlayback = useCallback(() => {
    loadHLSPlayback()
  }, [loadHLSPlayback])

  const callLoadNativeHLS = useCallback(() => {
    loadNativeHLS()
  }, [loadNativeHLS])

  // Initialize video player with proper strategy
  useEffect(() => {
    if (!browserCaps) return

    const isHEVCCamera = currentCamera !== 'lq'

    // Clean up existing instances
    if (hlsRef.current) {
      try {
        hlsRef.current.destroy()
      } catch (e) {
        console.error('Error destroying HLS:', e)
      }
      hlsRef.current = null
    }

    if (h265PlayerRef.current) {
      h265PlayerRef.current = null
    }

    console.log('=== Video Playback Strategy ===')
    console.log('Camera:', currentCamera)
    console.log('Is HEVC camera:', isHEVCCamera)

    // Strategy for HEVC cameras
    if (isHEVCCamera) {
      // For HEVC cameras, we can only use HLS.js if the browser has native HEVC decoding
      // (because backend serves raw HEVC, not fMP4)
      if (browserCaps.isSafari && canUseNativeHLS()) {
        // Safari with native HLS and HEVC support
        console.log('Using native Safari HLS for HEVC')
        switchToVideoElement()
        setTimeout(callLoadNativeHLS, 100)
      } else if (browserCaps.hevcSupported && typeof window.Hls !== 'undefined' && window.Hls.isSupported()) {
        // Browser has native HEVC decoding + HLS.js (e.g., Edge, newer Chrome)
        console.log('Using HLS.js with native HEVC decoding')
        switchToVideoElement()
        setTimeout(callLoadHLSPlayback, 100)
      } else {
        // No native HEVC support - try h265-web-player (WebAssembly decoder)
        const playerAvailable = typeof window.Player !== 'undefined'
        if (playerAvailable && browserCaps.webglSupported) {
          console.log('Using h265-web-player for HEVC (no native support)')
          setTimeout(callLoadH265WebPlayer, 100)
        } else {
          // Last resort: fallback to LQ camera
          console.log('No HEVC playback method available, falling back to LQ camera')
          setCurrentCamera('lq')
        }
      }
    } else {
      // LQ camera
      if (typeof window.Hls !== 'undefined' && window.Hls.isSupported()) {
        switchToVideoElement()
        setTimeout(callLoadHLSPlayback, 100)
      } else if (canUseNativeHLS()) {
        switchToVideoElement()
        setTimeout(callLoadNativeHLS, 100)
      } else {
        switchToVideoElement()
        setTimeout(callLoadHLSPlayback, 100)
      }
    }

    return () => {
      // Cleanup on unmount
      if (hlsRef.current) {
        try {
          hlsRef.current.destroy()
        } catch (e) {
          console.error('Error destroying HLS on cleanup:', e)
        }
        hlsRef.current = null
      }

      if (h265PlayerRef.current) {
        h265PlayerRef.current = null
      }

      // Clear canvas time tracking interval
      if (canvasTimeIntervalRef.current) {
        clearInterval(canvasTimeIntervalRef.current)
        canvasTimeIntervalRef.current = null
      }

      const video = videoRef.current
      if (video) {
        video.pause()
        video.src = ''
        video.load()
      }

      retryCountRef.current = 0
      bufferErrorCountRef.current = 0
      lastPlaybackTimeRef.current = 0
    }
  }, [browserCaps, currentCamera, switchToVideoElement, callLoadH265WebPlayer, callLoadHLSPlayback, callLoadNativeHLS])

  // Check camera availability when segment changes
  // NOTE: For HLS playback, the entire route is streamed continuously, so we should NOT
  // reload the video when segments change. Only check camera availability on initial load
  // or when user manually switches cameras, not during normal playback segment transitions.
  useEffect(() => {
    if (!route.segments || currentSegment >= route.segments.length) return

    const segment = route.segments[currentSegment]
    const availableCameras = Object.keys(segment.videos || {}) as CameraType[]

    // Only switch cameras if:
    // 1. Current camera is not available in this segment, AND
    // 2. We're not using HLS (which streams the entire route continuously)
    // For HLS playback, segment changes are just UI updates, not actual video switches
    const isUsingHLS = hlsRef.current !== null || (browserCaps?.isSafari && canUseNativeHLS())

    if (!isUsingHLS && !availableCameras.includes(currentCamera) && availableCameras.length > 0) {
      console.warn(
        `Camera ${currentCamera} not available in segment ${currentSegment + 1}, switching to ${availableCameras[0]}`
      )
      setCurrentCamera(availableCameras[0])
    }
  }, [currentSegment, route.segments, currentCamera, browserCaps])

  // Replay route handler
  const handleReplay = useCallback(() => {
    console.log('Replaying route from beginning')
    setShowReplayOverlay(false)
    const video = videoRef.current
    if (video) {
      video.currentTime = 0
      video.play().catch((e) => {
        console.warn('Replay autoplay failed:', e)
      })
    }
  }, [])

  // Keep segment ref in sync with state (for use in event handlers without stale closures)
  useEffect(() => {
    currentSegmentRef.current = currentSegment
  }, [currentSegment])

  // Video event handlers
  useEffect(() => {
    const video = videoRef.current
    if (!video) return

    const handleTimeUpdate = () => {
      const time = video.currentTime
      setCurrentTime(time)

      // Track which segment we're in during HLS playback for auto-reloading logs/cereal
      if (hlsRef.current && route.segments.length > 0) {
        const calculatedSegment = Math.floor(time / 60) // Each segment is 60 seconds

        // If we've moved to a different segment, update (use ref to avoid stale closure)
        if (
          calculatedSegment !== currentSegmentRef.current &&
          calculatedSegment < route.segments.length &&
          calculatedSegment >= 0
        ) {
          console.log(`Segment changed: ${currentSegmentRef.current} → ${calculatedSegment}`)
          setCurrentSegment(calculatedSegment)
        }
      }
    }

    const handleLoadStart = () => {
      console.log('Video loadstart event')
      setBuffering(true)
    }

    const handleLoadedMetadata = () => {
      console.log('Video metadata loaded:', {
        duration: video.duration,
        videoWidth: video.videoWidth,
        videoHeight: video.videoHeight,
      })
    }

    const handlePlay = () => {
      setBuffering(false)
    }

    const handlePlaying = () => {
      console.log('Video is playing')
      setBuffering(false)
    }

    const handleWaiting = () => {
      setBuffering(true)
    }

    const handleCanPlay = () => {
      console.log('Video can play')
      setBuffering(false)
    }

    const handleEnded = () => {
      console.log('Full route playback completed')
      setShowReplayOverlay(true)
      // Pause the video if it's playing
      if (!video.paused) {
        video.pause()
      }
    }

    video.addEventListener('loadstart', handleLoadStart)
    video.addEventListener('loadedmetadata', handleLoadedMetadata)
    video.addEventListener('timeupdate', handleTimeUpdate)
    video.addEventListener('play', handlePlay)
    video.addEventListener('playing', handlePlaying)
    video.addEventListener('waiting', handleWaiting)
    video.addEventListener('canplay', handleCanPlay)
    video.addEventListener('ended', handleEnded)

    return () => {
      video.removeEventListener('loadstart', handleLoadStart)
      video.removeEventListener('loadedmetadata', handleLoadedMetadata)
      video.removeEventListener('timeupdate', handleTimeUpdate)
      video.removeEventListener('play', handlePlay)
      video.removeEventListener('playing', handlePlaying)
      video.removeEventListener('waiting', handleWaiting)
      video.removeEventListener('canplay', handleCanPlay)
      video.removeEventListener('ended', handleEnded)
    }
  }, [route.segments.length, useCanvas])

  // Native video controls handle all interactions

  const handleBack = () => {
    onClose()
    navigate('/routes')
  }

  const handlePreserve = async (e: React.MouseEvent) => {
    e.stopPropagation()
    try {
      await preserveRoute(route.baseName || route.id || '')
      addToast('Route preservation toggled successfully', 'success')
    } catch (error: any) {
      console.error('Failed to preserve route:', error)
      addToast(error?.message || 'Failed to preserve route', 'error')
    }
  }

  const handleDelete = (e: React.MouseEvent) => {
    e.stopPropagation()
    setShowDeleteConfirm(true)
  }

  const handleConfirmDelete = async () => {
    try {
      await deleteRoute(route.baseName || route.id || '')
      addToast('Route deleted successfully', 'success')
      handleBack() // Close player and return to routes list
    } catch (error) {
      console.error('Failed to delete route:', error)
      addToast('Failed to delete route. Please try again.', 'error')
    }
  }

  return (
    <>
      <RouteDownloadModal
        isOpen={showExportModal}
        onClose={() => setShowExportModal(false)}
        route={route}
      />
      <ConfirmDialog
        isOpen={showDeleteConfirm}
        onClose={() => setShowDeleteConfirm(false)}
        onConfirm={handleConfirmDelete}
        title="Delete Route"
        message={`Are you sure you want to delete route "${route.date}"? This action cannot be undone and all video files and logs will be permanently removed.`}
        confirmText="Delete"
        cancelText="Cancel"
        variant="danger"
      />
      <div className="video-player" ref={containerRef}>
        <div className="video-header">
          <div className="video-header-left">
            <button
              className="video-back-btn"
              onClick={handleBack}
              title="Back to routes"
            >
              <Icon name="arrow_back" size={18} />
              <span>Routes</span>
            </button>
            <div className="video-title-container">
              <h2 className="video-title">{route.date}</h2>
              <div className="video-route-id">{route.baseName}</div>
            </div>
          </div>
          <div className="video-header-actions">
            <button
              type="button"
              className="icon-btn"
              title="Toggle FFmpeg Debug Logs"
              aria-label="Toggle debug logs"
              onClick={() => setShowDebugPanel(!showDebugPanel)}
            >
              <Icon name="terminal" size={24} />
            </button>
          </div>
        </div>

      <div className="player-container">
        <div className="video-section">
          <div className="video-wrapper" ref={videoWrapperRef}>
            {/* Canvas element for h265-web-player (shown by default) */}
            {useCanvas && (
              <canvas
                ref={canvasRef}
                className="video-element"
                style={{ width: '100%', height: '100%', backgroundColor: '#000' }}
              />
            )}

            {/* Video element for HLS.js/native HLS (shown when switched) */}
            {!useCanvas && (
              <video
                ref={videoRef}
                className="video-element"
                controls
                playsInline
                muted
                preload="metadata"
                style={{ width: '100%', height: '100%', backgroundColor: '#000' }}
              />
            )}

            {buffering && (
              <div className="video-loading">
                <div className="spinner"></div>
              </div>
            )}

            {showReplayOverlay && (
              <div className="video-replay">
                <button className="replay-btn" onClick={handleReplay}>
                  <Icon name="replay" size={48} />
                  <span>Replay Route</span>
                </button>
              </div>
            )}
          </div>

          {/* Player Footer - Camera Selector and Segment Info */}
          <div className="player-footer">
            <div className="player-footer-content">
              {availableCameras.length > 1 && (
                <div className="camera-buttons">
                  {availableCameras.map((camera) => {
                    const isHEVCCamera = camera !== 'lq'
                    const canPlay = isHEVCCamera ? canPlayHEVCCamera(camera) : true
                    const isAvailable = isCameraAvailableInSegment(camera, currentSegment)
                    const isActive = camera === currentCamera

                    // Hide HEVC camera buttons if HEVC is not supported
                    if (isHEVCCamera && !canPlay) {
                      return null
                    }

                    return (
                      <button
                        key={camera}
                        type="button"
                        className={`camera-btn ${isActive ? 'active' : ''}`}
                        onClick={() => handleCameraChange(camera)}
                        disabled={!isAvailable}
                        title={`${camera.toUpperCase()} camera${isHEVCCamera ? ' (HEVC)' : ' (H.264)'}`}
                      >
                        {camera.charAt(0).toUpperCase() + camera.slice(1)}
                      </button>
                    )
                  })}
                </div>
              )}
              <div className="segment-status">
                <span className="segment-status-label">Active Segment</span>
                <span className="segment-status-value">
                  Segment {currentSegment + 1} of {route.segments.length} -{' '}
                  {currentCamera === 'lq'
                    ? 'LQ (H.264)'
                    : `${currentCamera.toUpperCase()} (HEVC)`}
                  {' | '}
                  <span style={{ fontFamily: 'monospace', fontSize: '11px' }}>
                    {useCanvas ? 'Canvas' : 'Video'} | Time: {currentTime.toFixed(1)}s
                  </span>
                </span>
              </div>
            </div>
          </div>
        </div>

        {/* Sidebar Section */}
        <div className="player-sidebar">
          {/* Route Details */}
          <div className="sidebar-group sidebar-header">
            <h2 className="player-route-title">Route Details</h2>
            <div className="player-route-summary">
              <div className="player-route-time">
                <Icon name="schedule" size={16} />
                <span>{route.start_time || route.baseName}</span>
              </div>
              {(route.start_location || route.end_location) && (
                <div className="player-route-location">
                  <span className="location-label">Route</span>
                  <div className="location-path">
                    <span>{route.start_location || 'N/A'}</span>
                    <span className="location-arrow">→</span>
                    <span>{route.end_location || 'N/A'}</span>
                  </div>
                </div>
              )}
            </div>
            <div className="player-route-stats">
              <div className="player-route-stat">
                <span className="stat-label">Duration</span>
                <span className="stat-value">{route.duration || '--'}</span>
              </div>
              <div className="player-route-stat">
                <span className="stat-label">Segments</span>
                <span className="stat-value">{route.segments.length}</span>
              </div>
              <div className="player-route-stat">
                <span className="stat-label">Size</span>
                <span className="stat-value">{route.size || '--'}</span>
              </div>
              <div className="player-route-stat">
                <span className="stat-label">Distance</span>
                <span className="stat-value">{route.distance || route.mileage || '--'}</span>
              </div>
              <div className="player-route-stat">
                <span className="stat-label">Avg Speed</span>
                <span className="stat-value">{route.avg_speed || route.avgSpeed || '--'}</span>
              </div>
              <div className="player-route-stat">
                <span className="stat-label">Top Speed</span>
                <span className="stat-value">{route.top_speed || route.topSpeed || '--'}</span>
              </div>
              <div className="player-route-stat">
                <span className="stat-label">OP Engaged</span>
                <span className="stat-value">
                  {route.op_engaged_percent !== undefined
                    ? `${Math.round(route.op_engaged_percent)}%`
                    : route.driveStats?.opEngagedPercent !== undefined
                    ? `${Math.round(route.driveStats.opEngagedPercent)}%`
                    : '--'}
                </span>
              </div>
              <div className="player-route-stat">
                <span className="stat-label">Disengagements</span>
                <span className="stat-value">
                  {route.disengagements || route.driveStats?.disengagementCount || '--'}
                </span>
              </div>
              <div className="player-route-stat">
                <span className="stat-label">Alerts</span>
                <span className="stat-value">
                  {route.alert_count || route.alerts || route.driveStats?.alertCount || '--'}
                </span>
              </div>
            </div>
          </div>

          {/* Route Actions */}
          <div className="sidebar-group">
            <h3 className="sidebar-group-title">Actions</h3>
            <div className="route-actions-sidebar">
              <button type="button" className={`btn btn-secondary ${route.preserved ? 'active' : ''}`} onClick={handlePreserve} title={route.preserved ? 'Preserved' : 'Preserve this route'}>
                <Icon name={route.preserved ? 'star' : 'star_border'} size={18} />
                <span>{route.preserved ? 'Preserved' : 'Preserve'}</span>
              </button>
              <button type="button" className="btn btn-primary" title="Export videos and backup route data" onClick={() => setShowExportModal(true)}>
                <Icon name="download" size={18} />
                Export Videos & Logs
              </button>
              <button type="button" className="btn btn-danger" onClick={handleDelete}>
                <Icon name="delete" size={18} />
                Delete Route
              </button>
            </div>
            <div className="route-actions-info">
              <strong>Tip:</strong> Preserved routes are protected from automatic deletion but still count towards storage usage.
            </div>
          </div>
        </div>
      </div>

      {/* Bottom Panels */}
      <div className="bottom-panels">
        <FFmpegDebugPanel show={showDebugPanel} />
        <VehicleInfoPanel route={route} />
        <LogsPanel
          route={route}
          currentSegment={currentSegment}
          videoCurrentTime={currentTime}
        />
        <CerealDataPanel
          route={route}
          currentSegment={currentSegment}
          videoCurrentTime={currentTime}
        />
      </div>
      </div>
    </>
  )
}
