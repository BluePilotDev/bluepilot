/**
 * Video Playback Utilities
 * Cloned from app_old.js for consistent video playback behavior
 */

export interface BrowserCapabilities {
  isSafari: boolean
  isFirefox: boolean
  hevcSupported: boolean
  webglSupported: boolean
  hlsSupported: boolean
}

/**
 * Detect Safari browser
 */
export function detectSafari(): boolean {
  if (typeof navigator === 'undefined') return false
  const ua = navigator.userAgent
  return /safari/i.test(ua) && !/chrome|chromium|crios|android/i.test(ua)
}

/**
 * Detect Firefox browser
 */
export function detectFirefox(): boolean {
  if (typeof navigator === 'undefined') return false
  const ua = navigator.userAgent
  return /firefox/i.test(ua)
}

/**
 * Check for native HEVC/H.265 support
 * Works on Safari (all versions), Chrome 107+, Edge 107+
 */
export async function checkNativeHEVCSupport(): Promise<boolean> {
  try {
    const video = document.createElement('video')

    // Test multiple HEVC codec strings
    const hevcCodecs = [
      'video/mp4; codecs="hvc1.1.6.L93.B0"', // HEVC Main profile
      'video/mp4; codecs="hev1.1.6.L93.B0"', // Alternative HEVC format
      'video/mp4; codecs="hvc1"', // Simplified
      'video/mp4; codecs="hev1"', // Simplified alternative
    ]

    for (const codec of hevcCodecs) {
      const support = video.canPlayType(codec)
      if (support === 'probably' || support === 'maybe') {
        console.log(`Native HEVC support detected with codec: ${codec} (${support})`)
        return true
      }
    }

    console.log('No native HEVC support detected')
    return false
  } catch (e) {
    console.warn('Error checking HEVC support:', e)
    return false
  }
}

/**
 * Check for WebGL support (needed for h265-web-player fallback)
 */
export function checkWebGLSupport(): boolean {
  try {
    const canvas = document.createElement('canvas')

    // Check for WebGL 2.0 first (preferred)
    let gl: WebGL2RenderingContext | WebGLRenderingContext | null = canvas.getContext('webgl2')
    let version = 'WebGL 2.0'

    // Fall back to WebGL 1.0
    if (!gl) {
      gl = canvas.getContext('webgl') || canvas.getContext('experimental-webgl') as WebGLRenderingContext | null
      version = 'WebGL 1.0'
    }

    if (!gl) {
      console.warn('WebGL not supported at all')
      return false
    }

    console.log(`WebGL support detected: ${version}`)
    return true
  } catch (e) {
    console.warn('Error checking WebGL support:', e)
    return false
  }
}

/**
 * Check if browser supports native HLS playback
 * (Safari iOS/macOS and some other browsers)
 */
export function canUseNativeHLS(): boolean {
  const v = document.createElement('video')
  return !!(
    v.canPlayType('application/vnd.apple.mpegurl') ||
    v.canPlayType('application/x-mpegURL')
  )
}

/**
 * Check if HLS.js is loaded and supported
 */
export function isHLSJsSupported(): boolean {
  return typeof window.Hls !== 'undefined' && window.Hls.isSupported()
}

/**
 * Detect all browser capabilities
 */
export async function detectBrowserCapabilities(): Promise<BrowserCapabilities> {
  const isSafari = detectSafari()
  const isFirefox = detectFirefox()
  const hevcSupported = await checkNativeHEVCSupport()
  const webglSupported = checkWebGLSupport()
  const hlsSupported = isHLSJsSupported()

  console.log('=== Browser Capabilities Detected ===')
  console.log('Safari:', isSafari)
  console.log('Firefox:', isFirefox)
  console.log('Native HEVC:', hevcSupported)
  console.log('WebGL:', webglSupported)
  console.log('HLS.js:', hlsSupported)
  console.log('Native HLS:', canUseNativeHLS())

  if (typeof window.Hls !== 'undefined') {
    console.log('HLS.js version:', window.Hls.version || 'unknown')
    if (!window.Hls.isSupported()) {
      console.warn('⚠️ HLS.js is loaded but not supported on this browser')
    }
  } else {
    console.error('❌ HLS.js NOT loaded!')
  }

  return {
    isSafari,
    isFirefox,
    hevcSupported,
    webglSupported,
    hlsSupported
  }
}

/**
 * Load HLS.js script dynamically
 */
export function loadHLSScript(): Promise<void> {
  return new Promise((resolve, reject) => {
    if (typeof window.Hls !== 'undefined') {
      resolve()
      return
    }

    const script = document.createElement('script')
    script.src = '/vendor/hls/hls.min.js'
    script.onload = () => {
      console.log('HLS.js loaded successfully')
      resolve()
    }
    script.onerror = () => {
      console.error('Failed to load HLS.js')
      reject(new Error('Failed to load HLS.js'))
    }
    document.head.appendChild(script)
  })
}

/**
 * Load h265-web-player script dynamically
 */
export function loadH265PlayerScript(): Promise<void> {
  return new Promise((resolve, reject) => {
    if (typeof window.Player !== 'undefined') {
      resolve()
      return
    }

    const script = document.createElement('script')
    script.src = '/vendor/h265-web-player/player.js'
    script.onload = () => {
      console.log('h265-web-player loaded successfully')
      resolve()
    }
    script.onerror = () => {
      console.error('Failed to load h265-web-player')
      reject(new Error('Failed to load h265-web-player'))
    }
    document.head.appendChild(script)
  })
}

/**
 * Get HLS.js config for route playback
 */
export function getHLSConfig() {
  return {
    debug: true,
    enableWorker: true,
    lowLatencyMode: false,
    maxBufferLength: 30,
    maxMaxBufferLength: 600,
    maxBufferSize: 60 * 1000 * 1000,
    maxBufferHole: 0.5,
    enableSoftwareAES: true,
    fragLoadingTimeOut: 20000,
    manifestLoadingTimeOut: 10000,
    levelLoadingTimeOut: 10000,
    fragLoadingMaxRetry: 3,
    levelLoadingMaxRetry: 2,
    manifestLoadingMaxRetry: 2,
  }
}

// Extend Window interface for global video libraries
declare global {
  interface Window {
    Hls: any
    Player: any
    canvas: HTMLCanvasElement | null
  }
}
