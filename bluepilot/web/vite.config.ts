import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { VitePWA } from 'vite-plugin-pwa'
import path from 'path'

// Backend URL - can be overridden with VITE_BACKEND_URL env var
// Default: localhost:8088 for dev, set to device IP for preview testing
const BACKEND_URL = process.env.VITE_BACKEND_URL || 'http://localhost:8088'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [
    react(),
    VitePWA({
      registerType: 'autoUpdate',
      injectRegister: 'auto',
      devOptions: {
        enabled: true, // Enable service worker in dev mode for testing
      },
      workbox: {
        // Only precache the main JS/CSS bundles to avoid 429 rate limiting
        // Other assets will be cached at runtime when first requested
        globPatterns: ['assets/*.{js,css}'],
        // Disable navigation fallback - let the browser handle page loads normally
        navigateFallback: null,
        runtimeCaching: [
          {
            // Cache vendor scripts at runtime (avoids precache 429 errors)
            urlPattern: /\/vendor\/.*/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'vendor-cache',
              expiration: {
                maxEntries: 20,
                maxAgeSeconds: 60 * 60 * 24 * 30, // 30 days
              },
            },
          },
          {
            // Cache icons at runtime
            urlPattern: /\/icons\/.*/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'icon-cache',
              expiration: {
                maxEntries: 20,
                maxAgeSeconds: 60 * 60 * 24 * 30, // 30 days
              },
            },
          },
          {
            // Cache fonts
            urlPattern: /\.(?:woff2?|ttf)$/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'font-cache',
              expiration: {
                maxEntries: 10,
                maxAgeSeconds: 60 * 60 * 24 * 30, // 30 days
              },
            },
          },
          {
            // Cache images
            urlPattern: /\.(?:png|jpg|jpeg|svg|gif|webp|ico)$/i,
            handler: 'CacheFirst',
            options: {
              cacheName: 'image-cache',
              expiration: {
                maxEntries: 50,
                maxAgeSeconds: 60 * 60 * 24 * 30, // 30 days
              },
            },
          },
        ],
      },
      manifest: {
        name: 'BluePilot',
        short_name: 'BluePilot',
        description: 'BluePilot Web Interface - Route Manager and System Monitor',
        theme_color: '#151515',
        background_color: '#151515',
        display: 'standalone',
        orientation: 'portrait',
        scope: '/',
        start_url: '/',
        categories: ['automotive', 'utilities'],
        icons: [
          {
            src: 'icons/icon-72.png',
            sizes: '72x72',
            type: 'image/png',
          },
          {
            src: 'icons/icon-96.png',
            sizes: '96x96',
            type: 'image/png',
          },
          {
            src: 'icons/icon-128.png',
            sizes: '128x128',
            type: 'image/png',
          },
          {
            src: 'icons/icon-144.png',
            sizes: '144x144',
            type: 'image/png',
          },
          {
            src: 'icons/icon-152.png',
            sizes: '152x152',
            type: 'image/png',
          },
          {
            src: 'icons/icon-192.png',
            sizes: '192x192',
            type: 'image/png',
          },
          {
            src: 'icons/icon-384.png',
            sizes: '384x384',
            type: 'image/png',
          },
          {
            src: 'icons/icon-512.png',
            sizes: '512x512',
            type: 'image/png',
          },
          {
            src: 'icons/icon-512.png',
            sizes: '512x512',
            type: 'image/png',
            purpose: 'maskable',
          },
        ],
      },
    }),
  ],

  // Disable public directory to avoid conflicts with build output
  publicDir: false,

  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },

  // Build configuration to output to public/ folder
  build: {
    outDir: 'public',
    emptyOutDir: false, // Don't delete vendor/ folder

    rollupOptions: {
      output: {
        // Organize built assets
        assetFileNames: (assetInfo) => {
          const info = assetInfo.name.split('.')
          const ext = info[info.length - 1]

          if (/png|jpe?g|svg|gif|tiff|bmp|ico/i.test(ext)) {
            return 'assets/images/[name]-[hash][extname]'
          } else if (/woff2?|ttf|eot/i.test(ext)) {
            return 'assets/fonts/[name]-[hash][extname]'
          }

          return 'assets/[name]-[hash][extname]'
        },
        chunkFileNames: 'assets/[name]-[hash].js',
        entryFileNames: 'assets/[name]-[hash].js',
      },
    },

    // Optimize chunk splitting for better caching
    chunkSizeWarningLimit: 1000,

    // Minification settings
    minify: 'terser',
    terserOptions: {
      compress: {
        drop_console: false, // Keep console.logs for debugging
      },
    },
  },

  // Dev server configuration
  server: {
    port: 5173,
    strictPort: false,
    host: true, // Listen on all addresses

    // Proxy API requests to Python backend during development
    proxy: {
      '/api': {
        target: BACKEND_URL,
        changeOrigin: true,
      },
      '/_internal': {
        target: BACKEND_URL,
        changeOrigin: true,
      },
    },
  },

  // Preview server configuration (for testing production builds)
  preview: {
    port: 4173,
    host: true,

    // Proxy API and WebSocket requests to backend device
    proxy: {
      '/api': {
        target: BACKEND_URL,
        changeOrigin: true,
      },
      '/_internal': {
        target: BACKEND_URL,
        changeOrigin: true,
      },
      '/ws': {
        target: BACKEND_URL.replace('http', 'ws').replace(':8088', ':8089'),
        ws: true,
        changeOrigin: true,
      },
    },
  },
})
