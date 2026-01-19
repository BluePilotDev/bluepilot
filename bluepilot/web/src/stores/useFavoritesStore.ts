/**
 * Favorites Store
 * Manages bookmarked/favorited settings with backend persistence
 */

import { create } from 'zustand'
import { persist } from 'zustand/middleware'

export interface FavoriteControl {
  panelId: string
  groupName: string
  controlTitle: string
  param?: string
  timestamp: number
}

interface FavoritesState {
  favorites: FavoriteControl[]
  isLoaded: boolean
  isSyncing: boolean

  // Actions
  addFavorite: (favorite: Omit<FavoriteControl, 'timestamp'>) => void
  removeFavorite: (panelId: string, groupName: string, controlTitle: string) => void
  isFavorite: (panelId: string, groupName: string, controlTitle: string) => boolean
  clearFavorites: () => void
  loadFromBackend: () => Promise<void>
  setFavorites: (favorites: FavoriteControl[]) => void
}

// Debounce timer for saving to backend
let saveTimeout: ReturnType<typeof setTimeout> | null = null

// Save favorites to backend (debounced)
const saveToBackend = async (favorites: FavoriteControl[]) => {
  if (saveTimeout) {
    clearTimeout(saveTimeout)
  }

  saveTimeout = setTimeout(async () => {
    try {
      const response = await fetch('/api/favorite_settings', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ favorites }),
      })

      if (!response.ok) {
        console.error('Failed to save favorites to backend:', response.statusText)
      }
    } catch (error) {
      console.error('Error saving favorites to backend:', error)
    }
  }, 500) // 500ms debounce
}

export const useFavoritesStore = create<FavoritesState>()(
  persist(
    (set, get) => ({
      favorites: [],
      isLoaded: false,
      isSyncing: false,

      addFavorite: (favorite) => {
        const newFavorite = {
          ...favorite,
          timestamp: Date.now(),
        }
        set((state) => {
          const newFavorites = [...state.favorites, newFavorite]
          saveToBackend(newFavorites)
          return { favorites: newFavorites }
        })
      },

      removeFavorite: (panelId, groupName, controlTitle) => {
        set((state) => {
          const newFavorites = state.favorites.filter(
            (fav) =>
              !(
                fav.panelId === panelId &&
                fav.groupName === groupName &&
                fav.controlTitle === controlTitle
              )
          )
          saveToBackend(newFavorites)
          return { favorites: newFavorites }
        })
      },

      isFavorite: (panelId, groupName, controlTitle) => {
        return get().favorites.some(
          (fav) =>
            fav.panelId === panelId &&
            fav.groupName === groupName &&
            fav.controlTitle === controlTitle
        )
      },

      clearFavorites: () => {
        set({ favorites: [] })
        saveToBackend([])
      },

      setFavorites: (favorites) => {
        set({ favorites, isLoaded: true })
      },

      loadFromBackend: async () => {
        if (get().isSyncing) return

        set({ isSyncing: true })

        try {
          const response = await fetch('/api/favorite_settings')
          if (response.ok) {
            const data = await response.json()
            if (data.success && Array.isArray(data.favorites)) {
              const backendFavorites = data.favorites as FavoriteControl[]
              const localFavorites = get().favorites

              // If backend has favorites, use them (backend is source of truth)
              if (backendFavorites.length > 0) {
                set({ favorites: backendFavorites, isLoaded: true })
              } else if (localFavorites.length > 0 && !get().isLoaded) {
                // If backend is empty but we have local favorites, sync them to backend
                // This handles migration from localStorage-only to backend persistence
                saveToBackend(localFavorites)
                set({ isLoaded: true })
              } else {
                set({ isLoaded: true })
              }
            }
          }
        } catch (error) {
          console.error('Error loading favorites from backend:', error)
          // Keep using localStorage favorites as fallback
          set({ isLoaded: true })
        } finally {
          set({ isSyncing: false })
        }
      },
    }),
    {
      name: 'bluepilot-favorites', // localStorage key (used as fallback/cache)
    }
  )
)

// Auto-load from backend when store is created
// Use setTimeout to avoid blocking the initial render
setTimeout(() => {
  useFavoritesStore.getState().loadFromBackend()
}, 100)
