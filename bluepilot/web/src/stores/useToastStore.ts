import { create } from 'zustand'

export interface Toast {
  id: string
  message: string
  type: 'success' | 'error' | 'info'
  duration?: number
}

interface ToastState {
  toasts: Toast[]
  addToast: (message: string, type?: 'success' | 'error' | 'info', duration?: number) => void
  removeToast: (id: string) => void
  clearAllToasts: () => void
}

export const useToastStore = create<ToastState>((set) => ({
  toasts: [],

  addToast: (message: string, type: 'success' | 'error' | 'info' = 'info', duration?: number) => {
    const id = `${Date.now()}-${Math.random()}`

    // Default durations: success/info auto-dismiss, errors stay until manually closed
    const autoDismissDuration = type === 'error' ? undefined : (duration ?? 3000)
    const toast: Toast = { id, message, type, duration: autoDismissDuration }

    set((state) => ({
      toasts: [...state.toasts, toast]
    }))

    // Auto-remove toast after duration (if duration is set)
    if (autoDismissDuration !== undefined) {
      setTimeout(() => {
        set((state) => ({
          toasts: state.toasts.filter((t) => t.id !== id)
        }))
      }, autoDismissDuration)
    }
  },

  removeToast: (id: string) => {
    set((state) => ({
      toasts: state.toasts.filter((t) => t.id !== id)
    }))
  },

  clearAllToasts: () => {
    set({ toasts: [] })
  }
}))
