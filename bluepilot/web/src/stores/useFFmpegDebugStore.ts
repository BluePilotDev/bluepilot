import { create } from 'zustand'

export interface FFmpegLogMessage {
  timestamp: string
  route_info: string
  log_type: 'start' | 'end' | 'output' | 'error'
  message: string
  pid?: number
}

interface FFmpegDebugState {
  messages: FFmpegLogMessage[]
  maxMessages: number

  // Actions
  addMessage: (message: FFmpegLogMessage) => void
  clearMessages: () => void
  setMaxMessages: (max: number) => void
}

export const useFFmpegDebugStore = create<FFmpegDebugState>((set) => ({
  messages: [],
  maxMessages: 500, // Limit messages to prevent memory issues

  addMessage: (message: FFmpegLogMessage) =>
    set((state) => {
      const newMessages = [...state.messages, message]
      // Trim to max messages to prevent memory issues
      if (newMessages.length > state.maxMessages) {
        return { messages: newMessages.slice(-state.maxMessages) }
      }
      return { messages: newMessages }
    }),

  clearMessages: () => set({ messages: [] }),

  setMaxMessages: (max: number) =>
    set((state) => ({
      maxMessages: max,
      messages: state.messages.slice(-max),
    })),
}))
