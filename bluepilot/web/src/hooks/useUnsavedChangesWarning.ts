/**
 * useUnsavedChangesWarning Hook
 * Warns users before leaving the page when there are unsaved changes
 */

import { useEffect } from 'react'
import { useChangeTrackingStore } from '@/stores/useChangeTrackingStore'

export function useUnsavedChangesWarning() {
  const hasChanges = useChangeTrackingStore((state) => state.hasChanges)

  useEffect(() => {
    const handleBeforeUnload = (e: BeforeUnloadEvent) => {
      if (hasChanges()) {
        // Standard way to show browser's native confirmation dialog
        e.preventDefault()
        // Some browsers require returnValue to be set
        e.returnValue = 'You have unsaved changes. Are you sure you want to leave?'
        return e.returnValue
      }
    }

    window.addEventListener('beforeunload', handleBeforeUnload)

    return () => {
      window.removeEventListener('beforeunload', handleBeforeUnload)
    }
  }, [hasChanges])
}
