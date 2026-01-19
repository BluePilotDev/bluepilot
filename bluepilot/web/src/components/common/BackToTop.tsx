import { useState, useEffect, useCallback, type RefObject } from 'react'
import { Icon } from './Icon'
import './BackToTop.css'

interface BackToTopProps {
  /** Container element ref to monitor scroll position. If not provided, uses window scroll */
  containerRef?: RefObject<HTMLElement | null>
  /** Scroll threshold (px) before showing the button */
  threshold?: number
  /** Additional CSS class */
  className?: string
}

export function BackToTop({ containerRef, threshold = 300, className = '' }: BackToTopProps) {
  const [isVisible, setIsVisible] = useState(false)

  const checkScrollPosition = useCallback(() => {
    const container = containerRef?.current
    if (container) {
      setIsVisible(container.scrollTop > threshold)
    } else {
      setIsVisible(window.scrollY > threshold)
    }
  }, [containerRef, threshold])

  useEffect(() => {
    const container = containerRef?.current
    const target = container || window

    target.addEventListener('scroll', checkScrollPosition, { passive: true })
    checkScrollPosition()

    return () => {
      target.removeEventListener('scroll', checkScrollPosition)
    }
  }, [containerRef, checkScrollPosition])

  const scrollToTop = useCallback(() => {
    const container = containerRef?.current
    if (container) {
      container.scrollTo({ top: 0, behavior: 'smooth' })
    } else {
      window.scrollTo({ top: 0, behavior: 'smooth' })
    }
  }, [containerRef])

  if (!isVisible) {
    return null
  }

  return (
    <button
      type="button"
      className={`back-to-top ${className}`}
      onClick={scrollToTop}
      aria-label="Scroll to top"
      title="Back to top"
    >
      <Icon name="keyboard_arrow_up" size={24} />
    </button>
  )
}
