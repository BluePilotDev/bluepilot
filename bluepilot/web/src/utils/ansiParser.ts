/**
 * ANSI escape code parser for converting console output to HTML with colors
 * Supports standard ANSI color codes, bright colors, and text styling
 */

interface AnsiState {
  bold: boolean
  dim: boolean
  italic: boolean
  underline: boolean
  fgColor: string | null
  bgColor: string | null
}

// ANSI color code mappings (standard colors)
const ANSI_COLORS: { [key: number]: string } = {
  // Foreground colors (30-37)
  30: 'ansi-black',
  31: 'ansi-red',
  32: 'ansi-green',
  33: 'ansi-yellow',
  34: 'ansi-blue',
  35: 'ansi-magenta',
  36: 'ansi-cyan',
  37: 'ansi-white',
  // Bright foreground colors (90-97)
  90: 'ansi-bright-black',
  91: 'ansi-bright-red',
  92: 'ansi-bright-green',
  93: 'ansi-bright-yellow',
  94: 'ansi-bright-blue',
  95: 'ansi-bright-magenta',
  96: 'ansi-bright-cyan',
  97: 'ansi-bright-white',
}

// Background color mappings
const ANSI_BG_COLORS: { [key: number]: string } = {
  // Background colors (40-47)
  40: 'ansi-bg-black',
  41: 'ansi-bg-red',
  42: 'ansi-bg-green',
  43: 'ansi-bg-yellow',
  44: 'ansi-bg-blue',
  45: 'ansi-bg-magenta',
  46: 'ansi-bg-cyan',
  47: 'ansi-bg-white',
  // Bright background colors (100-107)
  100: 'ansi-bg-bright-black',
  101: 'ansi-bg-bright-red',
  102: 'ansi-bg-bright-green',
  103: 'ansi-bg-bright-yellow',
  104: 'ansi-bg-bright-blue',
  105: 'ansi-bg-bright-magenta',
  106: 'ansi-bg-bright-cyan',
  107: 'ansi-bg-bright-white',
}

/**
 * Escape HTML special characters to prevent XSS
 */
function escapeHtml(text: string): string {
  const div = document.createElement('div')
  div.textContent = text
  return div.innerHTML
}

/**
 * Build CSS class string from ANSI state
 */
function buildClassString(state: AnsiState): string {
  const classes: string[] = []

  if (state.bold) classes.push('ansi-bold')
  if (state.dim) classes.push('ansi-dim')
  if (state.italic) classes.push('ansi-italic')
  if (state.underline) classes.push('ansi-underline')
  if (state.fgColor) classes.push(state.fgColor)
  if (state.bgColor) classes.push(state.bgColor)

  return classes.join(' ')
}

/**
 * Process ANSI escape codes and update state
 */
function processAnsiCode(code: string, state: AnsiState): void {
  const codes = code.split(';').map(c => parseInt(c, 10))

  for (const num of codes) {
    if (isNaN(num)) continue

    switch (num) {
      case 0: // Reset all
        state.bold = false
        state.dim = false
        state.italic = false
        state.underline = false
        state.fgColor = null
        state.bgColor = null
        break
      case 1: // Bold
        state.bold = true
        break
      case 2: // Dim
        state.dim = true
        break
      case 3: // Italic
        state.italic = true
        break
      case 4: // Underline
        state.underline = true
        break
      case 22: // Normal intensity (turn off bold/dim)
        state.bold = false
        state.dim = false
        break
      case 23: // Not italic
        state.italic = false
        break
      case 24: // Not underlined
        state.underline = false
        break
      case 39: // Default foreground color
        state.fgColor = null
        break
      case 49: // Default background color
        state.bgColor = null
        break
      default:
        // Check if it's a foreground color
        if (ANSI_COLORS[num]) {
          state.fgColor = ANSI_COLORS[num]
        }
        // Check if it's a background color
        else if (ANSI_BG_COLORS[num]) {
          state.bgColor = ANSI_BG_COLORS[num]
        }
        break
    }
  }
}

/**
 * Convert ANSI-colored text to HTML with CSS classes
 * Supports ANSI escape codes commonly used in console output
 *
 * @param text - Raw text with ANSI escape codes
 * @returns HTML string with span elements and CSS classes
 */
export function ansiToHtml(text: string): string {
  // Match ANSI escape sequences: ESC[...m
  // Supports both \x1b and \u001b escape characters
  const ansiRegex = /\x1b\[([0-9;]*)m/g

  const state: AnsiState = {
    bold: false,
    dim: false,
    italic: false,
    underline: false,
    fgColor: null,
    bgColor: null,
  }

  const parts: string[] = []
  let lastIndex = 0
  let match: RegExpExecArray | null

  while ((match = ansiRegex.exec(text)) !== null) {
    const textBefore = text.substring(lastIndex, match.index)

    // Add previous text with current styling
    if (textBefore.length > 0) {
      const classString = buildClassString(state)
      const escapedText = escapeHtml(textBefore)

      if (classString) {
        parts.push(`<span class="${classString}">${escapedText}</span>`)
      } else {
        parts.push(escapedText)
      }
    }

    // Process the ANSI code and update state
    const code = match[1]
    processAnsiCode(code, state)

    lastIndex = match.index + match[0].length
  }

  // Add remaining text
  const remainingText = text.substring(lastIndex)
  if (remainingText.length > 0) {
    const classString = buildClassString(state)
    const escapedText = escapeHtml(remainingText)

    if (classString) {
      parts.push(`<span class="${classString}">${escapedText}</span>`)
    } else {
      parts.push(escapedText)
    }
  }

  return parts.join('')
}

/**
 * Strip ANSI escape codes from text (for plain text output)
 *
 * @param text - Raw text with ANSI escape codes
 * @returns Plain text without ANSI codes
 */
export function stripAnsi(text: string): string {
  return text.replace(/\x1b\[[0-9;]*m/g, '')
}
