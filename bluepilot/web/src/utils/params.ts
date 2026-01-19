import type { Parameter } from '@/types'

export interface FormattedParamValue {
  display: string
  raw: string
  isBinary: boolean
  decodedString?: string | null
  formatLabel?: string
  hexPreview?: string
}

const tryParseJsonString = (value: string): string | null => {
  const trimmed = value.trim()
  if ((!trimmed.startsWith('{') || !trimmed.endsWith('}')) && (!trimmed.startsWith('[') || !trimmed.endsWith(']'))) {
    return null
  }
  try {
    const parsed = JSON.parse(trimmed)
    return JSON.stringify(parsed, null, 2)
  } catch {
    return null
  }
}

const decodeBase64 = (encoded: string): string => {
  if (typeof atob === 'function') {
    return atob(encoded)
  }
  // eslint-disable-next-line @typescript-eslint/ban-ts-comment
  // @ts-ignore - Buffer is only available in Node environments, fallback for SSR tooling
  if (typeof Buffer !== 'undefined') {
    // eslint-disable-next-line @typescript-eslint/ban-ts-comment
    // @ts-ignore - Buffer typing may not exist in browser builds
    return Buffer.from(encoded, 'base64').toString('binary')
  }
  throw new Error('No base64 decoder available in this environment')
}

export const base64ToHex = (base64?: string | null, limit = 256): string => {
  if (!base64) return ''
  try {
    const binary = decodeBase64(base64).slice(0, limit)
    return Array.from(binary)
      .map((char) => char.charCodeAt(0).toString(16).padStart(2, '0').toUpperCase())
      .join(' ')
  } catch {
    return ''
  }
}

export const formatParamValueForDisplay = (param: Parameter): FormattedParamValue => {
  if (param.type === 'bytes') {
    const preview =
      param.decoded?.preview ??
      (param.byte_length !== undefined ? `${param.byte_length} bytes` : 'binary data')
    const decodedString = (() => {
      if (!param.decoded) return null
      if (param.decoded.format === 'text') {
        const data = param.decoded.data
        return typeof data === 'string' ? data : String(data ?? '')
      }
      if (param.decoded.data) {
        try {
          return JSON.stringify(param.decoded.data, null, 2)
        } catch {
          return null
        }
      }
      return null
    })()

    return {
      display: preview,
      raw: param.raw_value ?? preview,
      isBinary: true,
      decodedString,
      formatLabel: param.decoded?.format ?? (param.raw_format ? param.raw_format.toUpperCase() : 'binary'),
      hexPreview: base64ToHex(param.raw_value, 384),
    }
  }

  if (param.value === null || param.value === undefined) {
    return {
      display: 'null',
      raw: 'null',
      isBinary: false,
    }
  }

  if (typeof param.value === 'boolean' || typeof param.value === 'number') {
    const strValue = String(param.value)
    return {
      display: strValue,
      raw: strValue,
      isBinary: false,
    }
  }

  if (typeof param.value === 'string') {
    const asJson = tryParseJsonString(param.value)
    if (asJson) {
      return {
        display: asJson,
        raw: param.value,
        isBinary: false,
        formatLabel: 'JSON',
      }
    }
    return {
      display: param.value,
      raw: param.value,
      isBinary: false,
    }
  }

  const fallback = (() => {
    try {
      return JSON.stringify(param.value, null, 2)
    } catch {
      return String(param.value)
    }
  })()

  return {
    display: fallback,
    raw: fallback,
    isBinary: false,
  }
}
