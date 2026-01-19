Place the h265-web-player distribution files in this directory to enable offline playback for browsers that require the WebGL player.

Recommended files for version 1.0.2:

- `webgl.js`
- `pcm-player.js`
- `decoder.js`
- `player.js`
- `decoder.wasm`
- `decoder.data` (or similarly named resource file if provided by the release)

Download them from the project's release archive (e.g., https://github.com/structureio/h265web.js or the npm package) and copy them here. The web UI will attempt to load the local copies first and will fall back to the CDN only if a file is missing.
