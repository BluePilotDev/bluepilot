Place the HLS.js assets in this directory to allow offline use.

Required files (matching the version you want to ship):

- `hls.min.js`
- `hls.min.js.map` (optional, but recommended to avoid 404s if the script references it)

You can download them from the HLS.js release you are using (for example, from https://github.com/video-dev/hls.js/releases) and copy them here. After copying, the web UI will automatically prefer these local files and fall back to the CDN only if they are missing.
