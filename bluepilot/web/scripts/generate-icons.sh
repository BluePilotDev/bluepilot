#!/bin/bash
# PWA Icon Generator for BluePilot
# Usage: ./scripts/generate-icons.sh <source-image.png>

set -e

SOURCE_IMG="$1"
ICONS_DIR="$(dirname "$0")/../public/icons"

if [ -z "$SOURCE_IMG" ]; then
    echo "Usage: $0 <source-image.png>"
    echo "Example: $0 ~/Downloads/bp-icon.png"
    exit 1
fi

if [ ! -f "$SOURCE_IMG" ]; then
    echo "Error: Source image not found: $SOURCE_IMG"
    exit 1
fi

mkdir -p "$ICONS_DIR"

echo "Generating PWA icons from: $SOURCE_IMG"
echo "Output directory: $ICONS_DIR"

# Standard PWA icon sizes
SIZES=(72 96 128 144 152 192 384 512)

for size in "${SIZES[@]}"; do
    echo "Creating icon-${size}.png..."
    sips -z "$size" "$size" "$SOURCE_IMG" --out "$ICONS_DIR/icon-${size}.png" >/dev/null 2>&1
done

# Apple Touch Icon (180x180)
echo "Creating apple-touch-icon.png (180x180)..."
sips -z 180 180 "$SOURCE_IMG" --out "$ICONS_DIR/apple-touch-icon.png" >/dev/null 2>&1

# Favicon (32x32)
echo "Creating favicon.png (32x32)..."
sips -z 32 32 "$SOURCE_IMG" --out "$ICONS_DIR/favicon.png" >/dev/null 2>&1

echo ""
echo "Done! Generated icons:"
ls -la "$ICONS_DIR"
