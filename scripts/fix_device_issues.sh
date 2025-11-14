#!/usr/bin/env bash
#
# Fix Device Issues - Run this on the Comma device via SSH
#
# Usage:
#   ssh comma@10.0.1.125 'bash -s' < scripts/fix_device_issues.sh
#   OR
#   scp scripts/fix_device_issues.sh comma@10.0.1.125:/tmp/
#   ssh comma@10.0.1.125 "bash /tmp/fix_device_issues.sh"

set -e

echo "================================================"
echo "BluePilot Device Issue Fixer"
echo "================================================"
echo ""

# Fix 1: Kill duplicate manager processes
echo "[1/3] Checking for multiple manager processes..."
MANAGER_PIDS=$(ps aux | grep 'python3 ./manager.py' | grep -v grep | awk '{print $2}')
MANAGER_COUNT=$(echo "$MANAGER_PIDS" | wc -l)

if [ "$MANAGER_COUNT" -gt 1 ]; then
    echo "  Found $MANAGER_COUNT manager processes (should be 1)"
    echo "  Keeping the oldest process, killing others..."

    # Get oldest PID (first one)
    OLDEST_PID=$(echo "$MANAGER_PIDS" | head -1)

    # Kill all except oldest
    echo "$MANAGER_PIDS" | tail -n +2 | while read pid; do
        echo "  Killing duplicate manager PID: $pid"
        kill -15 "$pid" 2>/dev/null || true
    done

    echo "  ✓ Duplicate processes cleaned up"
else
    echo "  ✓ Only one manager process found (PID: $MANAGER_PIDS)"
fi

echo ""

# Fix 2: Clean up stale overlay locks (if any exist besides .overlay_init)
echo "[2/3] Cleaning up stale overlay locks..."
cd /data/openpilot
LOCK_FILES=$(find . -maxdepth 1 -name '.overlay*' ! -name '.overlay_init' 2>/dev/null)

if [ -n "$LOCK_FILES" ]; then
    echo "  Found stale lock files:"
    echo "$LOCK_FILES"
    rm -f .overlay_consistent .overlay_lock 2>/dev/null || true
    echo "  ✓ Stale locks removed"
else
    echo "  ✓ No stale locks found"
fi

echo ""

# Fix 3: Verify critical files exist
echo "[3/3] Verifying critical files..."
FILES_TO_CHECK=(
    "/data/openpilot/CHANGELOG.md"
    "/data/openpilot/sunnypilot/common/version.h"
)

ALL_OK=true
for file in "${FILES_TO_CHECK[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file exists"
    else
        echo "  ✗ MISSING: $file"
        ALL_OK=false
    fi
done

if [ "$ALL_OK" = true ]; then
    echo "  ✓ All critical files present"
fi

echo ""
echo "================================================"
echo "Fix Complete!"
echo "================================================"
echo ""
echo "Summary:"
echo "  - Manager processes: Fixed (if multiple were running)"
echo "  - Overlay locks: Cleaned"
echo "  - Critical files: Verified"
echo ""
echo "Next steps:"
echo "  1. Check Sentry for new errors: ./scripts/sentry_issues_agent.py stats"
echo "  2. Monitor the device for a few minutes"
echo "  3. If issues persist, check journalctl: journalctl -u comma -f"
echo ""








