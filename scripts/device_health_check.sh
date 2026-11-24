#!/usr/bin/env bash
#
# Device Health Check - Monitor your Comma device for potential issues
#
# Usage:
#   ./scripts/device_health_check.sh comma@10.0.1.125  (remote via SSH)
#   ./scripts/device_health_check.sh comma@localhost   (on device itself)
#   ./scripts/device_health_check.sh                   (on device, no SSH)
#

set -e

DEVICE_SSH=${1:-""}

# Check if we're running on the device itself
if [ -z "$DEVICE_SSH" ] || [ "$DEVICE_SSH" == "comma@localhost" ] || [ -d "/data/openpilot" ]; then
  # Running on device - use direct commands
  RUN_CMD=""
  DEVICE_SSH="local"
else
  # Running remotely - use SSH
  RUN_CMD="ssh $DEVICE_SSH"
fi

echo "================================================"
echo "BluePilot Device Health Check"
echo "Device: $DEVICE_SSH"
echo "================================================"
echo ""

# Get device info
echo "[Device Information]"
eval "$RUN_CMD" "cat /data/params/d/DongleId 2>/dev/null || echo unknown" | xargs echo "Dongle ID:"
eval "$RUN_CMD" "cat /data/params/d/Version 2>/dev/null || echo unknown" | xargs echo "Version:"
eval "$RUN_CMD" "cd /data/openpilot && git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown" | xargs echo "Git Branch:"
eval "$RUN_CMD" "cd /data/openpilot && git rev-parse --short HEAD 2>/dev/null || echo unknown" | xargs echo "Git Commit:"
eval "$RUN_CMD" "uptime | awk -F'up ' '{print \$2}' | awk -F',' '{print \$1}'" | xargs echo "Uptime:"
echo ""

# Check processes
echo "[Process Status]"
eval "$RUN_CMD" "ps aux | grep 'python3 ./manager.py' | grep -v grep | wc -l" | xargs echo "Manager processes:"
eval "$RUN_CMD" "ps aux | grep -E '(selfdrive|sunnypilot)' | grep -v grep | wc -l" | xargs echo "Total openpilot processes:"
echo ""

# Check recent crashes
echo "[Recent Crashes]"
eval "$RUN_CMD" "ls -lt /data/community/crashes/*.log 2>/dev/null | head -3" || echo "  No recent crashes"
echo ""

# Check disk space
echo "[Disk Space]"
eval "$RUN_CMD" "df -h /data | tail -1 | awk '{print \"  Usage: \" \$5 \" (\" \$3 \" / \" \$2 \")\"}'"
echo ""

# Check overlay status
echo "[Overlay Status]"
eval "$RUN_CMD" "ls -la /data/openpilot/.overlay* 2>/dev/null | awk '{print \"  \" \$NF}'" || echo "  No overlay locks"
echo ""

# Check critical files
echo "[Critical Files]"
FILES=(
  "/data/openpilot/CHANGELOG.md"
  "/data/openpilot/sunnypilot/common/version.h"
  "/data/openpilot/BPVERSION"
)

for file in "${FILES[@]}"; do
  eval "$RUN_CMD" "if [ -f '$file' ]; then echo '  ✓ $file'; else echo '  ✗ MISSING: $file'; fi"
done
echo ""

# Check for multiple publishers
echo "[Messaging Health]"
eval "$RUN_CMD" "lsof -i :8001-8200 2>/dev/null | grep -c LISTEN" | xargs echo "  Active ZMQ listeners:" || echo "  Unable to check"
echo ""

# Check recent Sentry errors (if device has reported any)
echo "[Recent Local Errors]"
eval "$RUN_CMD" "tail -5 /data/community/crashes/error.log 2>/dev/null" || echo "  No recent error log"
echo ""

# Check memory
echo "[Memory Usage]"
eval "$RUN_CMD" "free -h | grep Mem | awk '{print \"  Total: \" \$2 \"  Used: \" \$3 \"  Free: \" \$4}'"
echo ""

# Check if system is responsive
echo "[System Status]"
eval "$RUN_CMD" "systemctl is-active comma 2>/dev/null" | xargs echo "  Comma service:" || echo "  Service status unknown"
echo ""

echo "================================================"
echo "Health Check Complete!"
echo "================================================"
echo ""
echo "To monitor live:"
echo "  ssh $DEVICE_SSH 'journalctl -u comma -f'"
echo ""
echo "To check Sentry for device-specific issues:"
echo "  ./scripts/sentry_issues_agent.py list"
echo "  # Look for errors with your Dongle ID"
echo ""

