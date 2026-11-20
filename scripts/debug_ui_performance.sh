#!/usr/bin/env bash
# Tool to help debug UI performance issues and watchdog timeouts

set -e

DEVICE_IP="${1:-10.0.1.125}"
LOG_FILE="/data/ui_performance.log"

echo "=== BluePilot UI Performance Debugger ==="
echo "Device: comma@$DEVICE_IP"
echo ""

# Check if UI is running
echo "[1/4] Checking UI process status..."
ssh "comma@$DEVICE_IP" "ps aux | grep -E '\\./ui\$' | grep -v grep" || echo "  WARNING: UI process not running!"
echo ""

# Check for recent performance issues
echo "[2/4] Checking for recent performance warnings..."
ssh "comma@$DEVICE_IP" "tail -50 /data/community/swaglog.log 2>/dev/null | grep -i 'bp.ui.perf\\|watchdog\\|timeout' | tail -20" || echo "  No recent warnings found"
echo ""

# Fetch performance log
echo "[3/4] Fetching performance log..."
if ssh "comma@$DEVICE_IP" "test -f $LOG_FILE"; then
    scp "comma@$DEVICE_IP:$LOG_FILE" /tmp/ui_performance.log
    echo "  Downloaded to /tmp/ui_performance.log"

    # Analyze the log
    echo ""
    echo "[4/4] Performance Analysis:"
    echo "  Total slow operations: $(wc -l < /tmp/ui_performance.log)"
    echo ""
    echo "  Slowest operations (top 10):"
    sort -t',' -k3 -n -r /tmp/ui_performance.log | head -10 | awk -F',' '{
        ts=$1; op=$2; dur=$3;
        # Convert timestamp to human readable
        cmd = sprintf("date -r %d +\"%%Y-%%m-%%d %%H:%%M:%%S\"", ts/1000);
        cmd | getline time;
        close(cmd);
        printf("    %s | %-30s | %5d ms\n", time, op, dur);
    }'

    echo ""
    echo "  Operations by type:"
    awk -F',' '{print $2}' /tmp/ui_performance.log | sort | uniq -c | sort -rn | head -10 | awk '{
        printf("    %-30s | %5d times\n", $2, $1);
    }'

    echo ""
    echo "To view full log: cat /tmp/ui_performance.log"
else
    echo "  No performance log found yet. Log will be created when slow operations are detected."
    echo ""
    echo "[4/4] Monitoring swaglog for UI performance issues..."
    echo "  (Press Ctrl+C to stop)"
    ssh "comma@$DEVICE_IP" "tail -f /data/community/swaglog.log | grep --line-buffered 'bp.ui.perf\\|bp.ui.state\\|watchdog'"
fi
