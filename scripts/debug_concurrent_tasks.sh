#!/usr/bin/env bash
# Script to analyze concurrent task logs from /data/concurrent_tasks.log
# Similar to debug_ui_performance.sh but for QtConcurrent task tracking

set -e

LOG_FILE="/data/concurrent_tasks.log"
LOCAL_LOG="./concurrent_tasks_$(date +%Y%m%d_%H%M%S).log"

# Download log from device
echo "=== Downloading concurrent tasks log from device ==="
if scp "comma@10.0.1.125:${LOG_FILE}" "${LOCAL_LOG}" 2>/dev/null; then
    echo "Downloaded to ${LOCAL_LOG}"
else
    echo "Failed to download log. Make sure device is accessible at comma@10.0.1.125"
    echo "Checking for local log file..."
    if [ -f "${LOG_FILE}" ]; then
        LOCAL_LOG="${LOG_FILE}"
        echo "Using local log: ${LOCAL_LOG}"
    else
        echo "No log file found!"
        exit 1
    fi
fi

if [ ! -s "${LOCAL_LOG}" ]; then
    echo "Log file is empty!"
    exit 1
fi

echo ""
echo "=== Log Format ==="
echo "timestamp_ms,event,task_id,task_name,duration_ms,active_count"
echo ""

# Count total tasks
TOTAL_TASKS=$(grep ",START," "${LOCAL_LOG}" | wc -l | tr -d ' ')
echo "=== Summary ==="
echo "Total tasks started: ${TOTAL_TASKS}"

# Find tasks that never completed (still running or stuck)
echo ""
echo "=== Tasks That Never Completed ==="
awk -F',' '
    /,START,/ { started[$3] = $4; start_time[$3] = $1 }
    /,END,/ { delete started[$3]; delete start_time[$3] }
    END {
        if (length(started) == 0) {
            print "All tasks completed successfully!"
        } else {
            for (id in started) {
                print "Task " id ": " started[id] " (started at " start_time[id] ")"
            }
        }
    }
' "${LOCAL_LOG}"

# Show slowest tasks
echo ""
echo "=== Slowest Tasks (Top 10) ==="
grep ",END," "${LOCAL_LOG}" | \
    awk -F',' '{print $5 "ms - " $4}' | \
    sort -rn | \
    head -10

# Show tasks by type with average duration
echo ""
echo "=== Tasks By Type (with avg duration) ==="
grep ",END," "${LOCAL_LOG}" | \
    awk -F',' '{
        task=$4;
        duration=$5;
        count[task]++;
        total[task]+=$5;
    }
    END {
        for (task in count) {
            avg = total[task] / count[task];
            printf "%3d calls | %6.0fms avg | %s\n", count[task], avg, task;
        }
    }' | sort -rn

# Show peak concurrent task count
echo ""
echo "=== Peak Concurrent Tasks ==="
awk -F',' 'BEGIN {max=0} {if($6>max) max=$6} END {print "Peak: " max " concurrent tasks"}' "${LOCAL_LOG}"

# Show timeline of concurrent task count
echo ""
echo "=== Concurrent Task Count Over Time (sampled every 1000 events) ==="
awk -F',' 'NR % 1000 == 0 {print $1 "," $6}' "${LOCAL_LOG}" | \
    head -20 | \
    awk -F',' '{
        # Convert timestamp to seconds
        ts = int($1 / 1000);
        # Create simple bar chart
        bar = "";
        for (i = 0; i < $2; i++) bar = bar "#";
        printf "%10d: %2d %s\n", ts, $2, bar;
    }'

# Check for thread pool saturation (>5 concurrent tasks for extended period)
echo ""
echo "=== Thread Pool Saturation Warnings (>5 concurrent) ==="
awk -F',' '$6 > 5 {print $1 "," $6 "," $4}' "${LOCAL_LOG}" | \
    head -20 | \
    awk -F',' '{print "Time: " $1 " | Active: " $2 " | Latest: " $3}'

if [ "$(awk -F',' '$6 > 5' "${LOCAL_LOG}" | wc -l)" -gt 0 ]; then
    echo ""
    echo "⚠️  WARNING: Thread pool saturation detected! This can cause UI freezes."
fi

echo ""
echo "=== Analysis Complete ==="
echo "Full log saved at: ${LOCAL_LOG}"
echo ""
echo "To view raw log:"
echo "  cat ${LOCAL_LOG}"
echo ""
echo "To clear log on device:"
echo "  ssh comma@10.0.1.125 'rm ${LOG_FILE}'"
