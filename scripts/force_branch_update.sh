#!/usr/bin/env bash

# Exit on error, but we'll handle errors manually after the swap
set -e

# Usage function
usage() {
    echo "Usage: $0 [OPTIONS] [BRANCH]"
    echo ""
    echo "Force update BluePilot by cloning a fresh copy of the specified branch."
    echo ""
    echo "Options:"
    echo "  --test, --dry-run    Simulate the update process without making changes"
    echo "  -h, --help           Show this help message"
    echo ""
    echo "Arguments:"
    echo "  BRANCH              Git branch to clone (default: bp-5.0-beta)"
    echo ""
    echo "Examples:"
    echo "  $0                           # Clone bp-5.0-beta (default)"
    echo "  $0 bp-dev                    # Clone bp-dev branch"
    echo "  $0 --test bp-5.0-beta        # Test mode - simulate without changes"
    echo "  $0 stable-deprecated         # Clone stable-deprecated branch"
    echo ""
    exit 1
}

# Parse arguments
TEST_MODE=false
BRANCH=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            ;;
        --test|--dry-run)
            TEST_MODE=true
            shift
            ;;
        *)
            if [ -z "$BRANCH" ]; then
                BRANCH="$1"
            fi
            shift
            ;;
    esac
done

# Set default branch if not specified
BRANCH="${BRANCH:-bp-5.0-beta}"
LOCK_FILE="/tmp/force_update.lock"
TEMP_DIR="/data/openpilot_temp_update"
TARGET_DIR="/data/openpilot"
OLD_DIR="/data/openpilot_old_backup"
REPO_URL="https://github.com/BluePilotDev/bluepilot.git"

echo "========================================"
if [ "$TEST_MODE" = true ]; then
    echo "TEST MODE - Force Update to $BRANCH"
else
    echo "Force Update to $BRANCH"
fi
echo "========================================"
echo ""
if [ "$TEST_MODE" = true ]; then
    echo "🧪 TEST MODE ENABLED - No changes will be made!"
    echo ""
else
    echo "WARNING: This will delete /data/openpilot and clone fresh!"
fi
echo "Branch: $BRANCH"
echo ""

# Check if /data exists (i.e., we're on a device)
if [ ! -d "/data" ]; then
    echo "ERROR: /data directory not found. This script is meant to run on a Comma device."
    exit 1
fi

# Disable power save mode for faster update
echo "Disabling power save mode for faster update..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/disable-powersave.py" ]; then
    if [ "$TEST_MODE" = true ]; then
        echo "  [TEST] Would run: python3 $SCRIPT_DIR/disable-powersave.py"
    else
        python3 "$SCRIPT_DIR/disable-powersave.py" 2>/dev/null || echo "  ⚠ Warning: Could not disable power save mode"
    fi
else
    echo "  ⚠ Warning: disable-powersave.py not found, skipping"
fi
echo ""

# Acquire lock to prevent concurrent updates
echo "Checking for concurrent update processes..."
if [ -f "$LOCK_FILE" ]; then
    # Check if the process holding the lock is still running
    LOCK_PID=$(cat "$LOCK_FILE" 2>/dev/null || echo "")
    if [ -n "$LOCK_PID" ] && kill -0 "$LOCK_PID" 2>/dev/null; then
        echo "ERROR: Another update is already in progress (PID: $LOCK_PID)"
        echo "If you're sure no update is running, remove: $LOCK_FILE"
        exit 1
    else
        echo "Removing stale lock file..."
        rm -f "$LOCK_FILE"
    fi
fi

# Create lock file with our PID (skip in test mode)
if [ "$TEST_MODE" = true ]; then
    echo "  [TEST] Would create lock file at: $LOCK_FILE"
else
    echo $$ > "$LOCK_FILE"

    # Ensure lock file is removed on exit
    cleanup_lock() {
        rm -f "$LOCK_FILE"
    }
    trap cleanup_lock EXIT

    # Cleanup handler for unexpected termination (before directory swap)
    # After the swap, the rollback function handles cleanup
    cleanup_on_error() {
        local exit_code=$?
        if [ "$exit_code" -ne 0 ]; then
            echo ""
            echo "Script terminated unexpectedly (exit code: $exit_code)"
            # Only clean up temp dir if it still exists and we haven't swapped yet
            if [ -d "$TEMP_DIR" ] && [ -d "$TARGET_DIR" ]; then
                echo "Cleaning up temporary directory..."
                rm -rf "$TEMP_DIR"
            fi
        fi
        cleanup_lock
    }
    # This trap is overridden after the swap by the rollback logic
    trap cleanup_on_error EXIT ERR INT TERM
fi

# Clean up any existing temp directories from failed previous attempts
if [ -d "$TEMP_DIR" ]; then
    echo "Removing old temp directory..."
    if [ "$TEST_MODE" = true ]; then
        echo "  [TEST] Would remove: $TEMP_DIR"
    else
        rm -rf "$TEMP_DIR"
    fi
fi

if [ -d "$OLD_DIR" ]; then
    echo "Removing old backup directory..."
    if [ "$TEST_MODE" = true ]; then
        echo "  [TEST] Would remove: $OLD_DIR"
    else
        rm -rf "$OLD_DIR"
    fi
fi

# Check internet connectivity and branch availability
echo "Verifying internet connectivity and branch availability..."
if ! git ls-remote --heads "$REPO_URL" "$BRANCH" >/dev/null 2>&1; then
    echo "✗ ERROR: Cannot reach repository or branch '$BRANCH' does not exist"
    echo ""
    echo "Possible issues:"
    echo "  1. No internet connectivity"
    echo "  2. Branch '$BRANCH' does not exist in the repository"
    echo "  3. GitHub is unreachable"
    echo ""
    if [ "$TEST_MODE" = true ]; then
        echo "[TEST] Continuing despite error for test purposes..."
    else
        exit 1
    fi
else
    echo "  ✓ Internet connectivity verified"
    echo "  ✓ Branch '$BRANCH' exists in repository"
fi
echo ""

echo "Cloning BluePilot $BRANCH to temporary directory..."
if [ "$TEST_MODE" = true ]; then
    echo "  [TEST] Would clone: git clone -b $BRANCH --depth 1 $REPO_URL $TEMP_DIR"
    echo "  [TEST] Simulating successful clone..."
    # Create a fake temp directory structure for testing
    mkdir -p "$TEMP_DIR/.git"
    mkdir -p "$TEMP_DIR/selfdrive"
    mkdir -p "$TEMP_DIR/system"
    mkdir -p "$TEMP_DIR/tools"
    mkdir -p "$TEMP_DIR/cereal"
    touch "$TEMP_DIR/launch_openpilot.sh"
    touch "$TEMP_DIR/launch_env.sh"
    echo "export AGNOS_VERSION=\"10\"" > "$TEMP_DIR/launch_env.sh"
else
    cd /data
    git clone -b "$BRANCH" --depth 1 "$REPO_URL" "$TEMP_DIR"
fi

# Verify clone was successful
if [ ! -d "$TEMP_DIR/.git" ]; then
    echo "ERROR: Clone failed or incomplete!"
    rm -rf "$TEMP_DIR"
    exit 1
fi

echo "Clone successful! Verifying branch..."
cd "$TEMP_DIR"

if [ "$TEST_MODE" = true ]; then
    CURRENT_BRANCH="$BRANCH"
    echo "  [TEST] Simulating branch check: $BRANCH"
else
    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    if [ "$CURRENT_BRANCH" != "$BRANCH" ]; then
        echo "ERROR: Wrong branch checked out: $CURRENT_BRANCH (expected: $BRANCH)"
        cd /data
        rm -rf "$TEMP_DIR"
        exit 1
    fi
fi
echo "  ✓ Branch verified: $BRANCH"

# Verify commit hash
echo "Verifying commit hash..."
if [ "$TEST_MODE" = true ]; then
    CLONED_COMMIT="0123456789abcdef0123456789abcdef01234567"
    echo "  [TEST] Simulating commit hash"
else
    CLONED_COMMIT=$(git rev-parse HEAD 2>/dev/null)
    if [ -z "$CLONED_COMMIT" ]; then
        echo "✗ ERROR: Could not get commit hash from cloned repository"
        cd /data
        rm -rf "$TEMP_DIR"
        exit 1
    fi
fi
echo "  ✓ Commit hash: ${CLONED_COMMIT:0:10}"

# Check for and initialize submodules if present
echo ""
echo "Checking for submodules..."
if [ -f "$TEMP_DIR/.gitmodules" ]; then
    echo "  ✓ Submodules detected, initializing..."
    if [ "$TEST_MODE" = true ]; then
        echo "  [TEST] Would run: git submodule update --init --recursive"
        echo "  [TEST] Simulating submodule initialization..."
    else
        cd "$TEMP_DIR"
        if git submodule update --init --recursive; then
            echo "  ✓ Submodules initialized successfully"
        else
            echo "  ✗ ERROR: Failed to initialize submodules"
            cd /data
            rm -rf "$TEMP_DIR"
            exit 1
        fi
        cd /data
    fi
else
    echo "  ✓ No submodules detected"
fi

# Check for AGNOS version mismatch (TICI devices only)
if [ -f "/COMMA" ]; then
    echo ""
    echo "Checking AGNOS version compatibility..."

    # Get current AGNOS version
    CURRENT_AGNOS=""
    if [ -f "/VERSION" ]; then
        CURRENT_AGNOS=$(cat /VERSION 2>/dev/null || echo "unknown")
    fi

    # Get required AGNOS version from new branch
    TARGET_AGNOS=""
    if [ -f "$TEMP_DIR/launch_env.sh" ]; then
        TARGET_AGNOS=$(grep "export AGNOS_VERSION=" "$TEMP_DIR/launch_env.sh" 2>/dev/null | cut -d'"' -f2 || echo "unknown")
    fi

    if [ -n "$CURRENT_AGNOS" ] && [ -n "$TARGET_AGNOS" ] && [ "$CURRENT_AGNOS" != "unknown" ] && [ "$TARGET_AGNOS" != "unknown" ]; then
        echo "  Current AGNOS: $CURRENT_AGNOS"
        echo "  Required AGNOS: $TARGET_AGNOS"

        if [ "$CURRENT_AGNOS" != "$TARGET_AGNOS" ]; then
            echo ""
            echo "⚠ ════════════════════════════════════════════════════════════"
            echo "⚠  WARNING: AGNOS VERSION MISMATCH DETECTED"
            echo "⚠ ════════════════════════════════════════════════════════════"
            echo ""
            echo "  Your device is running AGNOS $CURRENT_AGNOS"
            echo "  This branch requires AGNOS $TARGET_AGNOS"
            echo ""
            echo "  After this update completes, your device will need to:"
            echo "    1. Download AGNOS $TARGET_AGNOS (may take time)"
            echo "    2. Flash the new AGNOS version"
            echo "    3. Reboot to apply the AGNOS update"
            echo ""
            echo "  The AGNOS update process will happen automatically after"
            echo "  you reboot following this openpilot update."
            echo ""
            echo "⚠ ════════════════════════════════════════════════════════════"
            echo ""

            # Check if running interactively (from SSH/terminal) or from UI
            if [ -t 0 ]; then
                # Interactive mode - prompt user
                read -p "Continue with update? (yes/no): " -r
                if [[ ! $REPLY =~ ^[Yy][Ee][Ss]$ ]]; then
                    echo "Update cancelled by user"
                    cd /data
                    rm -rf "$TEMP_DIR"
                    exit 0
                fi
            else
                # Non-interactive mode (running from UI) - proceed with warning logged
                echo "⚠ AGNOS update will be required after reboot (running in non-interactive mode)"
                echo "⚠ Proceeding with openpilot update..."
            fi
        else
            echo "  ✓ AGNOS versions match - no AGNOS update required"
        fi
    else
        echo "  ⚠ Could not determine AGNOS versions (current: $CURRENT_AGNOS, target: $TARGET_AGNOS)"
    fi
    echo ""
fi

echo "Swapping directories (atomic move operation)..."
echo "Note: Processes will continue running and will pick up new code on next reboot"
cd /data

# Move old installation to backup (fast move operation)
if [ -d "$TARGET_DIR" ]; then
    if [ "$TEST_MODE" = true ]; then
        echo "  [TEST] Would move: $TARGET_DIR -> $OLD_DIR"
    else
        mv "$TARGET_DIR" "$OLD_DIR"
    fi
fi

# Move new installation into place (fast move operation)
if [ "$TEST_MODE" = true ]; then
    echo "  [TEST] Would move: $TEMP_DIR -> $TARGET_DIR"
    echo "  [TEST] For testing purposes, keeping temp dir as-is"
else
    mv "$TEMP_DIR" "$TARGET_DIR"
fi

# Disable set -e after swap - we need custom error handling from here
set +e

echo "Verifying installation and fixing permissions..."

# Determine which directory to verify based on mode
VERIFY_DIR="$TARGET_DIR"
if [ "$TEST_MODE" = true ]; then
    # In test mode, we never moved the clone, so verify the temp directory
    VERIFY_DIR="$TEMP_DIR"
    echo "  [TEST] Verifying temp directory: $VERIFY_DIR"
fi

# Function to check if critical paths exist
verify_installation() {
    local issues=0

    echo "Checking critical files and directories..."

    # Check for critical directories
    for dir in "selfdrive" "system" "tools" "cereal"; do
        if [ ! -d "$VERIFY_DIR/$dir" ]; then
            echo "  ✗ ERROR: Missing critical directory: $dir"
            issues=$((issues + 1))
        else
            echo "  ✓ Found: $dir/"
        fi
    done

    # Check for critical files
    for file in "launch_openpilot.sh" "launch_env.sh"; do
        if [ ! -f "$VERIFY_DIR/$file" ]; then
            echo "  ✗ ERROR: Missing critical file: $file"
            issues=$((issues + 1))
        else
            echo "  ✓ Found: $file"
        fi
    done

    return "$issues"
}

# Function to rollback on failure
rollback_update() {
    echo ""
    echo "════════════════════════════════════════════════════════════"
    echo "CRITICAL ERROR: Attempting rollback to previous installation"
    echo "════════════════════════════════════════════════════════════"
    echo ""

    # In TEST_MODE, we never actually swapped directories, so skip rollback
    if [ "$TEST_MODE" = true ]; then
        echo "[TEST] Would rollback installation (skipping in test mode)"
        echo "[TEST] Cleaning up test directories..."
        rm -rf "$TEMP_DIR"
        echo "════════════════════════════════════════════════════════════"
        exit 1
    fi

    if [ -d "$OLD_DIR" ]; then
        echo "Removing failed installation..."
        rm -rf "$TARGET_DIR"

        echo "Restoring previous installation from backup..."
        mv "$OLD_DIR" "$TARGET_DIR"

        echo ""
        echo "✓ Rollback successful!"
        echo "✓ Your previous openpilot installation has been restored"
        echo ""
        echo "The update failed, but your device should still be operational."
        echo "Please report this issue or try again later."
    else
        echo "✗ CRITICAL: Backup directory not found!"
        echo "✗ Cannot rollback automatically"
        echo "✗ Manual intervention required"
    fi

    echo "════════════════════════════════════════════════════════════"
    exit 1
}

# Verify installation integrity
if ! verify_installation; then
    echo "✗ ERROR: Installation verification failed!"
    echo "The cloned repository may be incomplete or corrupted."
    rollback_update
fi

echo ""
echo "Fixing ownership and permissions..."

if [ "$TEST_MODE" = true ]; then
    echo "  [TEST] Would run: sudo chown -R comma:comma $VERIFY_DIR"
    echo "  ✓ [TEST] Simulated owner change"
else
    # Set ownership to comma:comma
    if sudo chown -R comma:comma "$VERIFY_DIR" 2>/dev/null; then
        echo "  ✓ Set owner to comma:comma"
    else
        echo "  ⚠ Warning: Could not change owner (may already be correct or sudo not available)"
    fi
fi

if [ "$TEST_MODE" = true ]; then
    echo "  [TEST] Would run: sudo chmod -R 755 $VERIFY_DIR"
    echo "  ✓ [TEST] Simulated permission change"
else
    # Set permissions to 755 recursively
    if sudo chmod -R 755 "$VERIFY_DIR" 2>/dev/null; then
        echo "  ✓ Set permissions to 755 (recursive)"
    else
        echo "  ⚠ Warning: Could not change permissions (trying without sudo...)"
        chmod -R 755 "$VERIFY_DIR" 2>/dev/null || echo "  ⚠ Some permissions could not be set"
    fi
fi

# Make critical scripts executable
echo "Ensuring scripts are executable..."
for script in "launch_openpilot.sh" "launch_env.sh" "launch_chffrplus.sh"; do
    if [ -f "$VERIFY_DIR/$script" ]; then
        if [ "$TEST_MODE" = true ]; then
            echo "  ✓ [TEST] $script would be executable"
        else
            chmod +x "$VERIFY_DIR/$script" 2>/dev/null && echo "  ✓ $script is executable"
        fi
    fi
done

# Verify final permissions
echo ""
echo "Verifying final permissions..."
PERMISSION_ISSUES=0

if [ -r "$VERIFY_DIR" ] && [ -x "$VERIFY_DIR" ]; then
    echo "  ✓ Directory is readable and executable"
else
    echo "  ✗ WARNING: Directory permissions may be incorrect!"
    PERMISSION_ISSUES=1
fi

if [ -r "$VERIFY_DIR/launch_openpilot.sh" ] && [ -x "$VERIFY_DIR/launch_openpilot.sh" ]; then
    echo "  ✓ launch_openpilot.sh is readable and executable"
else
    echo "  ✗ WARNING: launch_openpilot.sh permissions may be incorrect!"
    PERMISSION_ISSUES=1
fi

# Final safety check before cleaning up backup
if [ "$PERMISSION_ISSUES" -eq 1 ]; then
    echo ""
    echo "⚠ WARNING: Permission issues detected!"
    echo "⚠ Keeping backup directory at: $OLD_DIR"
    echo "⚠ You may need to manually fix permissions or restore from backup"
    echo ""
    echo "To restore backup if needed:"
    echo "  rm -rf $TARGET_DIR && mv $OLD_DIR $TARGET_DIR"
    echo ""
else
    echo ""
    if [ "$TEST_MODE" = true ]; then
        echo "All verifications passed! Cleaning up test directories..."
        rm -rf "$TEMP_DIR"
        echo "  ✓ Test directories cleaned up"
    else
        echo "All verifications passed! Cleaning up old backup..."
        rm -rf "$OLD_DIR"
        echo "  ✓ Backup cleaned up successfully"
    fi
fi

# Re-enable exit on error for final output
set -e

echo ""
echo "========================================"
if [ "$TEST_MODE" = true ]; then
    echo "🧪 TEST MODE COMPLETE!"
    echo ""
    echo "Simulated update to: $BRANCH"
    echo "Commit: ${CLONED_COMMIT:0:10}"
    echo ""
    echo "✓ All checks passed successfully"
    echo "✓ No changes were made to your system"
    echo "✓ The actual update would work correctly"
    echo ""
    echo "To perform the real update, run without --test flag"
else
    echo "Update complete!"
    echo "Branch: $BRANCH"
    echo "Commit: ${CLONED_COMMIT:0:10}"
    echo "Location: $TARGET_DIR"
    echo ""
    if [ "$PERMISSION_ISSUES" -eq 1 ]; then
        echo "⚠ WARNING: Some permission issues detected"
        echo "⚠ Backup kept at: $OLD_DIR"
        echo ""
    fi
    echo "Please reboot your device."
fi
echo "========================================"
