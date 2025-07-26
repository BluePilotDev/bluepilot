#!/bin/bash
###############################################################################
# boot-logo.sh - Standalone Boot Logo Management Script
#
# Version: 1.0.0
# Last Modified: $(date +%Y-%m-%d)
#
# This standalone script manages boot image updates/restoration
# for CommaAI devices, specifically for BluePilot customizations.
###############################################################################

readonly SCRIPT_VERSION="1.0.0"
readonly SCRIPT_NAME="Boot Logo Manager"

###############################################################################
# Runtime Configuration
###############################################################################
HEADLESS_MODE=false
FORCE_MODE=false
QUIET_MODE=false
LOG_FILE=""

###############################################################################
# Color Constants and Helper Functions
###############################################################################
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

# Helper functions for colored output
print_success() {
    local message="$1"
    if [ "$QUIET_MODE" != "true" ]; then
        echo -e "${GREEN}$message${NC}"
    fi
    [ -n "$LOG_FILE" ] && echo "$(date '+%Y-%m-%d %H:%M:%S') [SUCCESS] $message" >> "$LOG_FILE"
}

print_error() {
    local message="$1"
    if [ "$QUIET_MODE" != "true" ]; then
        echo -e "${RED}$message${NC}" >&2
    fi
    [ -n "$LOG_FILE" ] && echo "$(date '+%Y-%m-%d %H:%M:%S') [ERROR] $message" >> "$LOG_FILE"
}

print_warning() {
    local message="$1"
    if [ "$QUIET_MODE" != "true" ]; then
        echo -e "${YELLOW}$message${NC}"
    fi
    [ -n "$LOG_FILE" ] && echo "$(date '+%Y-%m-%d %H:%M:%S') [WARNING] $message" >> "$LOG_FILE"
}

print_info() {
    local message="$1"
    if [ "$QUIET_MODE" != "true" ]; then
        echo -e "${BLUE}$message${NC}"
    fi
    [ -n "$LOG_FILE" ] && echo "$(date '+%Y-%m-%d %H:%M:%S') [INFO] $message" >> "$LOG_FILE"
}

# Convenient prompt-pause function (only in interactive mode)
pause_for_user() {
    if [ "$HEADLESS_MODE" = "true" ]; then
        return 0
    fi
    read -p "Press enter to continue..."
}

###############################################################################
# Path Constants
###############################################################################
readonly BOOT_IMG="/usr/comma/bg.jpg"
readonly BLUEPILOT_BOOT_IMG="/data/openpilot/selfdrive/assets/img_bluepilot_boot.jpg"
readonly BOOT_IMG_BKP="${BOOT_IMG}.backup"

###############################################################################
# Partition Management Functions
###############################################################################
mount_partition_rw() {
    local partition="$1"
    print_info "Mounting $partition as read-write..."
    sudo mount -o remount,rw "$partition"
    if [ $? -eq 0 ]; then
        print_success "Successfully mounted $partition as read-write"
    else
        print_error "Failed to mount $partition as read-write"
        return 1
    fi
}

mount_partition_ro() {
    local partition="$1"
    print_info "Mounting $partition as read-only..."
    sudo mount -o remount,ro "$partition"
    if [ $? -eq 0 ]; then
        print_success "Successfully mounted $partition as read-only"
    else
        print_warning "Failed to mount $partition as read-only"
    fi
}

###############################################################################
# Backup Management Functions
###############################################################################
clean_backups() {
    print_info "Cleaning up backup files..."

    local files_removed=0
    if [ -f "$BOOT_IMG_BKP" ]; then
        if sudo rm -f "$BOOT_IMG_BKP"; then
            print_success "Removed boot image backup: $BOOT_IMG_BKP"
            files_removed=$((files_removed + 1))
        else
            print_error "Failed to remove boot image backup"
            return 1
        fi
    fi

    if [ $files_removed -eq 0 ]; then
        print_info "No backup files found to remove."
    else
        print_success "Cleaned up $files_removed backup file(s)."
    fi

    [ "$HEADLESS_MODE" != "true" ] && pause_for_user
    return 0
}

update_boot_image() {
    print_info "Updating boot image..."
    mount_partition_rw "/"

    # Ensure the original file exists before proceeding
    if [ ! -f "$BOOT_IMG" ]; then
        print_error "Boot image ($BOOT_IMG) does not exist. Aborting update."
        [ "$HEADLESS_MODE" != "true" ] && pause_for_user
        return 1
    fi

    # Create backup if it does not already exist
    if [ ! -f "$BOOT_IMG_BKP" ]; then
        sudo cp "$BOOT_IMG" "$BOOT_IMG_BKP"
        print_success "Backup created for boot image at $BOOT_IMG_BKP"
    else
        print_info "Backup for boot image already exists at $BOOT_IMG_BKP"
    fi

    # Ensure the BluePilot image exists
    if [ ! -f "$BLUEPILOT_BOOT_IMG" ]; then
        print_error "BluePilot boot image ($BLUEPILOT_BOOT_IMG) not found."
        [ "$HEADLESS_MODE" != "true" ] && pause_for_user
        return 1
    fi

    # Overwrite the original file with the BluePilot image
    sudo cp "$BLUEPILOT_BOOT_IMG" "$BOOT_IMG"
    print_success "Boot image updated with BluePilot file."
    mount_partition_ro "/"
    [ "$HEADLESS_MODE" != "true" ] && pause_for_user
}

restore_boot_image() {
    print_info "Restoring boot image from backup..."
    mount_partition_rw "/"

    # Check if backup exists before attempting restoration
    if [ ! -f "$BOOT_IMG_BKP" ]; then
        print_error "Backup for boot image not found at $BOOT_IMG_BKP"
        [ "$HEADLESS_MODE" != "true" ] && pause_for_user
        return 1
    fi

    # Restore backup to the original file location
    sudo cp "$BOOT_IMG_BKP" "$BOOT_IMG"

    # Remove the backup
    sudo rm -f "$BOOT_IMG_BKP"

    print_success "Boot image restored from backup."
    mount_partition_ro "/"
    [ "$HEADLESS_MODE" != "true" ] && pause_for_user
}

check_custom_status() {
    if [ -f "$BOOT_IMG_BKP" ]; then
        echo -e "${GREEN}Custom BluePilot image is currently active${NC}"
        return 0
    else
        echo "Default image is currently active"
        return 1
    fi
}

###############################################################################
# Menu System
###############################################################################
show_help() {
    cat <<EOL
$SCRIPT_NAME (v$SCRIPT_VERSION)
====================================================

Usage: ./boot-logo.sh [OPTIONS]

Main Options:
  --update          Update boot image with BluePilot image
  --restore         Restore original boot image
  --status          Check current status (custom vs default)
  --help            Show this help message

Headless/Startup Options:
  --headless        Run in non-interactive mode (no prompts)
  --force           Skip confirmation prompts
  --quiet           Suppress output (except errors)
  --log <file>      Log operations to specified file

Exit Codes:
  0                 Success
  1                 General error
  2                 File not found
  3                 Permission denied
  4                 Prerequisites failed

Examples:
  # Interactive mode
  ./boot-logo.sh

  # Headless startup script usage
  ./boot-logo.sh --update --headless --quiet --log /var/log/boot-logo.log

  # Force update without prompts
  ./boot-logo.sh --update --force

  # Check status for scripts
  if ./boot-logo.sh --status --quiet; then
    echo "Custom image active"
  fi

Status Detection:
  The script uses backup file existence to determine status:
  - Backup file exists = Custom image active
  - No backup file = Default image active

File Locations:
  Boot Image:     $BOOT_IMG
  BluePilot Boot: $BLUEPILOT_BOOT_IMG

Note: This script requires sudo permissions to modify system files.
EOL
}

display_main_menu() {
    clear
    echo "┌───────────────────────────────────────────────────┐"
    echo "│              $SCRIPT_NAME (v$SCRIPT_VERSION)              │"
    echo "├───────────────────────────────────────────────────┘"
    echo "│"
    echo "│ Current Status:"
    echo "│ $(check_custom_status && echo "├─ Custom BluePilot image active" || echo "├─ Default image active")"
    echo "│"

    # Check if files exist
    echo "│ File Status:"
    [ -f "$BOOT_IMG" ] && echo -e "│ ├─ Boot image: ${GREEN}Found${NC}" || echo -e "│ ├─ Boot image: ${RED}Missing${NC}"
    [ -f "$BLUEPILOT_BOOT_IMG" ] && echo -e "│ ├─ BluePilot boot: ${GREEN}Found${NC}" || echo -e "│ ├─ BluePilot boot: ${RED}Missing${NC}"
    [ -f "$BOOT_IMG_BKP" ] && echo -e "│ └─ Boot backup: ${GREEN}Found${NC}" || echo "│ └─ Boot backup: Not found"
    echo "│"
    echo "├───────────────────────────────────────────────────"
    echo "│"
    echo "│ Available Actions:"
    if check_custom_status >/dev/null 2>&1; then
        echo "│ 1. Restore Original Image"
    else
        echo "│ 1. Apply BluePilot Image"
    fi
    echo "│ 2. Force Update to BluePilot Image"
    echo "│ 3. Force Restore to Original Image"
    echo "│ 4. Show File Status Details"
    echo "│ H. Show Help"
    echo "│ Q. Exit"
    echo "│"
    echo "└───────────────────────────────────────────────────"
}

handle_menu_input() {
    read -p "Enter your choice: " choice
    case $choice in
        1)
            if check_custom_status >/dev/null 2>&1; then
                echo
                print_warning "This will restore the original boot image."
                if [ "$FORCE_MODE" = "true" ]; then
                    restore_boot_image
                else
                    read -p "Are you sure? (y/N): " confirm
                    if [[ "$confirm" =~ ^[Yy]$ ]]; then
                        restore_boot_image
                    else
                        print_info "Operation cancelled."
                        pause_for_user
                    fi
                fi
            else
                echo
                print_info "This will apply BluePilot custom boot image."
                if [ "$FORCE_MODE" = "true" ]; then
                    update_boot_image
                else
                    read -p "Continue? (Y/n): " confirm
                    if [[ "$confirm" =~ ^[Nn]$ ]]; then
                        print_info "Operation cancelled."
                        pause_for_user
                    else
                        update_boot_image
                    fi
                fi
            fi
            ;;
        2)
            echo
            print_warning "This will force update to BluePilot image (creates backup if needed)."
            if [ "$FORCE_MODE" = "true" ]; then
                update_boot_image
            else
                read -p "Continue? (y/N): " confirm
                if [[ "$confirm" =~ ^[Yy]$ ]]; then
                    update_boot_image
                else
                    print_info "Operation cancelled."
                    pause_for_user
                fi
            fi
            ;;
        3)
            echo
            print_warning "This will force restore to original image."
            if [ "$FORCE_MODE" = "true" ]; then
                restore_boot_image
            else
                read -p "Continue? (y/N): " confirm
                if [[ "$confirm" =~ ^[Yy]$ ]]; then
                    restore_boot_image
                else
                    print_info "Operation cancelled."
                    pause_for_user
                fi
            fi
            ;;
        4)
            show_file_details
            ;;
        [hH])
            show_help
            pause_for_user
            ;;
        [qQ])
            print_info "Exiting..."
            exit 0
            ;;
        *)
            print_error "Invalid choice."
            pause_for_user
            ;;
    esac
}

show_file_details() {
    clear
    echo "┌───────────────────────────────────────────────────┐"
    echo "│                 File Status Details               │"
    echo "├───────────────────────────────────────────────────┘"
    echo "│"

    # Show detailed file information
    for file in "$BOOT_IMG" "$BLUEPILOT_BOOT_IMG" "$BOOT_IMG_BKP"; do
        local basename=$(basename "$file")
        if [ -f "$file" ]; then
            local size=$(ls -lh "$file" | awk '{print $5}')
            local date=$(ls -l "$file" | awk '{print $6, $7, $8}')
            echo -e "│ ${GREEN}✓${NC} $basename"
            echo "│   Size: $size, Modified: $date"
            echo "│   Path: $file"
        else
            echo -e "│ ${RED}✗${NC} $basename"
            echo "│   File not found: $file"
        fi
        echo "│"
    done

    echo "└───────────────────────────────────────────────────"
    pause_for_user
}

###############################################################################
# Command Line Argument Parsing
###############################################################################
parse_arguments() {
    local action=""

    while [ $# -gt 0 ]; do
        case "$1" in
            --update)
                action="update"
                shift
                ;;
            --restore)
                action="restore"
                shift
                ;;
            --status)
                action="status"
                shift
                ;;
            --headless)
                HEADLESS_MODE=true
                shift
                ;;
            --force)
                FORCE_MODE=true
                shift
                ;;
            --quiet)
                QUIET_MODE=true
                shift
                ;;
            --log)
                if [ -n "$2" ]; then
                    LOG_FILE="$2"
                    shift 2
                else
                    print_error "Error: --log requires a file path"
                    exit 1
                fi
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            "")
                # Empty argument, skip
                shift
                ;;
            *)
                print_error "Unknown option: $1"
                echo
                show_help
                exit 1
                ;;
        esac
    done

    # Execute the action if one was specified
    if [ -n "$action" ]; then
        case "$action" in
            update)
                if update_boot_image; then
                    exit 0
                else
                    exit 1
                fi
                ;;
            restore)
                if restore_boot_image; then
                    exit 0
                else
                    exit 1
                fi
                ;;
            status)
                if check_custom_status >/dev/null 2>&1; then
                    [ "$QUIET_MODE" != "true" ] && echo "Custom BluePilot image is active"
                    exit 0
                else
                    [ "$QUIET_MODE" != "true" ] && echo "Default image is active"
                    exit 1
                fi
                ;;
        esac
    fi
}

###############################################################################
# Prerequisites Check
###############################################################################
check_prerequisites() {
    local errors=0

    # Check if running on expected system
    if [ ! -d "/data/openpilot" ]; then
        print_error "This script is designed for CommaAI devices with OpenPilot."
        errors=$((errors + 1))
    fi

    # Check sudo access
    if ! sudo -n true 2>/dev/null; then
        print_warning "This script requires sudo access to modify system files."
        print_info "You may be prompted for password during execution."
    fi

    # Check if we can write to temp location
    if [ ! -w "/tmp" ]; then
        print_error "Cannot write to /tmp directory."
        errors=$((errors + 1))
    fi

    return $errors
}

###############################################################################
# Main Execution
###############################################################################
main() {
    # Parse command line arguments
    parse_arguments "$@"

    # If we're in headless mode but no action was specified, that's an error
    if [ "$HEADLESS_MODE" = "true" ]; then
        print_error "Headless mode requires an action (--update, --restore, or --status)"
        exit 1
    fi

    # Check prerequisites
    if ! check_prerequisites; then
        print_error "Prerequisites check failed. Exiting."
        exit 4
    fi

    # Run interactive menu (only if not headless)
    while true; do
        display_main_menu
        handle_menu_input
    done
}

# Run the script
main "$@"
