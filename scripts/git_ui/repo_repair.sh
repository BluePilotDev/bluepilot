#!/bin/bash

# /data/repo_repair.sh
set -e

OPENPILOT_DIR="/data/openpilot"
REPO_INFO_FILE="/data/openpilot_repo_info"
TIMEOUT_CLONE=300
TIMEOUT_SUBMODULE=300
TIMEOUT_SCONS=600

log_info() {
    echo "[INFO] $1"
}

log_error() {
    echo "[ERROR] $1" >&2
}

save_repo_info() {
    log_info "Capturing current repository information..."

    if [ -d "$OPENPILOT_DIR/.git" ]; then
        cd "$OPENPILOT_DIR"
        REMOTE_URL=$(git remote get-url origin 2>&1)
        CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>&1)
    fi

    # If we can't get info from git, try reading from saved file
    if [ -z "$REMOTE_URL" ] || [ -z "$CURRENT_BRANCH" ]; then
        if [ -f "$REPO_INFO_FILE" ]; then
            source "$REPO_INFO_FILE"
            log_info "Using saved repository information"
        fi
    fi

    # If we still don't have values, use defaults
    REMOTE_URL=${REMOTE_URL:-"git@github.com:ford-op/sp-dev-c3.git"}
    CURRENT_BRANCH=${CURRENT_BRANCH:-"bp-dev-2.0"}

    # Save to file
    echo "REMOTE_URL='$REMOTE_URL'" > "$REPO_INFO_FILE"
    echo "CURRENT_BRANCH='$CURRENT_BRANCH'" >> "$REPO_INFO_FILE"

    log_info "Remote URL: $REMOTE_URL"
    log_info "Current branch: $CURRENT_BRANCH"
}

remove_repo() {
    log_info "Removing existing repository..."
    rm -rf "$OPENPILOT_DIR"
}

clone_repo() {
    log_info "Cloning repository..."
    cd /data
    timeout $TIMEOUT_CLONE git clone -v --progress --branch "$CURRENT_BRANCH" "$REMOTE_URL" "openpilot" 2>&1
    if [ $? -ne 0 ]; then
        log_error "Failed to clone repository (timeout: ${TIMEOUT_CLONE}s)"
        return 1
    fi
}

update_submodules() {
    log_info "Updating submodules..."
    cd "$OPENPILOT_DIR"
    timeout $TIMEOUT_SUBMODULE git submodule update --init --recursive --progress 2>&1
    if [ $? -ne 0 ]; then
        log_error "Failed to update submodules (timeout: ${TIMEOUT_SUBMODULE}s)"
        return 1
    fi
}

build_scons() {
    log_info "Building with scons..."
    cd "$OPENPILOT_DIR"
    scons -j$(nproc) 2>&1
    # if [ $? -ne 0 ]; then
    #     log_error "Failed to build with scons (timeout: ${TIMEOUT_SCONS}s)"
    #     return 1
    # fi
}

verify_repo() {
    log_info "Verifying repository..."
    if [ ! -d "$OPENPILOT_DIR/.git" ]; then
        log_error "Git repository not properly initialized"
        return 1
    fi

    cd "$OPENPILOT_DIR"
    git status 2>&1
    if [ $? -ne 0 ]; then
        log_error "Invalid git repository state"
        return 1
    fi
}

cleanup() {
    log_info "Cleaning up..."
    rm -f "/data/repo_repair.sh"
    rm -f "$REPO_INFO_FILE"
}

main() {
    log_info "Starting repository repair process..."

    save_repo_info || exit 1
    remove_repo || exit 1
    clone_repo || exit 1
    update_submodules || exit 1
    build_scons || exit 1
    verify_repo || exit 1

    log_info "Repository repair completed successfully"
    cleanup
}

main
