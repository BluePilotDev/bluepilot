#!/bin/bash
###############################################################################
# build_bp_public.sh - BluePilot Public Build Script
#
# This script builds a BluePilot branch and pushes it to a specified build branch.
# It pulls all build logic from the CommaUtility build process.
#
# Usage: ./build_bp_public.sh <source_branch> <build_branch>
# Example: ./build_bp_public.sh bp-dev bp-2.1
###############################################################################

set -e

###############################################################################
# Color Output Functions
###############################################################################
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

print_success() {
    echo -e "${GREEN}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

print_warning() {
    echo -e "${YELLOW}$1${NC}"
}

print_info() {
    echo -e "${BLUE}$1${NC}"
}

###############################################################################
# Argument Validation
###############################################################################
if [ $# -ne 2 ]; then
    print_error "Error: This script requires exactly 2 arguments."
    echo ""
    echo "Usage: $0 <source_branch> <build_branch>"
    echo ""
    echo "  source_branch  - The branch to build from (e.g., bp-dev)"
    echo "  build_branch   - The name of the branch to push the build to (e.g., bp-2.1)"
    echo ""
    echo "Example: $0 bp-dev bp-2.1"
    exit 1
fi

readonly CLONE_BRANCH="$1"
readonly BUILD_BRANCH="$2"
readonly OS=$(uname)

# Get the current repo directory
readonly SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
readonly REPO_DIR="$SCRIPT_DIR"

# Get the remote URL for the current repo
cd "$REPO_DIR" || exit 1
readonly GIT_REPO_URL=$(git config --get remote.origin.url)

# Determine build directory based on OS
if [ "$OS" = "Darwin" ]; then
    readonly BUILD_DIR="$HOME/Documents/bluepilot-utility/bp-build"
else
    readonly BUILD_DIR="/data/openpilot"
fi

print_info "[+] BluePilot Build Script"
print_info "[+] Source Branch: $CLONE_BRANCH"
print_info "[+] Build Branch: $BUILD_BRANCH"
print_info "[+] Repository: $GIT_REPO_URL"
print_info "[+] Build Directory: $BUILD_DIR"
echo ""

###############################################################################
# Helper Functions
###############################################################################
verify_disk_space() {
    local required_space=$1
    local available=$(df -m "$BUILD_DIR" 2>/dev/null | awk 'NR==2 {print $4}')
    if [ -z "$available" ]; then
        # If BUILD_DIR doesn't exist yet, check the parent directory
        local parent_dir=$(dirname "$BUILD_DIR")
        available=$(df -m "$parent_dir" | awk 'NR==2 {print $4}')
    fi
    if [ "$available" -lt "$required_space" ]; then
        print_error "Insufficient disk space. Need ${required_space}MB, have ${available}MB"
        return 1
    fi
    return 0
}

ensure_directory() {
    local target_dir="$1"
    local current_dir
    current_dir=$(pwd)
    if [ "$current_dir" != "$target_dir" ]; then
        cd "$target_dir" || {
            print_error "Could not change to directory: $target_dir"
            return 1
        }
        return 0
    fi
}

###############################################################################
# Git Environment Setup
###############################################################################
setup_git_env() {
    print_info "[-] Setting up Git environment..."

    # Source the identity file if it exists
    if [ -f "$REPO_DIR/release/identity_ford_op.sh" ]; then
        # shellcheck disable=SC1090
        source "$REPO_DIR/release/identity_ford_op.sh"
        print_success "[-] Git identity configured"
    else
        print_error "[-] identity_ford_op.sh not found at $REPO_DIR/release/identity_ford_op.sh"
        exit 1
    fi

    # Set up SSH key for git operations
    if [ -f /data/gitkey ]; then
        export GIT_SSH_COMMAND="ssh -i /data/gitkey"
        print_success "[-] Using SSH key: /data/gitkey"
    elif [ -f ~/.ssh/github ]; then
        export GIT_SSH_COMMAND="ssh -i ~/.ssh/github"
        print_success "[-] Using SSH key: ~/.ssh/github"
    elif [ -f ~/.ssh/id_rsa ]; then
        export GIT_SSH_COMMAND="ssh -i ~/.ssh/id_rsa"
        print_success "[-] Using SSH key: ~/.ssh/id_rsa"
    else
        print_warning "[-] No specific git key found, using default SSH configuration"
    fi
}

###############################################################################
# Build Functions
###############################################################################
build_openpilot() {
    print_info "[-] Building OpenPilot with SCons..."
    export PYTHONPATH="$BUILD_DIR"

    cd "$BUILD_DIR" || exit 1

    if ! scons -j"$(nproc)"; then
        print_error "[-] Build failed!"
        exit 1
    fi

    print_success "[-] Build completed successfully"
}

create_prebuilt_marker() {
    print_info "[-] Creating prebuilt marker..."
    touch "$BUILD_DIR/prebuilt"
}

handle_panda_directory() {
    print_info "[-] Processing panda directory..."

    cd "$BUILD_DIR" || exit 1

    mkdir -p "$BUILD_DIR/panda_tmp/board/obj"
    mkdir -p "$BUILD_DIR/panda_tmp/python"

    # Copy signed binaries
    cp -f "$BUILD_DIR/panda/board/obj/panda.bin.signed" "$BUILD_DIR/panda_tmp/board/obj/panda.bin.signed" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/board/obj/panda_h7.bin.signed" "$BUILD_DIR/panda_tmp/board/obj/panda_h7.bin.signed" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/board/obj/bootstub.panda.bin" "$BUILD_DIR/panda_tmp/board/obj/bootstub.panda.bin" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/board/obj/bootstub.panda_h7.bin" "$BUILD_DIR/panda_tmp/board/obj/bootstub.panda_h7.bin" 2>/dev/null || :

    # Comment out jungle imports
    if [ "$OS" = "Darwin" ]; then
        sed -i '' 's/^from .board.jungle import PandaJungle, PandaJungleDFU # noqa: F401/# &/' panda/__init__.py 2>/dev/null || :
    else
        sed -i 's/^from .board.jungle import PandaJungle, PandaJungleDFU # noqa: F401/# &/' panda/__init__.py 2>/dev/null || :
    fi

    # Copy essential files
    cp -r "$BUILD_DIR/panda/python/." "$BUILD_DIR/panda_tmp/python" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/.gitignore" "$BUILD_DIR/panda_tmp/.gitignore" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/__init__.py" "$BUILD_DIR/panda_tmp/__init__.py" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/mypy.ini" "$BUILD_DIR/panda_tmp/mypy.ini" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/panda.png" "$BUILD_DIR/panda_tmp/panda.png" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/pyproject.toml" "$BUILD_DIR/panda_tmp/pyproject.toml" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/requirements.txt" "$BUILD_DIR/panda_tmp/requirements.txt" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/setup.cfg" "$BUILD_DIR/panda_tmp/setup.cfg" 2>/dev/null || :
    cp -f "$BUILD_DIR/panda/setup.py" "$BUILD_DIR/panda_tmp/setup.py" 2>/dev/null || :

    # Replace original with processed version
    rm -rf "$BUILD_DIR/panda"
    mv "$BUILD_DIR/panda_tmp" "$BUILD_DIR/panda"

    print_success "[-] Panda directory processed"
}

create_opendbc_gitignore() {
    print_info "[-] Creating opendbc .gitignore..."

    cd "$BUILD_DIR" || exit 1

    cat >opendbc_repo/.gitignore <<'EOL'
.mypy_cache/
*.pyc
*.os
*.o
*.tmp
*.dylib
.*.swp
.DS_Store
.sconsign.dblite

opendbc/can/*.so
opendbc/can/*.a
opendbc/can/build/
opendbc/can/obj/
opendbc/can/packer_pyx.cpp
opendbc/can/parser_pyx.cpp
opendbc/can/packer_pyx.html
opendbc/can/parser_pyx.html
EOL
}

update_main_gitignore() {
    print_info "[-] Updating main .gitignore..."

    cd "$BUILD_DIR" || exit 1

    local GITIGNORE_PATH=".gitignore"
    local LINES_TO_REMOVE=(
        "*.dylib"
        "*.so"
        "selfdrive/pandad/pandad"
        "cereal/messaging/bridge"
        "selfdrive/logcatd/logcatd"
        "system/camerad/camerad"
        "selfdrive/modeld/_modeld"
        "selfdrive/modeld/_navmodeld"
        "selfdrive/modeld/_dmonitoringmodeld"
    )

    for LINE in "${LINES_TO_REMOVE[@]}"; do
        if [ "$OS" = "Darwin" ]; then
            sed -i '' "/^${LINE//\//\\/}$/d" "$GITIGNORE_PATH" 2>/dev/null || :
        else
            sed -i "/^${LINE//\//\\/}$/d" "$GITIGNORE_PATH" 2>/dev/null || :
        fi
    done
}

cleanup_directory() {
    local dir="$1"
    shift
    local patterns=("$@")

    if [ -d "$dir" ]; then
        for pattern in "${patterns[@]}"; do
            find "$dir/" -name "$pattern" -exec rm -rf {} + 2>/dev/null || :
        done
    fi
}

cleanup_tinygrad_repo() {
    print_info "[-] Cleaning up tinygrad_repo..."

    cd "$BUILD_DIR" || exit 1

    rm -rf tinygrad_repo/{cache,disassemblers,docs,examples,models,test,weights} 2>/dev/null || :
    rm -rf tinygrad_repo/extra/{accel,assembly,dataset,disk,dist,fastvits,intel,optimization,ptx,rocm,triton} 2>/dev/null || :
    find tinygrad_repo/extra -maxdepth 1 -type f -name '*.py' ! -name 'onnx*.py' ! -name 'thneed*.py' ! -name 'utils*.py' -exec rm -f {} + 2>/dev/null || :
    rm -rf tinygrad_repo/extra/{datasets,gemm} 2>/dev/null || :
    find tinygrad_repo/ -name '*tests*' -exec rm -rf {} + 2>/dev/null || :
    find tinygrad_repo/ -name '.git*' -exec rm -rf {} + 2>/dev/null || :
    find tinygrad_repo/ -name '*.md' -exec rm -f {} + 2>/dev/null || :
    rm -f tinygrad_repo/{.flake8,.pylintrc,.tokeignore,*.sh,*.ini,*.toml,*.py} 2>/dev/null || :
}

# BluePilot: JetLink ships only what the comma runs and its license: jetlink/, LICENSE, pyproject.toml, README.md
# (JetLink's docs/porting.md). The host apps (android, ios, macos, JetlinkKit), docs, scripts and tests stay out
cleanup_jetlink_repo() {
    print_info "[-] Cleaning up jetlink_repo..."

    cd "$BUILD_DIR" || exit 1

    [ -d jetlink_repo ] || return 0
    find jetlink_repo -mindepth 1 -maxdepth 1 ! -name jetlink ! -name LICENSE ! -name pyproject.toml ! -name README.md \
        ! -name '.git' -exec rm -rf {} + 2>/dev/null || :
}
# End BluePilot

cleanup_files() {
    local CURRENT_DIR
    CURRENT_DIR=$(pwd)

    print_info "[-] Cleaning up unnecessary files..."

    ensure_directory "$BUILD_DIR" || return 1

    # Remove compiled artifacts
    find . \( -name '*.a' -o -name '*.o' -o -name '*.os' -o -name '*.pyc' -o -name 'moc_*' -o -name '*.cc' -o -name '__pycache__' -o -name '.DS_Store' \) -exec rm -rf {} + 2>/dev/null || :

    # Remove development directories and files
    rm -rf .sconsign.dblite .venv .devcontainer .idea .mypy_cache .run .vscode 2>/dev/null || :
    rm -f .clang-tidy .env .gitmodules .gitattributes 2>/dev/null || :
    rm -rf teleoprtc_repo teleoprtc release 2>/dev/null || :
    rm -f selfdrive/modeld/models/supercombo.onnx 2>/dev/null || :
    rm -rf selfdrive/ui/replay/ 2>/dev/null || :

    # Remove unnecessary tools
    rm -rf tools/cabana tools/camerastream tools/car_porting tools/latencylogger tools/plotjuggler tools/profiling 2>/dev/null || :
    rm -rf tools/replay tools/rerun tools/scripts tools/serial tools/sim tools/tuning tools/webcam 2>/dev/null || :
    rm -f tools/*.py tools/*.sh tools/*.md 2>/dev/null || :
    rm -f conftest.py SECURITY.md uv.lock 2>/dev/null || :
    rm -f selfdrive/controls/lib/lateral_mpc_lib/.gitignore selfdrive/controls/lib/longitudinal_mpc_lib/.gitignore 2>/dev/null || :

    # Cleanup specific directories
    cleanup_directory "$BUILD_DIR/cereal" "*tests*" "*.md"
    cleanup_directory "$BUILD_DIR/common" "*tests*" "*.md"
    cleanup_directory "$BUILD_DIR/msgq_repo" "*tests*" "*.md" ".git*"
    cleanup_directory "$BUILD_DIR/opendbc_repo" "*tests*" "*.md" ".git*" "LICENSE"
    cleanup_directory "$BUILD_DIR/rednose_repo" "*tests*" "*.md" ".git*" "LICENSE"
    cleanup_directory "$BUILD_DIR/selfdrive" "*.h" "*.md" "*test*"
    cleanup_directory "$BUILD_DIR/system" "*tests*" "*.md"
    cleanup_directory "$BUILD_DIR/third_party" "*Darwin*" "LICENSE" "README.md"

    cleanup_tinygrad_repo
    cleanup_jetlink_repo  # BluePilot

    cd "$CURRENT_DIR" || return 1

    print_success "[-] Cleanup completed"
}

process_submodules() {
    print_info "[-] Converting submodules to regular directories..."

    cd "$BUILD_DIR" || exit 1

    # BluePilot: jetlink_repo (JetLink)
    local submodules=("msgq_repo" "opendbc_repo" "rednose_repo" "panda" "tinygrad_repo" "teleoprtc_repo" "jetlink_repo")

    for sub in "${submodules[@]}"; do
        if [ -d "$sub" ]; then
            print_info "[-]   Processing submodule: $sub"

            # Create a temporary copy preserving all attributes
            local tmp_dir="${sub}_tmp"
            rm -rf "$tmp_dir"
            cp -a "$sub" "$tmp_dir"

            # Remove any .git folder inside the copied submodule
            rm -rf "$tmp_dir/.git"

            # Remove the submodule from git's index
            git rm -rf --cached "$sub" 2>/dev/null || :

            # Remove the original submodule directory
            rm -rf "$sub"

            # Rename the temporary directory to the original name
            mv "$tmp_dir" "$sub"

            # Remove any leftover git metadata
            rm -rf ".git/modules/${sub}"

            # Force add the now-converted directory
            git add "$sub"
        fi
    done

    print_success "[-] Submodules converted to regular directories"
}

prepare_commit_push() {
    print_info "[-] Preparing commit and push..."

    cd "$BUILD_DIR" || exit 1

    if [ ! -f "$BUILD_DIR/common/version.h" ]; then
        print_error "Error: $BUILD_DIR/common/version.h not found."
        exit 1
    fi

    # Generate version information
    local VERSION
    VERSION=$(date '+%Y.%m.%d')
    local TIME_CODE
    TIME_CODE=$(date +"%H%M")
    local GIT_HASH
    GIT_HASH=$(git rev-parse HEAD)
    local DATETIME
    DATETIME=$(date '+%Y-%m-%dT%H:%M:%S')
    local SP_VERSION
    SP_VERSION=$(awk -F\" '{print $2}' "$BUILD_DIR/common/version.h")

    # Update version.h
    echo "#define COMMA_VERSION \"$VERSION-$TIME_CODE\"" >"$BUILD_DIR/common/version.h"

    # Create prebuilt marker
    create_prebuilt_marker

    # Create orphan branch for clean commit
    print_info "[-] Creating orphan branch..."
    git checkout --orphan temp_branch --quiet

    # Stage all files
    print_info "[-] Staging files..."
    git add -f -A >/dev/null 2>&1

    # Commit with detailed message
    print_info "[-] Creating commit..."
    git commit -m "BluePilot Public Build | v$VERSION-$TIME_CODE
version: BluePilot v$SP_VERSION release
date: $DATETIME
master commit: $GIT_HASH
" || {
        print_error "[-] Commit failed"
        exit 1
    }

    # Delete existing build branch if it exists locally
    if git show-ref --verify --quiet "refs/heads/$BUILD_BRANCH"; then
        print_info "[-] Deleting existing local branch: $BUILD_BRANCH"
        git branch -D "$BUILD_BRANCH" || exit 1
    fi

    # Delete existing build branch if it exists remotely
    if git ls-remote --heads "$GIT_REPO_URL" "$BUILD_BRANCH" | grep "$BUILD_BRANCH" >/dev/null 2>&1; then
        print_info "[-] Deleting existing remote branch: $BUILD_BRANCH"
        git push "$GIT_REPO_URL" --delete "$BUILD_BRANCH" || exit 1
    fi

    # Rename temp branch to build branch
    print_info "[-] Renaming branch to: $BUILD_BRANCH"
    git branch -m "$BUILD_BRANCH" >/dev/null 2>&1 || exit 1

    # Push to remote
    print_info "[-] Pushing to remote..."
    git push -f "$GIT_REPO_URL" "$BUILD_BRANCH" || exit 1

    print_success "[-] Successfully pushed to branch: $BUILD_BRANCH"
}

###############################################################################
# Main Build Process
###############################################################################
build_repo_branch() {
    print_info ""
    print_info "=========================================="
    print_info "Starting Build Process"
    print_info "=========================================="
    print_info ""

    # Check available disk space first
    verify_disk_space 5000 || {
        print_error "Insufficient disk space for build operation"
        return 1
    }

    local CURRENT_DIR
    CURRENT_DIR=$(pwd)

    # Clean up any existing build directory
    print_info "[-] Cleaning up previous build directory..."
    rm -rf "$BUILD_DIR"

    # Clone the repository
    print_info "[-] Cloning repository: $GIT_REPO_URL"
    print_info "[-] Branch: $CLONE_BRANCH"

    if ! git clone "$GIT_REPO_URL" -b "$CLONE_BRANCH" "$BUILD_DIR"; then
        print_error "[-] Failed to clone repository"
        exit 1
    fi
    print_success "[-] Repository cloned successfully"

    # Fix permissions on newly cloned directory (only on device, not macOS)
    if [ "$BUILD_DIR" = "/data/bluepilot-build" ]; then
        sudo chown -R comma:comma "$BUILD_DIR" 2>/dev/null || true
        sudo chmod 755 "$BUILD_DIR" 2>/dev/null || true
    fi

    cd "$BUILD_DIR" || exit 1

    # Update submodules if any
    if [ -f ".gitmodules" ]; then
        print_info "[-] Updating submodules..."
        if ! git submodule update --init --recursive; then
            print_error "[-] Failed to update submodules"
            exit 1
        fi
        print_success "[-] Submodules updated successfully"
    fi

    # Setup git environment
    setup_git_env

    # Build the project
    build_openpilot

    # Process panda directory
    handle_panda_directory

    # Convert submodules to regular directories
    process_submodules

    # Create gitignore files
    create_opendbc_gitignore
    update_main_gitignore

    # Cleanup unnecessary files
    cleanup_files

    # Create prebuilt marker
    create_prebuilt_marker

    # Prepare commit and push
    prepare_commit_push

    cd "$CURRENT_DIR" || exit 1

    print_info ""
    print_info "=========================================="
    print_success "Build Process Completed Successfully!"
    print_info "=========================================="
    print_info ""
    print_success "[+] Source Branch: $CLONE_BRANCH"
    print_success "[+] Build Branch: $BUILD_BRANCH"
    print_success "[+] Repository: $GIT_REPO_URL"
    print_info ""
}

###############################################################################
# Execute Build
###############################################################################
build_repo_branch
