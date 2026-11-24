#!/usr/bin/env bash
# A library release, through to a bp-sunnypilot PR (spec: ford-op/bg-opendbc#97). Bumps the library in
# ford-op/opendbc and merges that PR once its required checks pass (nothing else auto-merges there), then
# opens the bp-sunnypilot PR that moves opendbc_repo to the new bp-opendbc, saying whether it changes the
# panda firmware or the lateral Python (a drive before merge if so). Not merged automatically.
# usage: GH_TOKEN=<bluepilot-sync App token for opendbc and bp-sunnypilot> release_bumps.sh <tag> <commit>
set -euo pipefail
TAG=$1 SHA=$2
LIB=$(pwd)
WORK=$(mktemp -d)
git config --global user.name "bluepilot-sync[bot]"
git config --global user.email "348581487+bluepilot-sync[bot]@users.noreply.github.com"
OWNER=ford-op  # where opendbc and bp-sunnypilot live
# the clones have no submodules checked out: a fetch that recursed into a moved gitlink would fail
git config --global fetch.recurseSubmodules false
clone() {  # <repo> <branch> <dir>: history and trees only, no checkout, index at HEAD
  GIT_LFS_SKIP_SMUDGE=1 git -c filter.lfs.process= -c filter.lfs.required=false clone -q --filter=blob:none \
    --no-checkout -b "$2" "https://x-access-token:${GH_TOKEN}@github.com/$OWNER/$1.git" "$3"
  git -C "$3" read-tree HEAD
}
bump() {  # <dir> <path> <commit> <branch> <message>: commit a new gitlink and push it as <branch>, the
  # job's own branch (forced, so a rerun after a failure replaces its earlier attempt)
  git -C "$1" update-index --cacheinfo "160000,$3,$2"
  git -C "$1" -c filter.lfs.process= -c filter.lfs.required=false commit -q -m "$5"
  git -C "$1" push -q --force origin "HEAD:refs/heads/$4"
}
open_pr() {  # <repo> <base> <branch> <title> <body>: the branch's open PR, or a new one
  local url
  url=$(gh pr list -R "$OWNER/$1" -H "$3" --state open --json url -q '.[0].url')
  if [ -n "$url" ]; then gh pr edit "$url" --title "$4" --body "$5" >/dev/null; echo "$url"
  else gh pr create -R "$OWNER/$1" -B "$2" -H "$3" --title "$4" --body "$5"; fi
}
# bp-opendbc's required checks (its branch protection; the App can't read protection, so they're listed)
REQUIRED=("./test.sh (Linux)" "safety" "bluepilot-safety" "bluepilot-hooks")
wait_required() {  # <pr>: until every required check has reported and passed; fails on the first failure
  local out n state
  for _ in $(seq 150); do
    out=$(gh pr checks "$1" --required --json name,bucket 2>/dev/null || true)
    if jq -e 'any(.[]; .bucket == "fail" or .bucket == "cancel")' <<<"${out:-[]}" >/dev/null; then
      jq -r '.[] | select(.bucket == "fail" or .bucket == "cancel") | "failed: \(.name)"' <<<"$out"; return 1
    fi
    n=0
    for name in "${REQUIRED[@]}"; do
      state=$(jq -r --arg n "$name" '.[] | select(.name == $n) | .bucket' <<<"${out:-[]}")
      [ "$state" = pass ] && n=$((n + 1))
    done
    [ "$n" = "${#REQUIRED[@]}" ] && return 0
    sleep 20
  done
  echo "required checks didn't finish in 50 minutes"; return 1
}

# 1. ford-op/opendbc: the library pointer, merged on green
clone opendbc bp-opendbc "$WORK/bg"
PIN=$(git -C "$WORK/bg" rev-parse HEAD:opendbc/bluepilot_lateral)
if ! git -C "$LIB" merge-base --is-ancestor "$PIN" "$SHA"; then
  echo "$TAG (${SHA:0:8}) isn't ahead of ford-op/opendbc's pin ${PIN:0:8}: nothing to do"; exit 1
fi
if [ "$PIN" = "$SHA" ]; then
  echo "ford-op/opendbc already pins $TAG"
  BG=$(git -C "$WORK/bg" rev-parse HEAD)
else
  BRANCH="bump/library-$TAG"
  bump "$WORK/bg" opendbc/bluepilot_lateral "$SHA" "$BRANCH" "bluepilot_lateral: bump to $TAG"
  PR=$(open_pr opendbc bp-opendbc "$BRANCH" "bluepilot_lateral: bump to $TAG" \
"The library pointer moves to bluepilot-lateral [$TAG](https://github.com/BluePilotDev/bluepilot-lateral/releases/tag/$TAG) (${SHA:0:8}). Opened by the library's release job, which merges it once the required checks pass (ford-op/bg-opendbc#97).

## Validation
- [ ] CI \`./test.sh (Linux)\`
- [ ] CI \`safety\`
- [ ] CI \`bluepilot-safety\`
- [ ] CI \`bluepilot-hooks\`")
  echo "ford-op/opendbc: $PR"
  wait_required "$PR"
  gh pr view "$PR" --json body -q .body | sed 's/^- \[ \] CI/- [x] CI/' > "$WORK/body.md"
  gh pr edit "$PR" --body-file "$WORK/body.md"
  # its own message: with GitHub's default, a squash merge by the App adds the App as a co-author
  gh pr merge "$PR" --squash --delete-branch --body "Moves the library to $TAG."
  BG=$(gh pr view "$PR" --json mergeCommit -q .mergeCommit.oid)
  git -C "$WORK/bg" fetch -q origin bp-opendbc
fi

# 2. bp-sunnypilot: opendbc_repo to that bp-opendbc, as a PR
clone bp-sunnypilot bp-sunnypilot "$WORK/sp"
OLD=$(git -C "$WORK/sp" rev-parse HEAD:opendbc_repo)
if [ "$OLD" = "$BG" ]; then echo "bp-sunnypilot already at ${BG:0:8}"; exit 0; fi
OLDLIB=$(git -C "$WORK/bg" rev-parse "$OLD:opendbc/bluepilot_lateral")
changed() {  # what moved: ford-op/opendbc outside the library, then the library itself
  git -C "$WORK/bg" diff --name-only "$OLD" "$BG" -- . ':!opendbc/bluepilot_lateral'
  git -C "$LIB" diff --name-only "$OLDLIB" "$SHA" | sed 's|^|opendbc/bluepilot_lateral/|'
}
FILES=$(changed)
NOTESTS=$(grep -v '/tests/' <<<"$FILES" || true)
FW=$(grep -E '\.(h|c)$' <<<"$NOTESTS" || true)
LAT=$(grep -E '^opendbc/(bluepilot_lateral|car/ford|sunnypilot/car/ford)/.*\.py$|^opendbc/car/(lateral|interfaces|structs)\.py$|^opendbc/sunnypilot/car/interfaces\.py$|^opendbc/dbc/ford' <<<"$NOTESTS" || true)
# shellcheck disable=SC2016  # the backticks are Markdown, not expansions
list() { if [ -n "$1" ]; then sed 's/^/  - `/; s/$/`/' <<<"$1"; else echo "  - none"; fi; }
# A drive is only for BluePilot's own Ford safety or F-150 lateral behaviour -- not upstream
# firmware, other-platform rows or host-only files (upstream-only rides CI, per the drive rule).
DRIVE_FW=$(grep -E '^opendbc/safety/modes/ford\.h$|^opendbc/bluepilot_lateral/safety/.*\.(h|c)$' <<<"$FW" || true)
DRIVE_LAT=$(grep -E 'bluepilot_lateral/(can/fordcan_ext|lateral_angle_ext|carcontroller_ext)\.py$|^opendbc/car/ford/(carcontroller|fordcan)\.py$|^opendbc/dbc/ford' <<<"$LAT" || true)
# core/values_ext.py holds per-platform tables; a drive only if a Ford F-150 row actually changed.
if grep -q 'bluepilot_lateral/core/values_ext\.py$' <<<"$LAT" \
   && git -C "$LIB" diff "$OLDLIB" "$SHA" -- core/values_ext.py | grep -qE '^[+-].*F_150'; then
  DRIVE_LAT=$(printf '%s\nopendbc/bluepilot_lateral/core/values_ext.py\n' "$DRIVE_LAT")
fi
DRIVE_FILES=$(printf '%s\n%s\n' "$DRIVE_FW" "$DRIVE_LAT" | grep -v '^$' | sort -u || true)
if [ -n "$DRIVE_FILES" ]; then
  CLASS="Changes BluePilot's Ford safety or F-150 lateral, so this needs a drive before merge."
  DRIVE="
- [ ] A drive: BluePilot's Ford safety or F-150 lateral changed"
else
  CLASS="No BluePilot Ford safety or F-150 lateral change above (upstream firmware / other-platform / host-only), so this rides CI -- no drive."
  DRIVE=""
fi
BRANCH="opendbc/library-$TAG"
bump "$WORK/sp" opendbc_repo "$BG" "$BRANCH" "opendbc_repo: bp-opendbc ${BG:0:8} (bluepilot-lateral $TAG)"
PR=$(open_pr bp-sunnypilot bp-sunnypilot "$BRANCH" "opendbc_repo: bluepilot-lateral $TAG" \
"\`opendbc_repo\` moves to ford-op/opendbc \`bp-opendbc\` ${BG:0:8}, which has bluepilot-lateral [$TAG](https://github.com/BluePilotDev/bluepilot-lateral/releases/tag/$TAG). Opened by the library's release job (ford-op/bg-opendbc#97); not merged automatically.

- Panda firmware (\`.h\`/\`.c\`) changed:
$(list "$FW")
- Lateral Python or Ford DBC changed (outside tests):
$(list "$LAT")

$CLASS

## Validation
- [ ] CI \`build release\`
- [ ] CI \`build macOS\`
- [ ] CI \`static analysis\`
- [ ] CI \`unit tests\`
- [ ] CI \`process replay\`: the Ford segment matches bp-sunnypilot's own reference, or the difference is explained$DRIVE")

# BluePilot: a plain-English review of the bump's F-150 impact and drive status, posted on the bump
# PR. Fail-soft: it never blocks the release. Prompt: .github/bump-review-prompt.md.
if [ -n "${CLAUDE_CODE_OAUTH_TOKEN:-}" ] && [ -n "${PR:-}" ]; then
  review_bump() {
    npm install -g --silent @anthropic-ai/claude-code || return 0
    {
      cat "$(dirname "$0")/bump-review-prompt.md"
      printf '\n## Diffs (non-test files changed by this bump)\n'
      while IFS= read -r f; do
        [ -z "$f" ] && continue
        printf '\n### %s\n```diff\n' "$f"
        case "$f" in
          opendbc/bluepilot_lateral/*) git -C "$LIB" diff "$OLDLIB" "$SHA" -- "${f#opendbc/bluepilot_lateral/}" | head -200 ;;
          *) git -C "$WORK/bg" diff "$OLD" "$BG" -- "$f" | head -200 ;;
        esac
        printf '```\n'
      done <<<"$NOTESTS"
    } > bump-prompt.md
    claude -p "$(cat bump-prompt.md)" --max-turns 6 < /dev/null > bump-review.md || return 0
    [ -s bump-review.md ] || return 0
    { echo "<!-- bump-review $TAG -->"; cat bump-review.md; } > bump-comment.md
    gh pr comment "$PR" --body-file bump-comment.md || return 0
  }
  review_bump || true
fi
