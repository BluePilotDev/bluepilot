#!/usr/bin/env bash

has_submodule_changes() {
  local submodule_path="$1"
  if [ -n "$SUBMODULE_PATHS" ]; then
    echo "$SUBMODULE_PATHS" | grep -q "$submodule_path"
    return $?
  fi
  return 1
}

while read -r hash submodule ref; do
  if [ -z "$hash" ] || [ -z "$submodule" ]; then
    continue
  fi

  hash=$(echo "$hash" | sed 's/^[+-]//')

  # BluePilot: JetLink is pinned to a release tag (its default branch is main): the pin must be a tagged commit
  if [ "$submodule" = "jetlink_repo" ]; then
    git -C "$submodule" fetch --depth 100 origin "+refs/tags/*:refs/tags/*"
    if [ -n "$(git -C "$submodule" tag --points-at "$hash")" ]; then
      echo "$submodule ok (tag $(git -C "$submodule" tag --points-at "$hash" | head -1))"
      continue
    fi
    echo "$submodule: $hash is not a tagged JetLink release"
    exit 1
  fi
  # End BluePilot

  if [ "$submodule" = "tinygrad_repo" ]; then
    echo "Skipping $submodule"
    continue
  fi

  if [ "$CHECK_PR_REFS" = "true" ] && has_submodule_changes "$submodule"; then
    echo "Checking $submodule (non-master): verifying hash $hash exists"
    git -C "$submodule" fetch --depth 100 origin
    if git -C "$submodule" cat-file -e "$hash" 2>/dev/null; then
      echo "$submodule ok (hash exists)"
    else
      echo "$submodule: $hash does not exist in the repository"
      exit 1
    fi
  else
    git -C "$submodule" fetch --depth 100 origin master
    git -C "$submodule" branch -r --contains "$hash" | grep "origin/master"
    if [ "$?" -eq 0 ]; then
      echo "$submodule ok"
    else
      echo "$submodule: $hash not on master"
      exit 1
    fi
  fi
done <<< "$(git submodule status)"
