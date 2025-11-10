#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

cleanup_path() {
  local path="$1"
  if [[ -d "$path" ]]; then
    if [[ -f "$path/.git" || -d "$path/.git" ]]; then
      # Already initialised as a git repository/submodule. Leave untouched.
      return
    fi

    if find "$path" -mindepth 1 -print -quit >/dev/null 2>&1; then
      echo "Removing pre-existing directory '$path' to allow submodule checkout." >&2
    else
      echo "Removing empty directory '$path' to allow submodule checkout." >&2
    fi
    rm -rf "$path"
  fi
}

cleanup_path "$REPO_ROOT/TMC-API"
cleanup_path "$REPO_ROOT/ds4_driver"
cleanup_path "$REPO_ROOT/omnidirectional_controllers"

git submodule update --init --recursive --remote "$@"
