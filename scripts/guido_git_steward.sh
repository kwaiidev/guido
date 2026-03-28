#!/bin/zsh

set -euo pipefail

REPO_ROOT="/Users/carlos/Development/Guido"
PROMPT_FILE="$REPO_ROOT/scripts/automation/guido_git_steward_prompt.md"
CODEX_BIN="/Applications/Codex.app/Contents/Resources/codex"
STATE_DIR="/Users/carlos/Library/Caches/guido-git-steward"
LOG_DIR="/Users/carlos/Library/Logs/Guido"
LOCK_DIR="$STATE_DIR/run.lock"
LAST_MESSAGE_FILE="$STATE_DIR/last_message.txt"
RUN_LOG="$LOG_DIR/git-steward-run.log"

mkdir -p "$STATE_DIR" "$LOG_DIR"

if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  exit 0
fi

cleanup() {
  rmdir "$LOCK_DIR" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

if [[ ! -x "$CODEX_BIN" ]]; then
  printf '%s missing codex binary at %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$CODEX_BIN" >> "$RUN_LOG"
  exit 1
fi

if [[ ! -d "$REPO_ROOT/.git" && ! -f "$REPO_ROOT/.git" ]]; then
  printf '%s missing git repo at %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$REPO_ROOT" >> "$RUN_LOG"
  exit 1
fi

printf '%s start\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$RUN_LOG"

set +e
"$CODEX_BIN" exec \
  -C "$REPO_ROOT" \
  --sandbox workspace-write \
  -m gpt-5.4-mini \
  -c model_reasoning_effort="low" \
  -o "$LAST_MESSAGE_FILE" \
  - < "$PROMPT_FILE" >> "$RUN_LOG" 2>&1
exit_code=$?
set -e

printf '%s exit_code=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$exit_code" >> "$RUN_LOG"

exit "$exit_code"
