#!/usr/bin/env bash
# Render every record id in <list_file> via render_object.sh.
# Logs to ./batch.log. Reports failures at the end.
#
# Usage: bash render_batch.sh <list_file> [extra args passed to render_object.sh]
set -uo pipefail

if [ $# -lt 1 ]; then
    echo "usage: $0 <list_file> [extra render_object.sh args]" >&2
    exit 1
fi

LIST="$1"; shift
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$HERE/batch.log"
: > "$LOG"

RECS=()
while IFS= read -r line || [ -n "$line" ]; do
    case "$line" in
        ''|\#*) ;;
        *) RECS+=("$line") ;;
    esac
done < "$LIST"
TOTAL=${#RECS[@]}
FAILED=()
i=0
for rec in "${RECS[@]}"; do
    i=$((i+1))
    echo "[batch] [$i/$TOTAL] $rec" | tee -a "$LOG"
    if ! bash "$HERE/render_object.sh" "$rec" "$@" >>"$LOG" 2>&1; then
        echo "[batch] FAILED: $rec" | tee -a "$LOG"
        FAILED+=("$rec")
    fi
done

echo "[batch] done: $((TOTAL - ${#FAILED[@]}))/$TOTAL ok" | tee -a "$LOG"
if [ ${#FAILED[@]} -gt 0 ]; then
    echo "[batch] failed:" | tee -a "$LOG"
    for r in "${FAILED[@]}"; do echo "  $r" | tee -a "$LOG"; done
    exit 1
fi
