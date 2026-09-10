#!/usr/bin/env bash
#
# archive-data.sh -- preserve lamp-lab run data and taught state.
#
# Deliberately plain bash with no dependency on the repo's pyenv, tools.config,
# or any Python at all. The moments this script exists for -- a repo reorg, a
# broken refactor, a half-migrated environment -- are exactly the moments the
# application cannot be trusted to tell you where its data lives. A backup that
# imports the thing it is backing up is not a backup.
#
# Two tiers, handled differently on purpose:
#
#   Tier 1  ~/.sdl_lab   taught/measured state. MUTABLE and OVERWRITTEN in place,
#                        so it gets dated snapshots and never a mirror. On
#                        2026-07-31 routes.json went from 4046 bytes to 2; a
#                        mirror would have faithfully propagated that and
#                        destroyed the only good copy.
#   Tier 2  dataset/     run output. APPEND-ONLY by nature, so an additive
#                        mirror is right. rsync is called WITHOUT --delete,
#                        always. Removing that guarantee is the whole failure.
#
# Never deletes anything, in either tier, under any flag.
#
# Usage:
#   archive-data.sh              archive now
#   archive-data.sh --dry-run    show what would happen, write nothing
#   archive-data.sh --check      report archive status and staleness only
#   archive-data.sh --force      snapshot tier 1 even if unchanged
#
set -euo pipefail

ARCHIVE_ROOT=${SDL_ARCHIVE_ROOT:-/mnt/c/Users/Public/SDLarchive}
STATE_SRC=${SDL_LAB_STATE:-$HOME/.sdl_lab}
REPO=${SDL_REPO:-$HOME/SDLsetup}
DATASET_SRC="$REPO/dataset"

# The stale 2026-07-31 monitor log is 420 MB of transient stream chatter, not
# state. Everything else under .sdl_lab is kept.
STATE_EXCLUDE="microscope_monitor/live_session.log"

STALE_AFTER_DAYS=7

DRY=0; CHECK=0; FORCE=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    --check)   CHECK=1 ;;
    --force)   FORCE=1 ;;
    -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
    *) echo "unknown argument: $a" >&2; exit 2 ;;
  esac
done

STAMP=$(date +%Y%m%d-%H%M%S)
say()  { printf '%s\n' "$*"; }
die()  { printf 'ARCHIVE FAILED: %s\n' "$*" >&2; exit 1; }

# Combined fingerprint of the tier-1 tree: sorted paths plus content hashes.
state_tree_hash() {
  local dir="$1"
  ( cd "$dir" && find . -type f ! -path "./$STATE_EXCLUDE" -print0 \
      | sort -z | xargs -0 sha256sum 2>/dev/null | sha256sum | cut -d' ' -f1 )
}

tree_count() { find "$1" -type f 2>/dev/null | wc -l | tr -d ' '; }
tree_bytes() { find "$1" -type f -printf '%s\n' 2>/dev/null | awk '{s+=$1} END {print s+0}'; }

# ---------------------------------------------------------------- preflight
[ -d "$STATE_SRC" ]   || die "taught state not found at $STATE_SRC"
[ -d "$DATASET_SRC" ] || die "dataset not found at $DATASET_SRC"

ARCHIVE_PARENT=$(dirname "$ARCHIVE_ROOT")
[ -d "$ARCHIVE_PARENT" ] || die "archive parent $ARCHIVE_PARENT does not exist (is the Windows drive mounted?)"
if [ "$CHECK" -eq 0 ] && [ "$DRY" -eq 0 ]; then
  mkdir -p "$ARCHIVE_ROOT/state" "$ARCHIVE_ROOT/dataset" "$ARCHIVE_ROOT/bin" \
    || die "cannot create archive tree under $ARCHIVE_ROOT"
  touch "$ARCHIVE_ROOT/.wtest" 2>/dev/null || die "archive root $ARCHIVE_ROOT is not writable"
  rm -f "$ARCHIVE_ROOT/.wtest"
fi

LAST_SNAP=""
if [ -d "$ARCHIVE_ROOT/state" ]; then
  LAST_SNAP=$(find "$ARCHIVE_ROOT/state" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -1)
fi

# ------------------------------------------------------------------- status
say "archive root : $ARCHIVE_ROOT"
say "state source : $STATE_SRC   ($(tree_count "$STATE_SRC") files)"
say "dataset src  : $DATASET_SRC ($(tree_count "$DATASET_SRC") files, $(( $(tree_bytes "$DATASET_SRC") / 1048576 )) MB)"

if [ -n "$LAST_SNAP" ]; then
  age_days=$(( ( $(date +%s) - $(stat -c %Y "$LAST_SNAP") ) / 86400 ))
  say "last snapshot: $(basename "$LAST_SNAP")  (${age_days}d ago)"
  if [ "$age_days" -ge "$STALE_AFTER_DAYS" ]; then
    say ""
    say "*** ARCHIVE IS STALE: last run was ${age_days} days ago (threshold ${STALE_AFTER_DAYS}d) ***"
  fi
else
  say "last snapshot: NONE -- this machine has never been archived"
fi

if [ "$CHECK" -eq 1 ]; then
  say ""
  say "archived dataset: $(tree_count "$ARCHIVE_ROOT/dataset") files"
  say "state snapshots : $(find "$ARCHIVE_ROOT/state" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l | tr -d ' ')"
  exit 0
fi

# ------------------------------------------------- tier 1: state snapshot
CUR_HASH=$(state_tree_hash "$STATE_SRC")
PREV_HASH=""
[ -n "$LAST_SNAP" ] && [ -f "$LAST_SNAP/.treehash" ] && PREV_HASH=$(cat "$LAST_SNAP/.treehash")

SNAP_DEST="$ARCHIVE_ROOT/state/$STAMP"
if [ "$CUR_HASH" = "$PREV_HASH" ] && [ "$FORCE" -eq 0 ]; then
  say ""
  say "tier 1: taught state unchanged since $(basename "$LAST_SNAP") -- no new snapshot"
  SNAP_RESULT="unchanged"
else
  say ""
  say "tier 1: taught state CHANGED (or forced) -- snapshotting to state/$STAMP"
  if [ "$DRY" -eq 1 ]; then
    say "        [dry-run] would rsync $STATE_SRC/ -> $SNAP_DEST/"
    SNAP_RESULT="dry-run"
  else
    mkdir -p "$SNAP_DEST"
    rsync -a --exclude="$STATE_EXCLUDE" "$STATE_SRC/" "$SNAP_DEST/" \
      || die "tier 1 rsync failed"
    printf '%s\n' "$CUR_HASH" > "$SNAP_DEST/.treehash"
    # verify every tier-1 file by hash; the tree is small enough that a
    # sample would be an excuse rather than a check.
    bad=0
    while IFS= read -r -d '' f; do
      rel=${f#"$STATE_SRC"/}
      [ "$rel" = "$STATE_EXCLUDE" ] && continue
      a=$(sha256sum "$f" | cut -d' ' -f1)
      b=$(sha256sum "$SNAP_DEST/$rel" 2>/dev/null | cut -d' ' -f1 || true)
      [ "$a" = "$b" ] || { say "        MISMATCH $rel"; bad=$((bad+1)); }
    done < <(find "$STATE_SRC" -type f -print0)
    [ "$bad" -eq 0 ] || die "tier 1 verification failed on $bad file(s)"
    say "        verified $(tree_count "$SNAP_DEST") files by hash"
    SNAP_RESULT="snapshot $STAMP"
  fi
fi

# ------------------------------------------------- tier 2: dataset mirror
say ""
say "tier 2: mirroring dataset (additive; --delete is never used)"
if [ "$DRY" -eq 1 ]; then
  rsync -an --info=stats2 "$DATASET_SRC/" "$ARCHIVE_ROOT/dataset/" | tail -12
  MIRROR_RESULT="dry-run"
else
  rsync -a --info=stats2 "$DATASET_SRC/" "$ARCHIVE_ROOT/dataset/" | tail -8 \
    || die "tier 2 rsync failed"
  src_n=$(tree_count "$DATASET_SRC");            dst_n=$(tree_count "$ARCHIVE_ROOT/dataset")
  src_b=$(tree_bytes "$DATASET_SRC");            dst_b=$(tree_bytes "$ARCHIVE_ROOT/dataset")
  [ "$dst_n" -ge "$src_n" ] || die "tier 2 count check failed: source $src_n, archive $dst_n"
  [ "$dst_b" -ge "$src_b" ] || die "tier 2 byte check failed: source $src_b, archive $dst_b"
  say "        source $src_n files / $((src_b/1048576)) MB   archive $dst_n files / $((dst_b/1048576)) MB"
  # spot-check five newest files by hash
  bad=0; n=0
  while IFS= read -r f; do
    rel=${f#"$DATASET_SRC"/}
    a=$(sha256sum "$f" | cut -d' ' -f1)
    b=$(sha256sum "$ARCHIVE_ROOT/dataset/$rel" 2>/dev/null | cut -d' ' -f1 || true)
    [ "$a" = "$b" ] || { say "        MISMATCH $rel"; bad=$((bad+1)); }
    n=$((n+1))
  done < <(find "$DATASET_SRC" -type f -printf '%T@ %p\n' | sort -rn | head -5 | cut -d' ' -f2-)
  [ "$bad" -eq 0 ] || die "tier 2 spot-check failed on $bad of $n file(s)"
  say "        spot-checked $n newest files by hash"
  MIRROR_RESULT="$src_n files / $((src_b/1048576)) MB"
fi

# ------------------------------------------------------- record and self-copy
if [ "$DRY" -eq 0 ]; then
  cp -f "$0" "$ARCHIVE_ROOT/bin/$(basename "$0")" 2>/dev/null || true
  printf '%s  host=%s  state=%s  dataset=%s\n' \
    "$(date -Iseconds)" "$(hostname)" "$SNAP_RESULT" "$MIRROR_RESULT" \
    >> "$ARCHIVE_ROOT/archive.log"
  {
    printf 'last_run: %s\n' "$(date -Iseconds)"
    printf 'state: %s\n' "$SNAP_RESULT"
    printf 'dataset: %s\n' "$MIRROR_RESULT"
    printf 'state_snapshots: %s\n' "$(find "$ARCHIVE_ROOT/state" -mindepth 1 -maxdepth 1 -type d | wc -l | tr -d ' ')"
  } > "$ARCHIVE_ROOT/LATEST.txt"
fi

say ""
say "archive OK  ($SNAP_RESULT; dataset $MIRROR_RESULT)"
