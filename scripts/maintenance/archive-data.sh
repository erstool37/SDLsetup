#!/usr/bin/env bash
#
# archive-data.sh -- preserve lamp-lab run data and taught state.
#
# Deliberately plain bash with no dependency on the repo's pyenv, tools.config,
# or any Python. The moments this exists for -- a repo reorg, a broken refactor,
# a half-migrated environment -- are exactly the moments the application cannot
# be trusted to say where its data lives. A backup that imports the thing it is
# backing up is not a backup.
#
# Two tiers, handled differently on purpose:
#
#   Tier 1  ~/.sdl_lab   taught/measured state. MUTABLE, overwritten in place,
#                        so it gets dated immutable snapshots and never a
#                        mirror. On 2026-07-31 routes.json went from 4046 bytes
#                        to 2; a mirror would have propagated that over the last
#                        good copy.
#   Tier 2  dataset/     run output. APPEND-ONLY, so an additive mirror is
#                        right. rsync is called WITHOUT --delete, always.
#
# Never deletes anything, in either tier, under any flag.
#
# The guards below are not defensive padding. Each one is a finding from an
# adversarial review on 2026-09-10, and the comment on each says which failure
# it prevents. Do not remove one to make a run shorter.
#
# Usage:
#   archive-data.sh              archive now
#   archive-data.sh --dry-run    show what would happen, write nothing
#   archive-data.sh --check      report status; NON-ZERO if the archive is bad
#   archive-data.sh --force      snapshot tier 1 even if unchanged
#
set -Eeuo pipefail

ARCHIVE_ROOT=${SDL_ARCHIVE_ROOT:-/mnt/c/Users/Public/SDLarchive}
STATE_SRC=${SDL_LAB_STATE:-$HOME/.sdl_lab}
REPO=${SDL_REPO:-$HOME/SDLsetup}

# Trailing slashes make the ${f#"$SRC"/} prefix strip silently produce an
# absolute path, which turns verification into a total false failure.
ARCHIVE_ROOT=${ARCHIVE_ROOT%/}; STATE_SRC=${STATE_SRC%/}; REPO=${REPO%/}
DATASET_SRC="$REPO/dataset"

# Not copied at all: 420 MB of transient stream chatter, not state.
STATE_EXCLUDE="microscope_monitor/live_session.log"

# Copied like everything else, but excluded from the change-detection hash.
# Live processes rewrite these every few seconds (verified 2026-09-10: the
# hash moved every 2 s), so including them makes every run look like a change
# and buries a real taught-state edit in noise.
STATE_VOLATILE_RE='(environment/last_reading\.json|occupancy/.*\.json|microscope_monitor/live_session\.pid)$'

# /mnt/c is 9p/drvfs mounted WITHOUT the metadata option (verified 2026-09-10),
# so ownership and mode cannot be set there. Plain -a asks for -o -g and can
# abort the whole archive over cosmetics.
RSYNC_FLAGS="-rlptD --no-owner --no-group --omit-dir-times --modify-window=1"

STALE_AFTER_DAYS=7

DRY=0; CHECK=0; FORCE=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    --check)   CHECK=1 ;;
    --force)   FORCE=1 ;;
    -h|--help) sed -n '2,40p' "$0"; exit 0 ;;
    *) echo "unknown argument: $a" >&2; exit 2 ;;
  esac
done

STAMP=$(date +%Y%m%d-%H%M%S)
LOCK="$ARCHIVE_ROOT/.lock"
LOCK_HELD=0
TMPFILES=""

say() { printf '%s\n' "$*"; }
die() { printf 'ARCHIVE FAILED: %s\n' "$*" >&2; exit 1; }

cleanup() {
  # shellcheck disable=SC2086
  [ -n "$TMPFILES" ] && rm -f $TMPFILES 2>/dev/null
  [ "$LOCK_HELD" -eq 1 ] && rmdir "$LOCK" 2>/dev/null
  return 0
}
trap cleanup EXIT
# Without this, a failing $(...) kills the script with a bare status and no
# message -- non-zero, but not "say so loudly".
trap 'die "aborted at line $LINENO (rc=$?)"' ERR

# Content fingerprint of tier 1. LC_ALL=C because sort collation is
# locale-dependent and this lab has non-ASCII directory names, so the same
# unchanged tree would hash differently under cron and under a login shell.
# Symlinks are enumerated too: a taught file replaced by a link must not become
# invisible to both the hash and the verification.
state_tree_hash() {
  local dir="$1"
  ( cd "$dir" \
      && LC_ALL=C find . \( -type f -o -type l \) ! -path "./$STATE_EXCLUDE" -print0 \
       | grep -zEv "$STATE_VOLATILE_RE" \
       | LC_ALL=C sort -z \
       | xargs -r0 sha256sum \
       | LC_ALL=C sha256sum | cut -d' ' -f1 )
}

tree_count() { find "$1" -type f 2>/dev/null | wc -l | tr -d ' '; }
tree_bytes() { find "$1" -type f -printf '%s\n' 2>/dev/null | awk '{s+=$1} END {print s+0}'; }

# Age from the directory NAME, never from mtime. A copied archive has fresh
# mtimes and would report itself newly backed up; a forward clock jump would
# make the age negative and silence the staleness alarm forever.
snap_age_days() {
  local n; n=$(basename "$1")
  local t; t=$(date -d "${n:0:4}-${n:4:2}-${n:6:2} ${n:9:2}:${n:11:2}:${n:13:2}" +%s 2>/dev/null) || { echo 99999; return 0; }
  echo $(( ( $(date +%s) - t ) / 86400 ))
}

# Only timestamp-shaped directories count, so a stray or hand-made directory
# cannot sort last and freeze the change detection.
newest_snapshot() {
  find "$ARCHIVE_ROOT/state" -mindepth 1 -maxdepth 1 -type d \
    -regextype posix-extended -regex '.*/[0-9]{8}-[0-9]{6}$' 2>/dev/null | LC_ALL=C sort | tail -1
}

# ---------------------------------------------------------------- preflight
# A symlinked source passes [ -d ] but yields zero files from find, which would
# make the tree hash a constant and freeze tier 1 forever while still printing OK.
STATE_SRC=$(readlink -f "$STATE_SRC")     || die "cannot resolve taught-state path"
DATASET_SRC=$(readlink -f "$DATASET_SRC") || die "cannot resolve dataset path"
[ -d "$STATE_SRC" ]   || die "taught state not found at $STATE_SRC"
[ -d "$DATASET_SRC" ] || die "dataset not found at $DATASET_SRC"
[ "$(tree_count "$STATE_SRC")"   -gt 0 ] || die "taught state at $STATE_SRC contains ZERO files"
[ "$(tree_count "$DATASET_SRC")" -gt 0 ] || die "dataset at $DATASET_SRC contains ZERO files"

[ -d "$(dirname "$ARCHIVE_ROOT")" ] || die "archive parent missing -- is the Windows drive mounted?"

if [ "$CHECK" -eq 0 ] && [ "$DRY" -eq 0 ]; then
  mkdir -p "$ARCHIVE_ROOT/state" "$ARCHIVE_ROOT/dataset" "$ARCHIVE_ROOT/bin" \
    || die "cannot create archive tree under $ARCHIVE_ROOT"
  WPROBE="$ARCHIVE_ROOT/.wtest.$$"
  touch "$WPROBE" 2>/dev/null || die "archive root $ARCHIVE_ROOT is not writable"
  rm -f "$WPROBE"   # our own probe, uniquely named, never a pre-existing file
  mkdir "$LOCK" 2>/dev/null || die "another archive run holds $LOCK -- refusing to run two at once"
  LOCK_HELD=1
fi

LAST_SNAP=$(newest_snapshot)

# ------------------------------------------------------------------- status
say "archive root : $ARCHIVE_ROOT"
say "state source : $STATE_SRC   ($(tree_count "$STATE_SRC") files)"
say "dataset src  : $DATASET_SRC ($(tree_count "$DATASET_SRC") files, $(( $(tree_bytes "$DATASET_SRC") / 1048576 )) MB)"

AGE_DAYS=-1
if [ -n "$LAST_SNAP" ]; then
  AGE_DAYS=$(snap_age_days "$LAST_SNAP")
  say "last snapshot: $(basename "$LAST_SNAP")  (${AGE_DAYS}d ago)"
  [ "$AGE_DAYS" -ge "$STALE_AFTER_DAYS" ] && say "
*** ARCHIVE IS STALE: last run was ${AGE_DAYS} days ago (threshold ${STALE_AFTER_DAYS}d) ***"
else
  say "last snapshot: NONE -- this machine has never been archived"
fi

# Coverage gaps, announced every run. A test for a specific path is useless
# here: other users' homes are mode 750, so [ -e ] on anything inside them
# returns false for "permission denied" and the gap goes quiet -- which is
# precisely how ~/camera_captures stayed unnoticed for five weeks. Report the
# unreadability itself instead. Verified 2026-09-10: a live microscope session
# runs as another user and saves frames into its own home.
for h in /home/*; do
  [ "$h" = "$HOME" ] && continue
  [ -d "$h" ] || continue
  if [ -r "$h" ]; then
    [ -e "$h/camera_captures" ] && say "NOT COVERED: $h/camera_captures is outside both tiers"
  else
    say "NOT COVERED: $h is unreadable by $(id -un) -- any capture output there cannot be archived"
  fi
done
true

if [ "$CHECK" -eq 1 ]; then
  rc=0
  say ""
  say "archived dataset: $(tree_count "$ARCHIVE_ROOT/dataset") files"
  say "state snapshots : $(newest_snapshot >/dev/null 2>&1; find "$ARCHIVE_ROOT/state" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l | tr -d ' ')"
  # --check exits non-zero when the answer is "you have no backups", so a
  # monitoring layer that only reads the exit code cannot see green on empty.
  [ -n "$LAST_SNAP" ] || { say "*** NO STATE SNAPSHOTS EXIST ***"; rc=1; }
  [ "$(tree_count "$ARCHIVE_ROOT/dataset")" -gt 0 ] || { say "*** DATASET ARCHIVE IS EMPTY ***"; rc=1; }
  [ -z "$LAST_SNAP" ] || [ "$AGE_DAYS" -lt "$STALE_AFTER_DAYS" ] || rc=1
  exit "$rc"
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
elif [ "$DRY" -eq 1 ]; then
  say ""
  say "tier 1: taught state CHANGED -- would snapshot to state/$STAMP"
  SNAP_RESULT="dry-run"
else
  say ""
  say "tier 1: taught state CHANGED (or forced) -- snapshotting to state/$STAMP"
  # A second-granularity stamp can collide after a clock jump or a double
  # invocation. mkdir -p would succeed and rsync would then overwrite files
  # inside an existing dated snapshot -- destroying the last good copy with a
  # worse one, which is the exact incident this script exists to prevent.
  [ -e "$SNAP_DEST" ] && die "snapshot $SNAP_DEST already exists -- refusing to write into it"
  mkdir "$SNAP_DEST" || die "cannot create $SNAP_DEST"

  # -L stores symlink CONTENT, so the archive never holds a link pointing at a
  # path that does not exist on the Windows drive.
  # shellcheck disable=SC2086
  rsync $RSYNC_FLAGS -L --exclude="$STATE_EXCLUDE" "$STATE_SRC/" "$SNAP_DEST/" \
    || { mv "$SNAP_DEST" "$SNAP_DEST.FAILED" 2>/dev/null; die "tier 1 rsync failed"; }

  # Parts of this tree are rewritten every ~2 s by the running environment
  # monitor, so one source-vs-destination comparison races the writer and fails
  # at random. Re-copy on mismatch and compare again; a mismatch that survives
  # three attempts is real and still aborts.
  bad=0; checked=0; recopied=0
  while IFS= read -r -d '' f; do
    rel=${f#"$STATE_SRC"/}
    [ "$rel" = "$STATE_EXCLUDE" ] && continue
    checked=$((checked+1))
    ok=0
    for _attempt in 1 2 3; do
      b=$(sha256sum "$SNAP_DEST/$rel" 2>/dev/null | cut -d' ' -f1 || true)
      a=$(sha256sum "$f"              2>/dev/null | cut -d' ' -f1 || true)
      [ -n "$a" ] || { ok=1; break; }
      if [ "$a" = "$b" ]; then ok=1; break; fi
      mkdir -p "$(dirname "$SNAP_DEST/$rel")"
      cp -p "$f" "$SNAP_DEST/$rel" 2>/dev/null || true
      recopied=$((recopied+1))
    done
    [ "$ok" -eq 1 ] || { say "        MISMATCH $rel"; bad=$((bad+1)); }
  done < <(LC_ALL=C find "$STATE_SRC" \( -type f -o -type l \) -print0)

  # A loop that ran zero times leaves bad=0 and would pass vacuously; a
  # process substitution's exit status is invisible to set -e.
  [ "$checked" -gt 0 ] || { mv "$SNAP_DEST" "$SNAP_DEST.FAILED" 2>/dev/null; die "tier 1 enumerated ZERO files -- refusing to claim success"; }
  [ "$bad" -eq 0 ]     || { mv "$SNAP_DEST" "$SNAP_DEST.FAILED" 2>/dev/null; die "tier 1 verification failed on $bad file(s)"; }
  say "        verified $checked files by hash"
  [ "$recopied" -eq 0 ] || say "        re-copied $recopied file(s) that changed mid-snapshot"

  # The hash, the copy and the verify all ran against a live tree. A file
  # created after find traversed it would be neither copied nor verified, and
  # the run would still succeed. Re-hash and require the taught state to have
  # held still; if it moved, keep the snapshot (nothing is ever discarded) but
  # withhold the completeness marker so the next run takes a clean one.
  POST_HASH=$(state_tree_hash "$STATE_SRC")
  if [ "$POST_HASH" != "$CUR_HASH" ]; then
    say "        taught state MOVED during the snapshot -- keeping it, but not marking it"
    say "        complete; the next run will take a clean one"
    SNAP_RESULT="snapshot $STAMP (unmarked: source moved mid-copy)"
  else
    # LAST. .treehash is the completeness marker: written only after
    # verification passes, so a failed or interrupted snapshot can never be
    # mistaken for a good one, and a failure can never make the next run
    # report "unchanged" over bad data.
    printf '%s\n' "$CUR_HASH" > "$SNAP_DEST/.treehash"
    SNAP_RESULT="snapshot $STAMP"
  fi
fi

# ------------------------------------------------- tier 2: dataset mirror
say ""
say "tier 2: mirroring dataset (additive; --delete is never used)"
if [ "$DRY" -eq 1 ]; then
  # shellcheck disable=SC2086
  rsync $RSYNC_FLAGS -n --info=stats2 "$DATASET_SRC/" "$ARCHIVE_ROOT/dataset/" | tail -6
  MIRROR_RESULT="dry-run"
else
  # Omitting --delete makes rsync additive for NEW paths only. It still
  # REPLACES a file at a path that already exists, so the 4046-to-2-byte
  # truncation propagates into tier 2 exactly as it would have into tier 1.
  # Growth here is legitimate (a run.log archived mid-run gains lines);
  # shrinkage is not. Copy the archive's current version aside before anything
  # overwrites it with something smaller, so a truncation costs nothing.
  PENDING=$(mktemp); TMPFILES="$TMPFILES $PENDING"
  # shellcheck disable=SC2086
  rsync $RSYNC_FLAGS -n --out-format='%n' "$DATASET_SRC/" "$ARCHIVE_ROOT/dataset/" > "$PENDING" \
    || die "could not compute the pending transfer list"
  JOURNAL="$ARCHIVE_ROOT/superseded/$STAMP"
  shrunk=0
  while IFS= read -r rel; do
    [ -n "$rel" ] || continue
    case "$rel" in */) continue ;; esac
    src="$DATASET_SRC/$rel"; dst="$ARCHIVE_ROOT/dataset/$rel"
    [ -f "$src" ] && [ -f "$dst" ] || continue
    sz_s=$(stat -c %s "$src" 2>/dev/null || echo 0)
    sz_d=$(stat -c %s "$dst" 2>/dev/null || echo 0)
    if [ "$sz_s" -lt "$sz_d" ]; then
      mkdir -p "$(dirname "$JOURNAL/$rel")"
      cp -p "$dst" "$JOURNAL/$rel" || die "could not preserve $rel before it is overwritten by a smaller version"
      say "        SHRANK: $rel ($sz_d -> $sz_s bytes); previous version kept in superseded/$STAMP"
      shrunk=$((shrunk+1))
    fi
  done < "$PENDING"
  [ "$shrunk" -eq 0 ] || say "        journalled $shrunk file(s) that were about to shrink"

  # shellcheck disable=SC2086
  rsync $RSYNC_FLAGS --info=stats2 "$DATASET_SRC/" "$ARCHIVE_ROOT/dataset/" | tail -5 \
    || die "tier 2 rsync failed"

  # The old check was archive_count >= source_count. On an additive mirror that
  # is unfalsifiable: the archive only grows, so once it exceeds the source the
  # assertion is permanently true no matter what this run did. Ask rsync
  # instead what would STILL transfer; after a good sync the answer is nothing.
  # shellcheck disable=SC2086
  residual=$(rsync $RSYNC_FLAGS -n --out-format='%n' "$DATASET_SRC/" "$ARCHIVE_ROOT/dataset/" \
             | grep -cv '/$' || true)
  [ "$residual" -eq 0 ] || die "tier 2: $residual file(s) still differ between source and archive after mirroring"
  src_n=$(tree_count "$DATASET_SRC"); src_b=$(tree_bytes "$DATASET_SRC")
  say "        source $src_n files / $((src_b/1048576)) MB   archive $(tree_count "$ARCHIVE_ROOT/dataset") files"
  say "        rsync residual after mirror: 0 files"
  MIRROR_RESULT="$src_n files / $((src_b/1048576)) MB"
fi

# ------------------------------------------------------- record and self-copy
if [ "$DRY" -eq 0 ]; then
  # This copy is what you reach for when SDLsetup/ is gone. Its absence is
  # worth a warning, not a silent skip.
  cp -f "$0" "$ARCHIVE_ROOT/bin/$(basename "$0")" 2>/dev/null \
    || say "WARNING: could not copy this script into $ARCHIVE_ROOT/bin"
  printf '%s  host=%s  state=%s  dataset=%s\n' \
    "$(date -Iseconds)" "$(hostname)" "$SNAP_RESULT" "$MIRROR_RESULT" \
    >> "$ARCHIVE_ROOT/archive.log"
  {
    printf 'last_run: %s\n' "$(date -Iseconds)"
    printf 'state: %s\n' "$SNAP_RESULT"
    printf 'dataset: %s\n' "$MIRROR_RESULT"
    printf 'complete_snapshots: %s\n' "$(find "$ARCHIVE_ROOT/state" -mindepth 2 -maxdepth 2 -name .treehash 2>/dev/null | wc -l | tr -d ' ')"
    printf 'failed_snapshots: %s\n' "$(find "$ARCHIVE_ROOT/state" -mindepth 1 -maxdepth 1 -name '*.FAILED' 2>/dev/null | wc -l | tr -d ' ')"
  } > "$ARCHIVE_ROOT/LATEST.txt.new" && mv -f "$ARCHIVE_ROOT/LATEST.txt.new" "$ARCHIVE_ROOT/LATEST.txt"
  # written aside then moved, so a full disk cannot leave a truncated status
  # file behind a successful run
fi

say ""
if [ "$DRY" -eq 1 ]; then
  say "DRY RUN -- nothing was written"
else
  say "archive OK  ($SNAP_RESULT; dataset $MIRROR_RESULT)"
fi
