#!/bin/bash
# Run this ON THE MAC during a test. It copies the ObieApp Data Folder's runs to the
# arm's computer once a second, so follow_obie.py / plan_stencil.py can read them there.
#
#   ./sync_to_arm.sh ~/path/to/DataFolder croger@<arm-host>
#   ./sync_to_arm.sh ~/path/to/DataFolder croger@<arm-host> obie_runs   # folder on the arm (default)
#
# Copies template.json and TRF/ for every run; skips the per-hit WAVs (raw/), the
# .avc/.avr result files and ObieAppSettings/. Only reads the Data Folder. The copy
# on the arm mirrors it exactly (--delete), so Start Over etc. show up there too.
# One ssh connection is reused for every pass, so you log in once. Ctrl-C to stop.
#
# Get this file onto the Mac with, for example:
#   scp croger@<arm-host>:anne/impact_testing/sync_to_arm.sh ~/ && chmod +x ~/sync_to_arm.sh

DATA="${1%/}"
ARM="$2"
DEST="${3:-obie_runs}"
INTERVAL=1   # seconds between passes

if [ -z "$DATA" ] || [ -z "$ARM" ]; then
    sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
fi
if [ ! -d "$DATA" ] || [ -z "$(ls -A "$DATA")" ]; then
    echo "No Data Folder at $DATA (or it is empty); not syncing, so the arm's copy isn't wiped."
    exit 1
fi

SOCKET="$HOME/.ssh/obie-sync-%r@%h:%p"
SSH="ssh -o ControlMaster=auto -o ControlPath=$SOCKET -o ControlPersist=10m"
trap '$SSH -O exit "$ARM" 2>/dev/null; echo; echo "stopped"; exit 0' INT TERM

$SSH "$ARM" "mkdir -p '$DEST'" || exit 1
echo "Syncing $DATA -> $ARM:$DEST every ${INTERVAL}s. Ctrl-C to stop."
fails=0
while true; do
    if rsync -a --delete -e "$SSH" \
            --exclude 'raw/' --exclude 'ObieAppSettings/' --exclude '*.avc' --exclude '*.avr' \
            "$DATA/" "$ARM:$DEST/"; then
        [ $fails -gt 0 ] && echo "$(date +%T) syncing again"
        fails=0
    else
        fails=$((fails + 1))
        [ $fails -eq 1 ] && echo "$(date +%T) sync failed; retrying every ${INTERVAL}s"
    fi
    sleep $INTERVAL
done
