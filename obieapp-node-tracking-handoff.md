# ObieApp node tracking: how the arm can follow along

**Goal:** the UR arm code needs to know which stencil node to strike next during an ObieApp modal-analysis run. ObieApp must **not** be modified. The arm code reads the same files on disk that ObieApp's Stencil Builder (projector) reads.

## 1. How ObieApp does it today

ObieApp is a browser app (Chrome). Two of its tools take part, and they never talk to each other directly. **The Data Folder on disk is the only shared channel.**

```
Acquire (browser tab)                Stencil Builder (browser tab)          Projector popup
records hammer hits ──writes──▶  <run>/TRF/*.trf  ──polls every 1 s──▶  computes current node ──postMessage──▶ draws nodes
                                 <run>/template.json
```

- **Acquire** rewrites one `.trf` file per position after **every** hit. The file contains a running hit count.
- **Stencil Builder** "Watch" mode lists the run's `TRF/` folder once a second, reads each file's hit count and works out which node is current and which are done.
- The **projector window** gets that state through `window.postMessage`. This is browser-internal, with no network, websocket or server. External code can't listen to it.

**So the arm should copy the file polling, not the projector messages.** The polling lives in the Stencil Builder's `_pollTRF()` (`Web/tools/template/template.js`). Section 4 below is a Python equivalent.

## 2. Folder layout

```
<DataFolder>/
├── ObieAppSettings/                  ← ignore
└── <Instrument>/                     ← e.g. "Violin7"
    └── <TestName>/                   ← one run, e.g. "Test 3"
        ├── template.json             ← run settings + stencil (node layout)
        ├── raw/                      ← per-hit WAVs (ignore)
        ├── TRF/
        │   ├── <TestName> H01.trf    ← position 1
        │   ├── <TestName> H02.trf    ← position 2
        │   └── …
        └── <TestName> H.avc / .avr   ← written at end of run
```

A folder counts as an Acquire run if it has a `TRF/` subfolder.

## 3. Data formats

### template.json (in the run folder)

Acquire writes it when the run is set up. The Stencil Builder merges a `stencil` key into it when Watch starts. The fields that matter:

```jsonc
{
  "taps": 5,            // hits required per position
  "positions": 12,      // positions per prefix group
  "prefix": "H",        // may be a list such as "H,V" (each prefix is uppercased, max 3 chars)
  "sample_rate": 48000,
  "stencil": {
    "name": "Violin top 4x6",
    "type": "node-stencil",
    "grid":  { "rows": 4, "cols": 6, "xSpacing": 25, "ySpacing": 20 },
    "nodes": [
      { "id": 0, "label": "1", "row": 0, "col": 0, "xMm": -62.5, "yMm": -30 },
      …
    ]
  }
}
```

- `nodes[i].xMm` / `yMm` are **physical millimetres**, with the origin at the grid centre. These are the coordinates the arm needs. They are not projector pixels: the projector's scale, rotation and pan are display calibration only and are not stored here.
- Node `id` = position number − 1. TRF position 1 → node id 0, and so on. The default grid assigns ids in row-major order. Nodes can be dragged, but ids stay as assigned.
- `stencil` may be missing if no stencil was applied in Acquire and Watch has not been started in the Stencil Builder. Only `taps` may be missing in older runs.

### TRF filename → position number

The current Acquire naming is **`<TestName> <PREFIX><NN>.trf`**, for example `Test 3 H07.trf`. `NN` is the 1-based, zero-padded index **within that prefix group**. With `prefix: "H,V"` and `positions: 12`:

| File | Global position (1-based) | Node id |
|---|---|---|
| `… H01.trf` … `… H12.trf` | 1 … 12 | 0 … 11 |
| `… V01.trf` … `… V12.trf` | 13 … 24 | 12 … 23 |

global position = (index of prefix in the prefix list) × `positions` + NN

> ⚠️ **The Stencil Builder's own regex is out of date.** It uses `/_(\d+)\.trf$/i`, which matches the older naming (`… H_7.trf`). Acquire switched to `H07` in commit `4825810` (July 2026). That regex does not match current filenames, so the projector Watch may be stuck on node 1 with current runs. Don't copy that regex. Parse the `<PREFIX><NN>` form above. (This is being flagged separately to the ObieApp side; the arm code shouldn't depend on it.)

### TRF file: hit count (`n_hits`)

The binary FRF data is followed by a plain-text metadata block that starts with the 11-byte marker `b'\x00OBIE_META\n'`, then UTF-8 `key: value` lines:

```
sample_rate: 48000
bit_depth: 16
n_hits: 3          ← hits recorded so far at this position
threshold: 0.05
ham_cutoff: 0.050 s
mic_cutoff: 0.300 s
device: …
```

The arm only needs `n_hits`. You don't have to parse the binary part: just find the marker. (The full parser, if you ever want it, is `Python/fileio/trf_fileio.py` in the ObieApp repo.)

## 4. Done / current-node logic (port of `_pollTRF`)

Each tick (1 s is what ObieApp uses):

1. List `TRF/*.trf` and map each filename to its global position.
2. A position is **done** when its own file's `n_hits >= taps`. Once a position is done, cache it so you don't re-read that file.
3. **Current node** = the lowest-numbered position that has a file but isn't done.
4. If every position with a file is done, the current node = the next position (`count of files + 1`), or **finished** if that is past the node count.
5. If there are no files yet, the current node is node id 0.
6. If `taps` is missing, fall back to the old guess: every position below the highest file is done, and the highest is current.

Files can be **mid-write** (Acquire overwrites the whole file after each hit). If the marker is missing or the read fails, treat that position as "not done yet" and try again on the next tick. Never crash on it.

```python
import json, re, time
from pathlib import Path

MARKER = b'\x00OBIE_META\n'
NAME_RE = re.compile(r'\s([A-Z]{1,3})(\d+)\.trf$', re.I)

def read_meta(path):
    try:
        data = path.read_bytes()
    except OSError:
        return {}
    i = data.find(MARKER)
    if i < 0:
        return {}
    meta = {}
    for line in data[i + len(MARKER):].decode('utf-8', 'ignore').splitlines():
        k, sep, v = line.partition(':')
        if sep:
            meta[k.strip()] = v.strip()
    return meta

class RunWatcher:
    def __init__(self, run_dir):
        self.run = Path(run_dir)
        tpl = json.loads((self.run / 'template.json').read_text())
        self.taps = int(tpl['taps']) if str(tpl.get('taps', '')).isdigit() else None
        self.per_group = max(1, int(tpl.get('positions', 1)))
        self.prefixes = [p.strip()[:3].upper() for p in str(tpl.get('prefix', 'H')).split(',') if p.strip()] or ['H']
        self.nodes = (tpl.get('stencil') or {}).get('nodes', [])
        self.confirmed = set()

    def _position(self, name):
        m = NAME_RE.search(name)
        if not m or m.group(1).upper() not in self.prefixes:
            return None
        return self.prefixes.index(m.group(1).upper()) * self.per_group + int(m.group(2))

    def poll(self):
        """Returns (current_node_id or None if finished, set of done node ids)."""
        by_pos = {}
        for f in (self.run / 'TRF').glob('*.trf'):
            p = self._position(f.name)
            if p is not None:
                by_pos[p] = f
        pos = sorted(by_pos)
        if not pos:
            return (0 if self.nodes else None), set()
        if self.taps is None:                       # fallback heuristic
            return pos[-1] - 1, {p - 1 for p in pos[:-1]}
        for p in pos:
            if p not in self.confirmed:
                n = read_meta(by_pos[p]).get('n_hits', '')
                if n.isdigit() and int(n) >= self.taps:
                    self.confirmed.add(p)
        done = {p - 1 for p in pos if p in self.confirmed}
        pending = [p for p in pos if p not in self.confirmed]
        if pending:
            return pending[0] - 1, done
        nxt = len(pos)
        return (nxt if nxt < len(self.nodes) else None), done

# usage
w = RunWatcher('/path/to/DataFolder/Violin7/Test 3')
last = object()
while True:
    cur, done = w.poll()
    if cur != last:
        last = cur
        if cur is None:
            print('run finished'); break
        node = next(n for n in w.nodes if n['id'] == cur)
        print('move arm to', node['label'], node['xMm'], node['yMm'])   # → UR motion
    time.sleep(1)
```

(Step 4 uses `len(pos)` the same way ObieApp does, which assumes positions are recorded in order without gaps.)

## 5. Things the arm code has to handle on its own

- **Getting to the files.** The Data Folder is on the laptop running Chrome. The arm machine needs to read it, for example over an SMB/NFS share or `sshfs`, or with a small script on the laptop that sends the node id to the arm. Network mounts can lag a few seconds and cache directory listings, so polling a local disk is more reliable if you have the choice.
- **Choosing the run.** ObieApp's Watch makes the user pick an instrument and test. The arm needs the same: take the run path as an argument, or pick the newest run folder by modification time. (ObieApp's "Sync" picks the last run alphabetically when the watched TRF folder is empty after Acquire's "Start Over". Modification time is more robust.)
- **"Start Over" / Repeat in Acquire.** Repeat clears and re-records a position, so its `n_hits` drops back down. Because of the `confirmed` cache, a position won't un-finish (ObieApp behaves the same way). Start Over creates a new test folder, so switch runs when the watched `TRF/` folder empties.
- **Mapping mm to robot coordinates.** `xMm` / `yMm` are relative to the stencil's grid centre. The arm needs its own calibration (where the grid origin and axes are in the robot base frame). Nothing in ObieApp provides that.
- **Timing.** The file is written right after each hit is processed, so the arm should wait for `n_hits` to change (or the current node to change) before striking again, not use a fixed delay.
- **Read-only.** Never write into the Data Folder. Acquire and the Stencil Builder both rewrite `template.json` and the TRFs.
