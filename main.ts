// Push-optimal Sokoban solver for make.fun competition sokoban-solver-01-run-20261121.
// Strategy: decompose the level into rooms off a central hallway, solve each room
// push-optimally with A* over push states, then expand pushes to full player walks.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// ---------------------------------------------------------------------------
// Parsing + grid helpers
// ---------------------------------------------------------------------------

interface Level {
  rows: number;
  cols: number;
  wall: Uint8Array; // 1 if wall
  goal: Uint8Array; // 1 if goal
  boxes: number[]; // cell indices of boxes
  player: number; // player start cell
}

function parseLevel(text: string): Level {
  const lines = text.split(/\r?\n/).filter((l) => l.length > 0);
  const rows = lines.length;
  const cols = Math.max(...lines.map((l) => l.length));
  const wall = new Uint8Array(rows * cols);
  const goal = new Uint8Array(rows * cols);
  const boxes: number[] = [];
  let player = -1;
  for (let r = 0; r < rows; r++) {
    const line = lines[r];
    for (let c = 0; c < cols; c++) {
      const ch = c < line.length ? line[c] : ' ';
      const idx = r * cols + c;
      if (ch === '#') wall[idx] = 1;
      else if (ch === '.') goal[idx] = 1;
      else if (ch === '$') boxes.push(idx);
      else if (ch === '*') { boxes.push(idx); goal[idx] = 1; }
      else if (ch === '@') player = idx;
      else if (ch === '+') { player = idx; goal[idx] = 1; }
    }
  }
  return { rows, cols, wall, goal, boxes, player };
}

function cell(r: number, c: number, cols: number): number { return r * cols + c; }

function rowOf(idx: number, cols: number): number { return Math.floor(idx / cols); }
function colOf(idx: number, cols: number): number { return idx % cols; }

// BFS over walk-space (non-wall, non-box cells). Returns dist array (Infinity for unreachable).
// walkMask: Uint8Array, 1 if the cell is in the walk-space (non-wall). boxMask: Uint8Array, 1 if box.
function bfsDist(start: number, walkMask: Uint8Array, boxMask: Uint8Array, rows: number, cols: number): Float64Array {
  const n = rows * cols;
  const dist = new Float64Array(n);
  dist.fill(Infinity);
  if (walkMask[start] === 0 || boxMask[start] === 1) return dist;
  dist[start] = 0;
  const queue = new Int32Array(n);
  let head = 0, tail = 0;
  queue[tail++] = start;
  while (head < tail) {
    const cur = queue[head++];
    const r = rowOf(cur, cols), c = colOf(cur, cols);
    const d = dist[cur] + 1;
    // neighbour order u, d, l, r
    if (r > 0) {
      const nb = cur - cols;
      if (walkMask[nb] === 1 && boxMask[nb] === 0 && dist[nb] === Infinity) { dist[nb] = d; queue[tail++] = nb; }
    }
    if (r < rows - 1) {
      const nb = cur + cols;
      if (walkMask[nb] === 1 && boxMask[nb] === 0 && dist[nb] === Infinity) { dist[nb] = d; queue[tail++] = nb; }
    }
    if (c > 0) {
      const nb = cur - 1;
      if (walkMask[nb] === 1 && boxMask[nb] === 0 && dist[nb] === Infinity) { dist[nb] = d; queue[tail++] = nb; }
    }
    if (c < cols - 1) {
      const nb = cur + 1;
      if (walkMask[nb] === 1 && boxMask[nb] === 0 && dist[nb] === Infinity) { dist[nb] = d; queue[tail++] = nb; }
    }
  }
  return dist;
}

// BFS that also returns a path (list of cells) from start to target. Returns null if unreachable.
// walkMask: non-wall cells; boxMask: box cells (obstacles).
function bfsPath(start: number, target: number, walkMask: Uint8Array, boxMask: Uint8Array, rows: number, cols: number): number[] | null {
  const n = rows * cols;
  const prev = new Int32Array(n);
  prev.fill(-2);
  if (walkMask[start] === 0 || boxMask[start] === 1) return null;
  prev[start] = -1;
  const queue = new Int32Array(n);
  let head = 0, tail = 0;
  queue[tail++] = start;
  while (head < tail) {
    const cur = queue[head++];
    if (cur === target) break;
    const r = rowOf(cur, cols), c = colOf(cur, cols);
    // u, d, l, r
    if (r > 0) {
      const nb = cur - cols;
      if (walkMask[nb] === 1 && boxMask[nb] === 0 && prev[nb] === -2) { prev[nb] = cur; queue[tail++] = nb; }
    }
    if (r < rows - 1) {
      const nb = cur + cols;
      if (walkMask[nb] === 1 && boxMask[nb] === 0 && prev[nb] === -2) { prev[nb] = cur; queue[tail++] = nb; }
    }
    if (c > 0) {
      const nb = cur - 1;
      if (walkMask[nb] === 1 && boxMask[nb] === 0 && prev[nb] === -2) { prev[nb] = cur; queue[tail++] = nb; }
    }
    if (c < cols - 1) {
      const nb = cur + 1;
      if (walkMask[nb] === 1 && boxMask[nb] === 0 && prev[nb] === -2) { prev[nb] = cur; queue[tail++] = nb; }
    }
  }
  if (prev[target] === -2) return null;
  const path: number[] = [];
  let cur = target;
  while (cur !== -1) { path.push(cur); cur = prev[cur]; }
  path.reverse();
  return path;
}// ---------------------------------------------------------------------------
// Hallway + room decomposition
// ---------------------------------------------------------------------------

interface Room {
  cells: Set<number>; // room cells (strictly one side of hallway)
  door: number; // door cell (in room side, adjacent to hallway)
  entry: number; // hallway cell adjacent to door
  boxes: number[]; // box cells in this room
  goals: number[]; // goal cells in this room
  walkMask: Uint8Array; // room cells + door + entry (non-wall)
  side: number; // -1 above, +1 below
}

// Find the hallway row: the row containing the player with the longest run of non-wall cells.
function findHallwayRow(level: Level): number {
  const pr = rowOf(level.player, level.cols);
  // longest run of non-wall cells in the player's row
  let bestRow = -1, bestLen = -1;
  const r = pr;
  let run = 0;
  for (let c = 0; c < level.cols; c++) {
    if (level.wall[cell(r, c, level.cols)] === 0) run++;
    else { if (run > bestLen) { bestLen = run; bestRow = r; } run = 0; }
  }
  if (run > bestLen) { bestLen = run; bestRow = r; }
  return bestRow;
}

// Find the hallway column (fallback if no hallway row).
function findHallwayCol(level: Level): number {
  const pc = colOf(level.player, level.cols);
  let bestCol = -1, bestLen = -1;
  const c = pc;
  let run = 0;
  for (let r = 0; r < level.rows; r++) {
    if (level.wall[cell(r, c, level.cols)] === 0) run++;
    else { if (run > bestLen) { bestLen = run; bestCol = c; } run = 0; }
  }
  if (run > bestLen) { bestLen = run; bestCol = c; }
  return bestCol;
}

// Flood-fill from door restricted to one side of the hallway row.
function floodRoom(level: Level, door: number, hallwayRow: number, side: number): Set<number> {
  const cells = new Set<number>();
  const stack = [door];
  cells.add(door);
  while (stack.length > 0) {
    const cur = stack.pop()!;
    const r = rowOf(cur, level.cols), c = colOf(cur, level.cols);
    const nbs = [
      [r - 1, c], [r + 1, c], [r, c - 1], [r, c + 1],
    ];
    for (const [nr, nc] of nbs) {
      if (nr < 0 || nr >= level.rows || nc < 0 || nc >= level.cols) continue;
      if (side < 0 && nr >= hallwayRow) continue; // above room: rows strictly above hallway
      if (side > 0 && nr <= hallwayRow) continue; // below room: rows strictly below hallway
      const idx = cell(nr, nc, level.cols);
      if (level.wall[idx] === 1) continue;
      if (cells.has(idx)) continue;
      cells.add(idx);
      stack.push(idx);
    }
  }
  return cells;
}

// Decompose the level into rooms. Returns null if decomposition fails.
function decompose(level: Level): Room[] | null {
  const hallwayRow = findHallwayRow(level);
  let hallwayCol = -1;
  let horizontal = true;
  if (hallwayRow < 0) {
    hallwayCol = findHallwayCol(level);
    if (hallwayCol < 0) return null;
    horizontal = false;
  }
  const rooms: Room[] = [];
  if (horizontal) {
    const hr = hallwayRow;
    for (let c = 0; c < level.cols; c++) {
      // check above and below
      for (const side of [-1, 1]) {
        const rr = hr + side;
        if (rr < 0 || rr >= level.rows) continue;
        const door = cell(rr, c, level.cols);
        if (level.wall[door] === 1) continue;
        // door cell: could be floor, goal, box, box-on-goal
        const roomCells = floodRoom(level, door, hr, side);
        // collect boxes and goals in room cells
        const boxes: number[] = [];
        const goals: number[] = [];
        for (const idx of roomCells) {
          if (level.boxes.includes(idx)) boxes.push(idx);
          if (level.goal[idx] === 1) goals.push(idx);
        }
        if (boxes.length !== goals.length) return null;
        const entry = cell(hr, c, level.cols);
        const walkMask = new Uint8Array(level.rows * level.cols);
        for (const idx of roomCells) walkMask[idx] = 1;
        walkMask[door] = 1;
        walkMask[entry] = 1;
        rooms.push({ cells: roomCells, door, entry, boxes, goals, walkMask, side });
      }
    }
    // sort rooms by increasing door column
    rooms.sort((a, b) => colOf(a.door, level.cols) - colOf(b.door, level.cols));
  } else {
    // vertical hallway fallback: treat whole level as one room
    return null;
  }
  if (rooms.length === 0) return null;
  return rooms;
}// ---------------------------------------------------------------------------
// Dead-square table + assignment heuristic + A* room solver
// ---------------------------------------------------------------------------

// Precompute LIVE cells by reverse reachability from goals within a walk-space.
// A cell c becomes live if there is a direction d with c+d live AND c-d a non-wall walk-space cell.
function computeLive(walkMask: Uint8Array, goals: number[], rows: number, cols: number): Uint8Array {
  const n = rows * cols;
  const live = new Uint8Array(n);
  const stack: number[] = [];
  for (const g of goals) { live[g] = 1; stack.push(g); }
  while (stack.length > 0) {
    const c = stack.pop()!;
    const r = rowOf(c, cols), col = colOf(c, cols);
    // c is live. A cell x can push a box toward c in direction d if x+d = c,
    // i.e. x = c-d, and the player can stand at x-d = c-2d (walkable).
    const dirs = [
      [-1, 0], [1, 0], [0, -1], [0, 1],
    ];
    for (const [dr, dc] of dirs) {
      // x = c - d
      const xr = r - dr, xc = col - dc;
      if (xr < 0 || xr >= rows || xc < 0 || xc >= cols) continue;
      const x = cell(xr, xc, cols);
      if (live[x] === 1) continue;
      if (walkMask[x] === 0) continue;
      // player stand cell = x - d = c - 2d
      const sr = r - 2 * dr, sc = col - 2 * dc;
      if (sr < 0 || sr >= rows || sc < 0 || sc >= cols) continue;
      const stand = cell(sr, sc, cols);
      if (walkMask[stand] === 0) continue;
      live[x] = 1;
      stack.push(x);
    }
  }
  return live;
}

// Precompute BFS distances between all pairs of cells in the walk-space (ignoring boxes).
function computeDistances(walkMask: Uint8Array, rows: number, cols: number): Float64Array {
  const n = rows * cols;
  const dist = new Float64Array(n * n);
  dist.fill(Infinity);
  const boxMask = new Uint8Array(n);
  for (let s = 0; s < n; s++) {
    if (walkMask[s] === 0) continue;
    const d = bfsDist(s, walkMask, boxMask, rows, cols);
    for (let t = 0; t < n; t++) dist[s * n + t] = d[t];
  }
  return dist;
}

// Minimum-cost assignment of boxes to goals (brute force permutations, max 4 boxes).
function assignmentHeuristic(boxes: number[], goals: number[], dist: Float64Array, n: number): number {
  const nb = boxes.length;
  if (nb === 0) return 0;
  // build cost matrix
  const cost: number[][] = [];
  for (let i = 0; i < nb; i++) {
    const row: number[] = [];
    for (let j = 0; j < nb; j++) {
      const d = dist[boxes[i] * n + goals[j]];
      row.push(d);
    }
    cost.push(row);
  }
  // brute force permutations
  const perm = new Array(nb).fill(0).map((_, i) => i);
  let best = Infinity;
  const used = new Array(nb).fill(false);
  const cur = new Array(nb).fill(0);
  function rec(pos: number): void {
    if (pos === nb) {
      let sum = 0;
      for (let i = 0; i < nb; i++) {
        const d = cost[i][cur[i]];
        if (d === Infinity) { sum = Infinity; break; }
        sum += d;
      }
      if (sum < best) best = sum;
      return;
    }
    for (let j = 0; j < nb; j++) {
      if (used[j]) continue;
      if (cost[pos][j] === Infinity) continue;
      used[j] = true;
      cur[pos] = j;
      rec(pos + 1);
      used[j] = false;
    }
  }
  rec(0);
  return best;
}// ---------------------------------------------------------------------------
// A* room solver
// ---------------------------------------------------------------------------

// Binary min-heap for A* open set.
class MinHeap {
  f: number[] = [];
  g: number[] = [];
  state: number[] = []; // state index
  size = 0;
  push(f: number, g: number, s: number): void {
    let i = this.size++;
    this.f[i] = f; this.g[i] = g; this.state[i] = s;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.f[p] <= this.f[i]) break;
      this.swap(i, p);
      i = p;
    }
  }
  pop(): [number, number, number] | null {
    if (this.size === 0) return null;
    const top: [number, number, number] = [this.f[0], this.g[0], this.state[0]];
    this.size--;
    if (this.size > 0) {
      this.f[0] = this.f[this.size]; this.g[0] = this.g[this.size]; this.state[0] = this.state[this.size];
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = 2 * i + 2;
        let m = i;
        if (l < this.size && this.f[l] < this.f[m]) m = l;
        if (r < this.size && this.f[r] < this.f[m]) m = r;
        if (m === i) break;
        this.swap(i, m);
        i = m;
      }
    }
    return top;
  }
  private swap(a: number, b: number): void {
    let t = this.f[a]; this.f[a] = this.f[b]; this.f[b] = t;
    t = this.g[a]; this.g[a] = this.g[b]; this.g[b] = t;
    t = this.state[a]; this.state[a] = this.state[b]; this.state[b] = t;
  }
}

// Canonicalize player: smallest cell index of player's reachable region.
function canonicalPlayer(player: number, walkMask: Uint8Array, boxMask: Uint8Array, rows: number, cols: number): number {
  const n = rows * cols;
  const seen = new Uint8Array(n);
  const stack = [player];
  seen[player] = 1;
  let min = player;
  while (stack.length > 0) {
    const cur = stack.pop()!;
    if (cur < min) min = cur;
    const r = rowOf(cur, cols), c = colOf(cur, cols);
    if (r > 0) { const nb = cur - cols; if (walkMask[nb] === 1 && boxMask[nb] === 0 && seen[nb] === 0) { seen[nb] = 1; stack.push(nb); } }
    if (r < rows - 1) { const nb = cur + cols; if (walkMask[nb] === 1 && boxMask[nb] === 0 && seen[nb] === 0) { seen[nb] = 1; stack.push(nb); } }
    if (c > 0) { const nb = cur - 1; if (walkMask[nb] === 1 && boxMask[nb] === 0 && seen[nb] === 0) { seen[nb] = 1; stack.push(nb); } }
    if (c < cols - 1) { const nb = cur + 1; if (walkMask[nb] === 1 && boxMask[nb] === 0 && seen[nb] === 0) { seen[nb] = 1; stack.push(nb); } }
  }
  return min;
}

// Check if a 2x2 block of all walls/boxes exists holding at least one box not on a goal.
function has2x2Freeze(boxes: number[], boxSet: Set<number>, wall: Uint8Array, goal: Uint8Array, rows: number, cols: number): boolean {
  for (const b of boxes) {
    if (goal[b] === 1) continue;
    const r = rowOf(b, cols), c = colOf(b, cols);
    // check the 4 possible 2x2 blocks containing b
    for (const [dr, dc] of [[0, 0], [-1, 0], [0, -1], [-1, -1]]) {
      const r0 = r + dr, c0 = c + dc;
      if (r0 < 0 || r0 + 1 >= rows || c0 < 0 || c0 + 1 >= cols) continue;
      let allBlocked = true;
      for (let rr = r0; rr <= r0 + 1; rr++) {
        for (let cc = c0; cc <= c0 + 1; cc++) {
          const idx = cell(rr, cc, cols);
          if (wall[idx] === 1 || boxSet.has(idx)) continue;
          allBlocked = false;
          break;
        }
        if (!allBlocked) break;
      }
      if (allBlocked) return true;
    }
  }
  return false;
}// ---------------------------------------------------------------------------
// A* room solver (push-optimal)
// ---------------------------------------------------------------------------

// State encoding: sorted box positions + canonical player position.
// We store states in a Map keyed by a string "box1,box2,...|player".
// Returns the push sequence (list of {box, dir}) or null if unsolved.
function solveRoom(
  room: Room,
  level: Level,
  startPlayer: number,
  cap: number,
  greedy: boolean,
): { pushes: { box: number; dir: number }[]; finalBoxes: number[]; finalPlayer: number } | null {
  const rows = level.rows, cols = level.cols, n = rows * cols;
  const walkMask = room.walkMask;
  const goals = room.goals;
  const goalSet = new Set(goals);
  const wall = level.wall;
  const goalArr = level.goal;

  // Precompute live cells and distances
  const live = computeLive(walkMask, goals, rows, cols);
  const dist = computeDistances(walkMask, rows, cols);

  // initial box positions (sorted)
  let boxes = [...room.boxes].sort((a, b) => a - b);
  // initial player canonical
  const boxMask0 = new Uint8Array(n);
  for (const b of boxes) boxMask0[b] = 1;
  const player0 = canonicalPlayer(startPlayer, walkMask, boxMask0, rows, cols);

  // state key
  const keyOf = (boxes: number[], player: number): string => boxes.join(',') + '|' + player;

  const startKey = keyOf(boxes, player0);
  const gScore = new Map<string, number>();
  gScore.set(startKey, 0);
  const cameFrom = new Map<string, { prev: string; box: number; dir: number }>();
  const heap = new MinHeap();
  const h0 = assignmentHeuristic(boxes, goals, dist, n);
  if (h0 === Infinity) return null;
  heap.push(h0, 0, 0); // state index 0 = start
  const stateList: { boxes: number[]; player: number }[] = [{ boxes, player: player0 }];
  const stateKeyToIdx = new Map<string, number>();
  stateKeyToIdx.set(startKey, 0);

  let expansions = 0;
  while (heap.size > 0) {
    const top = heap.pop()!;
    const f = top[0], g = top[1], sidx = top[2];
    const st = stateList[sidx];
    const curBoxes = st.boxes;
    const curPlayer = st.player;
    const curKey = keyOf(curBoxes, curPlayer);
    if (gScore.get(curKey) !== g) continue; // stale entry
    expansions++;
    if (expansions > cap) return null;

    // goal test: all boxes on goals AND player can reach entry cell
    let allOnGoal = true;
    for (const b of curBoxes) if (goalArr[b] !== 1) { allOnGoal = false; break; }
    if (allOnGoal) {
      // check player can reach entry cell with final box positions as obstacles
      const boxMaskF = new Uint8Array(n);
      for (const b of curBoxes) boxMaskF[b] = 1;
      const reach = bfsDist(curPlayer, walkMask, boxMaskF, rows, cols);
      if (reach[room.entry] !== Infinity) {
        // reconstruct push sequence
        const pushes: { box: number; dir: number }[] = [];
        let k = curKey;
        while (k !== startKey) {
          const cf = cameFrom.get(k)!;
          pushes.push({ box: cf.box, dir: cf.dir });
          k = cf.prev;
        }
        pushes.reverse();
        return { pushes, finalBoxes: curBoxes, finalPlayer: curPlayer };
      }
    }

    // generate moves
    const boxSet = new Set(curBoxes);
    const boxMask = new Uint8Array(n);
    for (const b of curBoxes) boxMask[b] = 1;
    // player reachable region
    const reach = bfsDist(curPlayer, walkMask, boxMask, rows, cols);
    const dirs = [[-1, 0], [1, 0], [0, -1], [0, 1]];
    const dirIdx = [0, 1, 2, 3]; // u,d,l,r
    for (let bi = 0; bi < curBoxes.length; bi++) {
      const b = curBoxes[bi];
      const br = rowOf(b, cols), bc = colOf(b, cols);
      for (let di = 0; di < 4; di++) {
        const [dr, dc] = dirs[di];
        const pushCell = cell(br - dr, bc - dc, cols);
        const dest = cell(br + dr, bc + dc, cols);
        // pushCell must be in player's reachable region
        if (reach[pushCell] === Infinity) continue;
        // dest must be a walk-space cell (room/door/entry) not wall and no box
        if (walkMask[dest] !== 1) continue;
        if (boxSet.has(dest)) continue;
        // new box positions
        const newBoxes = [...curBoxes];
        newBoxes[bi] = dest;
        newBoxes.sort((a, b) => a - b);
        // dead-square pruning
        let dead = false;
        for (const nb of newBoxes) {
          if (goalArr[nb] !== 1 && live[nb] !== 1) { dead = true; break; }
        }
        if (dead) continue;
        // 2x2 freeze pruning
        const newBoxSet = new Set(newBoxes);
        if (has2x2Freeze(newBoxes, newBoxSet, wall, goalArr, rows, cols)) continue;
        // new player = b (where box was), canonicalize
        const newBoxMask = new Uint8Array(n);
        for (const nb of newBoxes) newBoxMask[nb] = 1;
        const newPlayer = canonicalPlayer(b, walkMask, newBoxMask, rows, cols);
        const newKey = keyOf(newBoxes, newPlayer);
        const newG = g + 1;
        const oldG = gScore.get(newKey);
        if (oldG === undefined || newG < oldG) {
          gScore.set(newKey, newG);
          cameFrom.set(newKey, { prev: curKey, box: b, dir: di });
          let sidx2 = stateKeyToIdx.get(newKey);
          if (sidx2 === undefined) {
            sidx2 = stateList.length;
            stateList.push({ boxes: newBoxes, player: newPlayer });
            stateKeyToIdx.set(newKey, sidx2);
          }
          const h = assignmentHeuristic(newBoxes, goals, dist, n);
          if (h === Infinity) continue;
          const f2 = greedy ? h : newG + h;
          heap.push(f2, newG, sidx2);
        }
      }
    }
  }
  return null;
}// ---------------------------------------------------------------------------
// Push-to-walk expansion + whole-level driver
// ---------------------------------------------------------------------------

// Convert a cell difference to a move letter.

// dir: 0=u,1=d,2=l,3=r. pushCell = where player stands (b - dir). dest = where box goes (b + dir).
function pushCellOf(b: number, dir: number, cols: number): number {
  const r = rowOf(b, cols), c = colOf(b, cols);
  if (dir === 0) return cell(r + 1, c, cols); // push up: stand below
  if (dir === 1) return cell(r - 1, c, cols); // push down: stand above
  if (dir === 2) return cell(r, c + 1, cols); // push left: stand right
  return cell(r, c - 1, cols); // push right: stand left
}
function destOf(b: number, dir: number, cols: number): number {
  const r = rowOf(b, cols), c = colOf(b, cols);
  if (dir === 0) return cell(r - 1, c, cols);
  if (dir === 1) return cell(r + 1, c, cols);
  if (dir === 2) return cell(r, c - 1, cols);
  return cell(r, c + 1, cols);
}

function moveLetter(from: number, to: number, cols: number): string {
  const dr = rowOf(to, cols) - rowOf(from, cols);
  const dc = colOf(to, cols) - colOf(from, cols);
  if (dr === -1) return 'u';
  if (dr === 1) return 'd';
  if (dc === -1) return 'l';
  if (dc === 1) return 'r';
  return '';
}

// BFS walk from start to target over walkMask with boxMask obstacles; returns move string or null.
function walkMoves(start: number, target: number, walkMask: Uint8Array, boxMask: Uint8Array, rows: number, cols: number): string | null {
  const path = bfsPath(start, target, walkMask, boxMask, rows, cols);
  if (path === null) return null;
  let s = '';
  for (let i = 1; i < path.length; i++) s += moveLetter(path[i - 1], path[i], cols);
  return s;
}

// Solve the whole level. Returns the full move string.
function solveLevel(level: Level): string {
  const rooms = decompose(level);
  if (rooms === null) {
    // fallback: whole level as one room
    return solveWholeLevel(level);
  }
  const rows = level.rows, cols = level.cols, n = rows * cols;
  // whole-level walk mask (all non-wall cells)
  const wholeWalk = new Uint8Array(n);
  for (let i = 0; i < n; i++) if (level.wall[i] === 0) wholeWalk[i] = 1;
  // current box positions across whole level
  let boxes = [...level.boxes];
  let player = level.player;
  let moves = '';
  const CAP = 5000000;
  for (const room of rooms) {
    // walk player to room's entry cell over whole level with current boxes as obstacles
    const boxMask = new Uint8Array(n);
    for (const b of boxes) boxMask[b] = 1;
    const toEntry = walkMoves(player, room.entry, wholeWalk, boxMask, rows, cols);
    if (toEntry === null) {
      // cannot reach entry; skip this room (emit nothing for it)
      continue;
    }
    moves += toEntry;
    player = room.entry;
    // walk into the door cell if possible (door may hold a box; then stay at entry)
    const toDoor = walkMoves(player, room.door, wholeWalk, boxMask, rows, cols);
    if (toDoor !== null) {
      moves += toDoor;
      player = room.door;
    }
    // solve the room push-optimally
    let result = solveRoom(room, level, player, CAP, false);
    if (result === null) {
      // fall back to greedy best-first
      result = solveRoom(room, level, player, CAP, true);
    }
    if (result === null) {
      // room unsolved; emit nothing for it
      continue;
    }
    // expand pushes to walks within the room's walk-space
    const roomWalk = room.walkMask;
    let curPlayer = player;
    for (const p of result.pushes) {
      const boxMaskR = new Uint8Array(n);
      for (const b of boxes) boxMaskR[b] = 1;
      const pushCell = pushCellOf(p.box, p.dir, cols);
      const walk = walkMoves(curPlayer, pushCell, roomWalk, boxMaskR, rows, cols);
      if (walk === null) { moves = ''; break; }
      moves += walk;
      curPlayer = pushCell;
      moves += moveLetter(pushCell, p.box, cols); // push letter
      // update box positions
      const bi = boxes.indexOf(p.box);
      if (bi >= 0) {
        boxes[bi] = destOf(p.box, p.dir, cols);
      }
      curPlayer = p.box;
    }
    if (moves === '') break; // walk failed, abort
    player = curPlayer;
  }
  return moves;
}

// Fallback: solve whole level as one room.
function solveWholeLevel(level: Level): string {
  const rows = level.rows, cols = level.cols, n = rows * cols;
  const walkMask = new Uint8Array(n);
  for (let i = 0; i < n; i++) if (level.wall[i] === 0) walkMask[i] = 1;
  const goals: number[] = [];
  for (let i = 0; i < n; i++) if (level.goal[i] === 1) goals.push(i);
  const room: Room = {
    cells: new Set(),
    door: -1,
    entry: -1,
    boxes: [...level.boxes],
    goals,
    walkMask,
    side: 0,
  };
  const result = solveRoom(room, level, level.player, 5000000, false);
  if (result === null) return '';
  // expand pushes to walks
  let moves = '';
  let curPlayer = level.player;
  let boxes = [...level.boxes];
  for (const p of result.pushes) {
    const boxMask = new Uint8Array(n);
    for (const b of boxes) boxMask[b] = 1;
    const pushCell = pushCellOf(p.box, p.dir, cols);
    const walk = walkMoves(curPlayer, pushCell, walkMask, boxMask, rows, cols);
    if (walk === null) return '';
    moves += walk;
    curPlayer = pushCell;
    moves += moveLetter(pushCell, p.box, cols);
    const bi = boxes.indexOf(p.box);
    if (bi >= 0) {
      boxes[bi] = destOf(p.box, p.dir, cols);
    }
    curPlayer = p.box;
  }
  return moves;
}// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

function solve(levelText: string): string {
  try {
    const level = parseLevel(levelText);
    return solveLevel(level);
  } catch {
    return '';
  }
}

if (import.meta.url === pathToFileURL(process.argv[1]).href) {
  for (const id of readdirSync(I).sort()) {
    let moves = '';
    try {
      const levelText = readFileSync(join(I, id, 'level.txt'), 'utf8');
      moves = solve(levelText);
    } catch {
      moves = '';
    }
    mkdirSync(join(O, id), { recursive: true });
    writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
  }
}