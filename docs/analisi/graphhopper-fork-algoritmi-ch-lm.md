# GIScience GraphHopper fork v4.16.0: routing internals for a Kotlin reimplementation

I read every class you listed. Everything below comes from the source; parameter defaults are the literal values in the code.

**Path root.** All paths are relative to this absolute root:
`ROOT = /private/tmp/claude-501/-Users-francescobuzle-Library-Application-Support-Claude-scratch-workspaces-81b23b6d-2e82-4080-8c3e-daebdc218714-021ec3bc-8eaf-4943-9d44-9d379ba48be5-scratch-2026-10-22-04314d/cb75dc37-cc9c-4039-a86c-72de0ce7cc50/scratchpad/src/gh`
- `C` means `ROOT/core/src/main/java/com/graphhopper`.
- `W` means `ROOT/web-api/src/main/java/com/graphhopper`.

**Fork markers.** Fork changes are marked with `// ORS-GH MOD START/END` (sometimes `ORS GH-MOD`, `ORS-MOD`, `OSM MOD`). The git history is a single squashed commit (`e45cce9`), so the markers are the only way to tell fork code from upstream.

---

## 1. Core algorithms

### SPTEntry (`C/routing/SPTEntry.java`)

**Fields:**
- `int edge`
- `int originalEdge` (ORS)
- `int adjNode`
- `double weight`
- `long time` (ORS, used for TD)
- `SPTEntry parent`

**Behaviour:**
- `compareTo` orders by `weight` only.
- `getWeightOfVisitedPath()` returns `weight`.
- `AStar.AStarEntry` adds `weightOfVisitedPath`. Its `weight` field is the heap key (g + h).
- CH variants (`C/routing/ch/CHEntry.java`, `AStarCHEntry.java`):
  - `CHEntry` adds `incEdge`, the original edge id (needed for edge-based CH).
  - `AStarCHEntry` adds `weightOfVisitedPath`.

**Priority queue and maps:**
- The queue is `java.util.PriorityQueue<SPTEntry>`. A decrease-key is done as `remove(entry)` (O(n)), mutate, then `add`. Entries are lazy-free: there is no "settled" flag.
- The maps are `GHIntObjectHashMap<SPTEntry>`, keyed by traversal id.

**Initial capacities:**
- Dijkstra: `min(max(200, nodes/10), 2000)`.
- Non-CH bidirectional: `min(max(200, nodes/10), 150_000)`.
- CH bidirectional: the same, then capped at 2000.

### Traversal id (`C/routing/util/TraversalMode.java`, `C/util/GHUtility.java`)

- **Node-based:** the id is `adjNode`.
- **Edge-based:** the id is `createEdgeKey(base, adj, edge, reverse)`:
  ```java
  edgeId <<= 1;
  if (reverse) return (nodeA >= nodeB) ? edgeId : edgeId + 1;
  return (nodeA > nodeB) ? edgeId + 1 : edgeId;
  ```
  The backward search therefore keys the same directed edge exactly as the forward search does. That is how the two directions meet.
- Related helpers:
  - `createEdgeKey(edge, rev) = (edge << 1) + (rev ? 1 : 0)`
  - `reverseEdgeKey`: k even → k+1, else k-1
  - `getEdgeFromEdgeKey = key / 2`
- There is no class called EdgeBasedNodeKey. The edge key is the traversal id.

### Edge weight with turn (`GHUtility.calcWeightWithTurnWeightWithAccess(w, edge, reverse, prevOrNextEdge[, edgeEnterTime])`)

1. Access check:
   - For a loop edge, the weight is infinite if it has neither forward nor backward access.
   - Otherwise it is infinite if the access flag for the travel direction is false.
2. `edgeW = w.calcEdgeWeight(edge, reverse, edgeEnterTime)`. The `edgeEnterTime` overload is ORS; non-TD calls pass `-1`.
3. If `prevOrNextEdge` is valid, add the turn weight:
   - `origEdge = reverse ? edge.getOrigEdgeLast() : edge.getOrigEdgeFirst()`
   - reverse: `calcTurnWeight(origEdge, base, prevOrNext)`
   - forward: `calcTurnWeight(prevOrNext, base, origEdge)`
- `calcMillisWithTurnMillis` uses `edge.getEdge()`, not the orig-first/last edge.

**`accept()` (`AbstractRoutingAlgorithm`):**
- Node-based traversal rejects an immediate u-turn (`iter.getEdge() == currEdge.edge`).
- ORS adds `additionalEdgeFilter.accept(iter)`, injected through `setEdgeFilter`.

### Dijkstra (`C/routing/Dijkstra.java`)

**Loop:**
1. Start with `currEdge = SPTEntry(from, 0)`. For node-based traversal, put it in `fromMap`.
2. Each iteration: `visitedNodes++`, then break if `maxVisitedNodes < visitedNodes` or `currEdge.adjNode == to`.
3. Relax every edge:
   - On a new key, create the entry.
   - On a better weight, do remove, update, re-add.
4. Poll the next entry. Stop when the heap is empty.

**Path extraction:**
- `PathExtractor` follows parents, accumulates distance and `calcMillisWithTurnMillis`, reverses the edge list, and sets `weight = entry.weight`.

**ORS additions:**
- `reverseDirection` flag (for isochrones/matrix).
- `originalEdge = EdgeIteratorStateHelper.getOriginalEdge(iter)`, which maps a virtual edge to its real edge.
- The traversal id is still built with `reverse=false` (the code carries a TODO about this).

### AStar (`C/routing/AStar.java`)

- **Heuristic:** the same Dijkstra loop with heap key `g + h(v)`.
  - Default h is `BeelineWeightApproximator` on `DistancePlaneProjection`.
  - The start entry key is `0 + h(from)`.
- **Relaxation:** relax only if the entry is new or `ase.weightOfVisitedPath > tmpWeight`; then recompute h.
- **Stop:** when the polled `adjNode == to`.

### BeelineWeightApproximator (`C/routing/weighting/BeelineWeightApproximator.java`)

- Formula: `approximate(v) = weighting.getMinWeight(dist(v, to)) * epsilon`.
- Defaults:
  - `epsilon = 1`.
  - The class default distance calculator is `DistanceCalcEarth`; the algorithms switch it to `DIST_PLANE`.
  - `getSlack() = 0`.
- `RoutingAlgorithmFactorySimple.getApproximation(prop, ...)` reads:
  - `<algo>.approximation`: `"BeelineSimplification"` (plane, the default) or `"BeelineAccurate"` (earth).
  - `<algo>.epsilon`: default 1.

### BalancedWeightApproximator (`C/routing/weighting/BalancedWeightApproximator.java`), used by all bidirectional A*

- `setFromTo(s, t)`:
  ```java
  rev.setTo(s); fwd.setTo(t);
  fromOffset = 0.5 * fwd.approximate(s);
  toOffset   = 0.5 * rev.approximate(t);
  ```
- `approximate(v, reverse)`:
  ```java
  a = 0.5 * (fwd.approximate(v) - rev.approximate(v));
  return reverse ? fromOffset - a : toOffset + a;
  ```
- `getSlack()` returns the forward approximator's slack.
- ORS adds `getReverseApproximation()`.

### Bidirectional base (`C/routing/AbstractBidirAlgo.java`, `AbstractNonCHBidirAlgo.java`)

**Main loop.** It strictly alternates one forward step and one backward step:
```java
while (!finished() && !isMaxVisitedNodesExceeded()) {
    if (!finishedFrom) finishedFrom = !fillEdgesFrom();
    if (!finishedTo)   finishedTo   = !fillEdgesTo();
}
```

**One step (`fillEdgesFrom`; `fillEdgesTo` is symmetric):**
1. Poll and increment the visited counter.
2. If `fromEntryCanBeSkipped()`, return true without expanding.
3. If `fwdSearchCanBeStopped()`, return false.
4. Otherwise expand all edges, with `bestWeightMapOther = bestWeightMapTo`.

**Meeting point (`updateBestPath`):**
- Look up the same traversal id in the other map.
- Candidate `μ = g_f + g_b`.
- Edge-based: both trees contain the meeting edge, so use `entry = entry.parent` and `μ -= calcEdgeWeight(edge, reverse)` (edge weight without turn cost).
- If `μ < bestWeight`, update the best entries.

**Initialisation (`postInit`):**
- Node-based: call `updateBestPath` on the start entry.
- Edge-based with `from == to` and no edge restrictions: `bestWeight = 0` and both directions finish.
- Then `postInitFrom/To` performs the first expansion. With `fromOutEdge`/`toInEdge` restrictions it filters on `getOrigEdgeFirst() == fromOutEdge` / `getOrigEdgeLast() == toInEdge`. These restrictions are only allowed for edge-based traversal.

**Stop criterion:**
- DijkstraBidirectionRef uses the base `finished()`: `finishedFrom || finishedTo || currFrom.weight + currTo.weight - approximatorOffset >= bestWeight`.
- `approximatorOffset` is an ORS field and is 0 by default.

**Path extraction (`DefaultBidirPathExtractor`):**
1. Forward chain, reversed. ORS also reverses `path.getTimes()`.
2. Meeting-point turn time: `calcTurnMillis(inEdge, viaNode, outEdge)`.
3. Backward chain.

**TD:** `calcPath(from, to, at)` and `calcPaths(..., at)` throw "Cannot use bidirectional algorithm for time-dependent routing" (ORS).

### AStarBidirection (`C/routing/AStarBidirection.java`)

- Heap key is `g + balanced.approximate(v, reverse)`.
- `init()`:
  - `stoppingCriterionOffset = weightApprox.approximate(to, true) + weightApprox.getSlack()`.
  - ORS "ALT patch" (issue #21): if the approximator is an `LMApproximator`, set `approximatorOffset = 2 * lm.getFactor()`.
- `finished()` is overridden: `finishedFrom || finishedTo || currFrom.weight + currTo.weight >= bestWeight + stoppingCriterionOffset`.
- **Consequence:** because of this override, the ORS `approximatorOffset` is never read for AStarBidirection. Grep confirms it is only read in `AbstractBidirAlgo.finished()`, so the patch is effectively dead code.

### TD variants (all ORS)

**TDDijkstra** (`C/routing/TDDijkstra.java`, extends Dijkstra):
- Requires `weighting.isTimeDependent()`.
- `calcPath(from, to, at)`: if `reverseDirection` (arrival mode), swap source and target. Start entry has `time = at`.
- Relaxation weight: `calcWeightWithTurnWeightWithAccess(w, iter, reverseDirection, currEdge.edge, currEdge.time)`.
- Entry time: `time = currEdge.time ± calcEdgeMillis(iter, reverseDirection, currEdge.time)`, with minus when reverse.
- The traversal id does use `reverseDirection`.
- Path extraction: `TDPathExtractor`. Each edge's time is `|Δ entry.time|`; edges are reversed only when not in reverse mode; from/to are swapped back.

**TDAStar** (`C/routing/TDAStar.java`, extends AStar):
- Same scheme.
- `reverse()` flips direction and calls `weightApprox.reverse()`.
- Returns an empty path when `maxVisitedNodes` is exceeded or the heap empties.

**Factory selection (`C/routing/RoutingAlgorithmFactorySimple.java`):**
- `td_dijkstra` and `td_astar`.
- The hint `"arrival"` calls `.reverse()`.
- In `LMRoutingAlgorithmFactory`, `td_astar` uses the LM approximator with epsilon 1.0.

### maxVisitedNodes

- `RouterConfig.maxVisitedNodes` defaults to `Integer.MAX_VALUE` (config key `routing.max_visited_nodes`); the request hint is `max_visited_nodes`.
- The path calculators throw "No path found due to maximum nodes exceeded" when `algo.getVisitedNodes() >= max`.

### AlternativeRoute, plateau method (`C/routing/AlternativeRoute.java`)

**Class defaults:**

| Parameter | Default |
|---|---|
| `maxWeightFactor` | 1.4 |
| `maxExplorationFactor` | 0.8 |
| `maxShareFactor` | 0.6 |
| `minPlateauFactor` | 0.2 |
| `maxPaths` | 2 |

**Factory settings:**
- Flex factory (`RoutingAlgorithmFactorySimple`):
  - `alternative_route.max_paths` = 2
  - `alternative_route.max_weight_factor` = 1.4
  - `alternative_route.max_share_factor` = 0.6
  - `alternative_route.min_plateau_factor` = 0.2
  - `alternative_route.max_exploration_factor` = 1
- LM factory: the same, except `setMaxExplorationFactor(0.6)`.

**Search.** `AlternativeBidirSearch extends AStarBidirection` with `explorationFactor = maxExplorationFactor * 2`:
```java
finished = (finishedFrom && finishedTo) || maxVisitedExceeded || finishedFrom || finishedTo
        || currFrom.weight + currTo.weight > explorationFactor * (bestWeight + stoppingCriterionOffset);
```

**Scoring.** The call is `calcAlternatives(bestPath, maxPaths, maxWeightFactor, 7, maxShareFactor, 0.8, minPlateauFactor, -0.2)`, so:
`sortBy = 7*weight + 0.8*shareWeight + (-0.2)*plateauWeight`.

**Candidate filter.** For each forward-map entry that also exists in the backward map:
1. Edge-based: `toEntry = toEntry.parent`.
2. Node-based: skip if both entries have the same edge.
3. Skip if `weight > 1.4 * best`.
4. Skip if this is the best path.
5. Skip unless the entry is the start of a plateau: the next to-entry along the from-entry's edge must have a different edge.
6. Walk the plateau along `toEntry.parent` while forward entries share the same edge, summing `plateauWeight`. Skip if `plateauWeight <= 0` or `plateauWeight/weight < minPlateauFactor`.
7. Share: walk the parents to the first traversal id contained in the best path's id set, summing `shareWeight`. Accept only if `shareWeight/bestWeight < maxShare`.
8. Keep the list sorted, with the best path first, truncated to `maxPaths`.

**AlternativeRouteCH** (`C/routing/AlternativeRouteCH.java`, via-node method):
- Defaults: `max_weight_factor` 1.25, `max_share_factor` 0.8, `local_optimality_factor` 0.25, `max_paths` 3.
- The search continues until both heap minima are `>= best * 1.25`.
- Candidate v: present in both maps, `g_f + g_b <= best * 1.25`, preliminary share ≤ 0.8.
- Candidates are sorted by `2*(g_f + g_b) + share`.
- For each candidate: route s→v and v→t (separate CH searches), check detour length vs direct length × factor and the share, then run a T-test: route between the nodes `T = 0.5 * 0.25 * detourDistance` metres before and after v; v must lie on that path.
- `AlternativeRouteEdgeCH.java` has the same defaults.

### RoundTripRouting (`C/routing/RoundTripRouting.java`, `C/routing/util/tour/MultiPointTour.java`)

**Parameters:**
- `round_trip.distance` default 10000 m.
- `round_trip.seed` default 0.
- `points = min(20, hint(round_trip.points, 2 + (int)(dist/50000)))`.
- `maxRetries` default 3 (`RouterConfig.maxRoundTripRetries`).

**Point generation:**
- Generated points = `points - 1`.
- Distance per leg: `dist/(points+1)` modified by ±`random.nextDouble()*0.1*d`.
- Heading: `initial + 360*i/points`; the initial heading is `random.nextInt(360)` when it is NaN.
- Each point is placed with `projectCoordinate`. On a failed snap, `distance *= 0.95` and retry.

**Routing:**
- Intermediate legs start/end at `closestEdge.getBaseNode()`.
- Forced algorithm `astarbi` with `astarbi.epsilon = 2`.
- `AvoidEdgesWeighting` (penalty factor 5) multiplies the weight of edges already used by previous legs.
- Not allowed with CH.

---

## 2. Contraction Hierarchies

### Preparation flow (`C/routing/ch/PrepareContractionHierarchies.java`)

`doWork()`:
1. `initFromGraph()`:
   - Build a `CHPreparationGraph` (node- or edge-based).
   - `buildFromGraph`: for every base edge, `wFwd/wBwd = calcEdgeWeightWithAccess`; edges with both weights infinite are skipped; weights are stored as **float**.
   - Create the contractor.
   - `sortedNodes = MinHeapWithUpdate(nodes)`, a float-priority binary min-heap with id positions; lowest priority is contracted first.
2. `setMaxLevelOnAllNodes`: `level = nodes` for every node. A node is contracted iff `level != maxLevel`.
3. `contractNodesUsingHeuristicNodeOrdering()`, unless a fixed `NodeOrderingProvider` is given (then level = index i).

**Default parameters:**

| Parameter | Config key | Node-based | Edge-based |
|---|---|---|---|
| Periodic updates | `prepare.ch.updates.periodic` | 20 % | 0 (off) |
| Lazy updates (last nodes) | `prepare.ch.updates.lazy` | 10 % | 100 % |
| Neighbor updates | `prepare.ch.updates.neighbor` | 20 % | 0 |
| Contracted nodes | `prepare.ch.contracted_nodes` | 100 % | 100 % |
| Log messages | `prepare.ch.log_messages` | 20 % | 5 % |

**Heuristic contraction loop:**
1. Compute the initial priority of all nodes. `updatePrioritiesOfRemainingNodes` skips nodes where `doNotContract(node)` (an ORS hook, default `isContracted`).
2. `nodeContractor.prepareContraction()`.
3. Derived counts:
   - `periodicCount = round(max(10, init*p/100))`, or `Long.MAX_VALUE` if p = 0.
   - `lazyThreshold = round(init*lazy/100)`.
   - `nodesToAvoidContract = round(init*(100 - contracted)/100)`.
4. Loop while the heap is not empty:
   1. If `checkCounter > 0 && checkCounter % periodicCount == 0`, recompute all priorities.
   2. `checkCounter++`; `node = poll()`.
   3. If the heap is not empty and `size < lazyThreshold`: recompute the node's priority; if `prio > peekValue()`, push it back and continue.
   4. `contractNode(node, level++)`: `chBuilder.setLevel`, then `nodeContractor.contractNode`. Contracting returns the neighbours: `prepareGraph.disconnect(node)` removes the node's edges and returns the distinct adjacent nodes in deterministic order.
   5. If `size < nodesToAvoidContract`, break.
   6. For each neighbour: if `rand.nextInt(100) < neighborPct` (with `Random(123)`), recompute its priority with `update`.
5. After the loop:
   - `finishContractionHook()` (ORS, empty by default).
   - `nodeContractor.finishContraction()`, which calls `chBuilder.replaceSkippedEdges(prepareGraph::getShortcutForPrepareEdge)` to remap prepare-edge ids to final CH edge ids.

**ORS hooks and visibility changes in this file:**
- Many fields made public: `chConfig`, `chStore`, `chBuilder`, `params`, `graph`, `nodeContractor`, `nodes`, `maxLevel`, `sortedNodes`, `pMap`.
- `getCHStore(chConfig)` is overridable.
- `initFromGraph()` is public; `contractNode(int, int)` is protected.
- `doNotContract(int)` and `finishContractionHook()` are new.
- ORS core uses these to stop contraction for "core" nodes and to set core levels.

### Node-based contractor (`C/routing/ch/NodeBasedNodeContractor.java`, `NodeBasedWitnessPathSearcher.java`)

**Priority:**
```
priority = edgeDifferenceWeight(10) * (shortcutsCount - prepareGraph.getDegree(node))
         + originalEdgesCountWeight(1) * sum(origEdgeCount(in) + origEdgeCount(out)) over needed shortcuts
```
- `degree` counts entries in the in and out adjacency lists of the prepare graph.
- Parameter keys: `prepare.ch.node.edge_difference_weight` and `prepare.ch.node.original_edge_count_weight`.

**Shortcut search (`findAndHandleShortcuts`):**
- For each incoming (non-loop, finite) prepare edge u→v: `witness.init(u, ignoreNode = v)`, one search per source, reused incrementally across targets.
- For each outgoing edge v→w, with w ≠ v and w ≠ u:
  - `direct = in.w + out.w`.
  - `maxW = findUpperBound(w, direct, maxSettled)`.
  - If `maxW <= direct`, a witness exists; otherwise this is a shortcut candidate.

**Witness search limits:**
- `maxSettled = (int) meanDegree * 100`. The cast is applied before the multiply.
- `meanDegree` starts at `origEdges / nodes` using **integer division**, assigned in `prepareContraction()`.
- After each contraction: `meanDegree = (meanDegree*2 + degree)/3`, where degree is the number of processed incoming edges.
- **Quirk to reproduce:** in the first priority round `meanDegree` is still 0, so every possible shortcut is counted.

**NodeBasedWitnessPathSearcher:**
- Data: `double[] weights` (∞ initialised), `IntFloatBinaryHeap(1000)` with float keys, and a changed-node list for reset.
- Loop runs while: the heap is not empty, `settledNodes < max`, and `heap.peekKey() <= acceptedWeight`.
- Early exits: return `weights[target]` immediately if it is already `<= accepted`; also return it once the target is settled.
- `settledNodes` is reset only in `init`, so it accumulates across targets.

**Shortcut insertion:**
1. `addOrUpdateShortcut` adds or updates a prepare shortcut u→w with skipped edges (in, out) and `origEdgeCount` summed. An existing shortcut u→w is updated only if the new weight is lower.
2. Before `disconnect`, `insertShortcuts(node)` writes all shortcuts adjacent to the node into CHStorage.
3. Out-shortcuts get the FWD flag.
4. For in-shortcuts, the skipped edges are swapped, and a forward shortcut is merged into a bidirectional one (`flags = 3`) if it has the same target, bit-identical weight, the same mapped skipped edges, and is forward-only.

### Edge-based contractor (`C/routing/ch/EdgeBasedNodeContractor.java`, `EdgeBasedWitnessPathSearcher.java`)

**Priority:**
```
priority = 1 * (numShortcuts / numPrevEdges)
         + 3 * (numOrigEdges / numPrevOrigEdges)
         + 2 * hierarchyDepth[node]
```
- Weight keys: `prepare.ch.edge.edge_quotient_weight`, `prepare.ch.edge.original_edge_quotient_weight`, `prepare.ch.edge.hierarchy_depth_weight`.
- An isolated node (`numAllEdges == 0`) gets `-Infinity`.
- After contracting, each neighbour gets `hierarchyDepth[n] = max(hierarchyDepth[n], hierarchyDepth[node] + 1)`.

**Search structure:**
- For each distinct source node s (in-neighbour), and each original in-edge of s:
  - `initSearch(center, s, origInEdge)`. Initial entries are the out-edges of s with turn cost `turn(origInEdge, s, firstOrig)` added.
  - The parent is a dummy entry that stores the initial turn cost and the first orig edge key.
- For each distinct target t, and each original out-edge of t:
  - `runSearch(t, targetEdge)`.
  - If it returns a bridge path (a path through the center that beats every witness), handle the shortcut chain. Duplicates are suppressed with a `LongHashSet` of (firstKey, lastKey).
  - The initial turn cost is subtracted from the entry weight.
- `addShortcutsToPrepareGraph` builds shortcuts recursively along the entry chain (a shortcut may cover several edges via loops).

**EdgeBasedWitnessPathSearcher:**
- Arrays indexed by edge key, size `2*origEdges`: `weights`, `prepareEdges`, `parents`, `adjNodesAndIsPathToCenters` (`(adjNode << 1) | isPathToCenter`).
- The heap is `IntFloatBinaryHeap`.
- Termination:
  - Stop when `numPathsToCenter < 1 && (!bestIsBridge || bestWeight == ∞)`.
  - Stop when `weights[peek] > bestPathWeight` (only peeked, so it can resume for the next target).
  - After `numSettledEdges > maxSettledEdges`, only entries that are paths to the center are expanded.
- Tie-breaking: a witness wins over a bridge path at equal weight (tolerance 1e-6). `MAX_ZERO_WEIGHT_LOOP = 1e-3`.
- Dynamic limit, recomputed every `settledEdgeStatsResetInterval` searches:
  ```java
  maxSettledEdges = max(minimumMaxSettledEdges, (int)(mean + sigmaFactor * sqrt(variance)));
  ```
- Defaults:

| Parameter | Config key | Default |
|---|---|---|
| `sigmaFactor` | `prepare.ch.edge.witness_search.sigma_factor` | 3.0 |
| `minimumMaxSettledEdges` | `...min_max_settled_edges` | 100 |
| `settledEdgeStatsResetInterval` | `...reset_interval` | 10000 |

**Turn costs during preparation:**
- `CHPreparationGraph.buildTurnCostFunctionFromTurnCostStorage` stores per-via-node sorted arrays of (fromEdge, toEdge) pairs with their cost.
- `uTurn` cost is `weighting.calcTurnWeight(1, 0, 1)` when `in == out`.
- Invalid edges give 0.

**Edge-based shortcut insertion:**
- Shortcuts are single-direction only: out = FWD, in = BWD. In-loops are skipped because they were already added via the out explorer.
- Each shortcut stores orig first/last edge ids.

### CHPreparationGraph (`C/routing/ch/CHPreparationGraph.java`)

- One object per edge/shortcut, threaded through two linked lists per node: `prepareEdgesOut[]` and `prepareEdgesIn[]`.
- `degrees[]` per node.
- Shortcut prepare-edge ids start at `edges` (`nextShortcutId`).
- `shortcutsByPrepareEdges` maps prepare edge → final CH edge (`origEdges + shortcutIndex`).
- Edge-based mode builds an `OrigGraph` (original-edge adjacency used for source/target orig edges).
- ORS changes: many members made public, `getPrepareEdgesOut/In()` added, and `getTime/setTime` on the iterator (these throw UnsupportedOperationException).

### CHStorage layout (`C/storage/CHStorage.java`, `CHStorageBuilder.java`)

**Shortcut record (bytes):**

| Offset | Field | Notes |
|---|---|---|
| 0 | `NODEA` | `nodeA << 1 \| fwdBit` |
| 4 | `NODEB` | `nodeB << 1 \| bwdBit` |
| 8 | `WEIGHT` | unsigned int milli-weight |
| 12 | `SKIP_EDGE1` | |
| 16 | `SKIP_EDGE2` | |
| 20 | `ORIG_FIRST` | edge-based only |
| 24 | `ORIG_LAST` | edge-based only |
| +4 | `S_TIME` | ORS, only when `type == "core"` |

- Record size is 20 bytes (node-based) or 28 bytes (edge-based).
- Access flags (`PrepareEncoder`): FWD = 0x1, BWD = 0x2, both = 0x3.

**Weight encoding:**
- `int = round(w * 1000)`.
- `MIN_WEIGHT = 0.001`: smaller weights are clamped up and reported through `LowWeightShortcut`.
- `MAX_STORED = Integer.MAX_VALUE << 1` (as long). Weights `>= MAX_STORED/1000` are stored as the marker, which reads back as +∞ and increments `numShortcutsExceedingWeight`.
- Decode with `(long) int & 0xFFFFFFFFL`.

**Node record:** `LEVEL (int) | LAST_SC (int)`, 8 bytes. `LAST_SC` is the highest shortcut index whose nodeA is this node, or -1.

**Headers:**
- nodes file: version, nodeCount, entryBytes, `coreNodeCount` (ORS, at offset 12).
- shortcuts file: version, count, entryBytes, numExceeding, edgeBased.
- ORS file names: `nodes_<type>_<name>`, `shortcuts_<type>_<name>`, with type `"ch"` or `"core"`.

**Invariants:**
- A shortcut is registered only at nodeA, the lower-level node.
- Shortcuts are appended in non-decreasing level(nodeA) order. `checkNewShortcut` enforces `level(a) < level(b)`, distinct levels, and monotonic nodeA level.
- ORS `addShortcutCore` skips these checks and writes `S_TIME`.

### Query graph view (`C/storage/RoutingCHGraphImpl.java`, `RoutingCHEdgeIteratorImpl.java`, `RoutingCHEdgeIteratorStateImpl.java`)

**Edge ids:** CH edge id = `baseEdges + shortcutIndex`.

**`setBaseNode(n)`:**
- Start at `lastShortcut(n)`.
- Iterate shortcuts in decreasing index while `nodeA == n`. Accept an outgoing shortcut if it has FWD, an incoming one if it has BWD; a loop is accepted if it has either flag.
- Then iterate base edges, accepting when the direction weight is finite (loops: either direction).

**Weights:**
- Shortcut: `getWeight` returns the stored weight.
- Base edge: `calcEdgeWeight(baseEdge, reverse)`, or ∞ if there is no access (loops are exempt).

**ORS additions:**
- `getTime(reverse)`: the shortcut's `S_TIME`, or `calcEdgeMillisWithAccess` for base edges.
- `getCoreNodes()`.

### CH query

**Common base (`C/routing/AbstractBidirCHAlgo.java`):**
- `CHLevelEdgeFilter`:
  - Accept if either endpoint is virtual (`>= baseGraph.getNodes()`).
  - Accept all shortcuts (they only ever go upward).
  - Otherwise require `level(base) <= level(adj)`.
- `finished()` is true when both directions are finished, or `currFrom.weight >= best && currTo.weight >= best`. Note this uses heap keys.
- Edge weight: `getWeight(reverse)` plus `graph.getTurnWeight` using orig-first/last.
- ORS: the "turn costs with node-based" check is disabled, for core landmarks.

**Node-based algorithms:**
- `DijkstraBidirectionCH` is the default `dijkstrabi`, with stall-on-demand (hint `stall_on_demand`, default true).
- Stall check: for the entry at v, scan v's in-edges (forward search) or out-edges (backward search), at all levels, skipping `entry.edge`. Stall if `d(w) + w(w→v) - d(v) < -0.001`. The 0.001 tolerance exists because of rounded shortcut weights.
- `DijkstraBidirectionCHNoSOD` is the variant without stalling.
- `AStarBidirectionCH`: heap key `g + balanced h`; uses the CH `finished()`.

**Edge-based algorithms (`AbstractBidirectionEdgeCHNoSOD`):**
- The first expansion uses `CHEdgeFilter.ALL_EDGES`.
- `origEdgeId = reverse ? origFirst : origLast`.
- Traversal id: `createEdgeKey(otherNode(origEdge, adj), adj, origEdge, reverse)`.
- Meeting (`updateBestPath`):
  - Special case: the search hits the opposite endpoint directly (respecting edge restrictions).
  - Otherwise: for every base edge at `entry.adjNode`, look up `key = createEdgeKey(iter.adj, iter.base, edge, !reverse)` in the other map; candidate = `g + g_other + turn(origEdge, node, edge)` in the appropriate order.
- The default algorithm is `AStarBidirectionEdgeCHNoSOD` (`astarbi`):
  - Heap key = g (`useHeuristicForNodeOrder = false`).
  - The heuristic is used only for pruning: skip the entry if `g + h > bestWeight`.
- `dijkstrabi` maps to `DijkstraBidirectionEdgeCHNoSOD`.

**Path unpacking (`C/routing/ch/ShortcutUnpacker.java`):**
- Recursive.
- Node-based: `sk2 = getEdge(skip2, adj)`; if null, swap the skipped edges; then `sk1 = getEdge(skip1, sk2.base)`.
- Edge-based: swap skip1/skip2 when reverse; the turn context between the halves is `getOppositeEdge(sk1, base)` (orig first/last).
- `reverseOrder = true` is used by the extractors, which then visit the original edges.
- Node-based extractor time: `weighting.calcEdgeMillis`.
- Edge-based extractor time: `calcMillisWithTurnMillis`, using the query-graph-wrapped weighting.

**Factory and calculator (`C/routing/ch/CHRoutingAlgorithmFactory.java`, `C/routing/CHPathCalculator.java`):**
- The factory wraps the graph in a `QueryRoutingCHGraph`.
- Unfavored edges (heading) are rejected for CH.
- The calculator uses `calcPath(from, to, srcOut, tgtIn)` when restricted, otherwise `calcPaths`.
- The TD `at` argument is ignored.

---

## 3. Landmarks (ALT)

### LandmarkStorage (`C/routing/lm/LandmarkStorage.java`)

**Constants:**
- `SHORT_INFINITY = 65535`, `SHORT_MAX = 65534`.
- `PRECISION = 1 << 16`.
- `factor = maxWeight / 65536`.
- `UNSET_SUBNETWORK = -1`, `UNCLEAR_SUBNETWORK = 0`.
- `minimumNodes = min(nodes/2, 500_000)`, overridden by `prepare.lm.min_network_size`.
- Turn-cost weightings are rejected.
- Traversal is always node-based.

**Weight storage (`landmarkWeightDA`):**
- One row per node, `LM_ROW_LENGTH = landmarks*4` bytes. For landmark i: unsigned short "from" at `i*4 + 0` = d(LM → node), unsigned short "to" at `i*4 + 2` = d(node → LM).
- Everything is initialised to 65535 (`SHORT_INFINITY`).
- Store: `(short)(w/factor)`, a truncating cast. Values `>= SHORT_MAX` are stored as `SHORT_MAX` and counted as "maxed out"; if more than 10 % of a landmark's nodes are maxed out, a warning is logged.
- Read: `SHORT_INFINITY` is returned as `SHORT_MAX`.
- After the rows: per subnetwork, `landmarks` ints of landmark node ids. Subnetwork 0 is a dummy array of -1.
- Header: [0] nodes, [4] landmarks, [8] subnetworkCount, [12] `round(factor * 1e6)`. On load, `factor = header/1e6`.

**Subnetwork ids (`SubnetworkStorage`):**
- One signed byte per node, so at most 127 subnetworks.
- Edges flagged in `<profile>_subnetwork` (from `PrepareRoutingSubnetworks`) are blocked.
- `areaIndex` border edges are blocked; the built-in `map.geo.json` split areas are loaded unless `prepare.lm.split_area_location` is set.
- Components come from `TarjanSCC.findComponents(graph, filter, true)`; components smaller than `minimumNodes` are skipped.

**Factor estimation (when `maximum_lm_weight` < 0; the `LMProfile` default is -1):**
1. For each big component, pick a random start (`Random(0)`).
2. Find 3 landmarks from it.
3. Run a full Dijkstra from each of those 3 with the real weighting and take the last settled weight.
4. `maxWeight = max(...) * 1.008`.

**Landmark selection (`findLandmarks`), a farthest-node heuristic:**
- Selection weighting `LM_BFS`: every edge with finite real weight costs 1 (BFS hop count).
- Landmark 0 is the last node settled by a full search from the start node. The start node itself is not a landmark.
- Landmark k+1 is the last node settled by a multi-source search seeded with landmarks 0..k.
- Landmark suggestions (by bbox) override this.

**Per landmark (`createLandmarksForSubnetwork`):**
- A full forward Dijkstra from the landmark (`DefaultLandmarkExplorer` = `DijkstraBidirectionRef` with one side pre-finished) fills "from".
- A full backward Dijkstra fills "to".
- The subnetwork id is assigned from the first landmark's trees. If that fails (conflict), the subnetwork is abandoned.
- If the reachable set is smaller than `minimumNodes`, those nodes get `UNCLEAR_SUBNETWORK` (0).

**`chooseActiveLandmarks(from, to, active[], reverse)`:**
- Return false if either subnetwork is `<= 0`; the approximator then falls back to beeline permanently.
- Throw `ConnectionNotFoundException` if the subnetworks differ.
- For each landmark:
  ```java
  fw = from(lm,to) - from(lm,from);
  tw = to(lm,from) - to(lm,to);
  key = reverse ? max(-fw, -tw) : max(fw, tw);
  ```
- Sort descending and take the top `active.length`. On re-selection (`active[0] >= 0`), up to `min(len-2, 2)` of the previous landmarks are kept.

**ORS overridable hooks (for core landmarks):**
- `getLandmarksFileName()` returns `"landmarks_"`; the file is `landmarks_<lmConfigName>`.
- `getLandmarkExplorer(filter, weighting, reverse)` and `getLandmarkSelector(filter)`, with a public `LandmarkExplorer` interface.
- `getIndex(node)` for a node-index remap.
- `getBaseNodes()`.
- `estimateMaxWeight` and `createLandmarksForSubnetwork` made protected; `setWeight` protected.
- Getters: `getLandmarkWeightDA`, `getLandmarkIDs`, `getSubnetworkStorage`, and so on.
- Related: `PrepareLandmarks.createLandmarkStorage(...)` and `LMPreparationHandler.createPreparationsInternal(...)` are overridable, and the PREPARE/DISABLE/COUNT strings are overridable fields.

### LMApproximator (`C/routing/lm/LMApproximator.java`)

**`setTo(t)`:**
- Sets t on the beeline fallback (`DistanceCalcEarth`).
- `findClosestRealNode(t)`: a Dijkstra from t until the first node with id `< maxBaseNodes` is polled. Sets `towerNodeNextToT` and `weightFromTToTowerNode`.

**`approximate(v)`:**
1. If in fallback mode (and no recalculation pending), or the storage is empty: return the beeline value.
2. If v is virtual (`v >= maxBaseNodes`): return 0.
3. If `v == towerNodeNextToT`: return 0.
4. On first use: `chooseActiveLandmarks(v, towerNodeNextToT, ...)` and cache the "from" and "to" weights of t for the active landmarks.
5. Per active landmark i:
   ```java
   rhs1 = fromW_T[i] - from(lm_i, v);   // d(LM,t) - d(LM,v)
   rhs2 = to(lm_i, v) - toW_T[i];       // d(v,LM) - d(t,LM)
   r = reverse ? max(-rhs2, -rhs1) : max(rhs1, rhs2);
   ```
6. `maxInt = max(0, r_i...)`.
7. Return `max(0, ((maxInt - 1) * factor - weightFromTToTowerNode) * epsilon)`.

**Other:**
- `getSlack()` returns `factor`.
- `reverse()` returns a new approximator with `!reverse`.
- ORS adds `getFactor()`.

**How it plugs into A\*:**
- `LMRoutingAlgorithmFactory`, with `active = hint lm.active_landmarks`:
  - The factory default is `max(1, min(landmarks/2, 12))`, but the Router overrides it with `routing.lm.active_landmarks`, default `min(8, landmarks)`.
  - The landmark count default is 16 (`prepare.lm.landmarks`).
- `astarbi` (the default) uses AStarBidirection with `BalancedWeightApproximator(LM)` and epsilon `astarbi.epsilon` (default 1).
- `astar` uses `astar.epsilon`.
- `alternative_route` is supported.
- `td_astar` is ORS.

---

## 4. Weighting

### Interface (`C/routing/weighting/Weighting.java`)

**Upstream methods:**
- `getMinWeight(dist)`
- `calcEdgeWeight(edge, reverse)`
- `calcEdgeMillis(edge, reverse)`
- `calcTurnWeight(in, via, out)`
- `calcTurnMillis`
- `hasTurnCosts`
- `getFlagEncoder`
- `getName`
- default `calcEdgeWeightWithAccess`

**ORS additions:**
- `calcEdgeWeight(edge, reverse, long edgeEnterTime)`
- `calcEdgeMillis(edge, reverse, long)`
- `calcEdgeMillisWithAccess` (returns -1 if there is no access)
- `isTimeDependent()`
- `getSpeedCalculator()` / `setSpeedCalculator()`

### AbstractWeighting

- `speedCalculator = new DefaultSpeedCalculator(encoder)`, which reads `avg_speed` for the direction (ORS).
- `calcEdgeMillis(edge, rev, t)`:
  - Loop edges are read forward.
  - Throws if there is no access in that direction.
  - `speed = speedCalculator.getSpeed(edge, rev, t)`.
  - Returns `(long)(distance * 3600 / speed)`. Speed 0 throws.
- Valid weighting names match `[|_a-z]+`.

### Concrete weightings

**FastestWeighting:**
- `speed = speedCalculator.getSpeed(...)`; 0 gives ∞.
- `time = dist / speed * 3.6`.
- Road access multipliers:
  - `road_access_destination_factor`: default 10 for motor vehicles, 1 otherwise; range [1, 10].
  - `road_access_private_factor`: default 10 for motor vehicles, 1.2 otherwise.
- Unfavored edge: add `heading_penalty` (default 300 s). Millis add `round(300 * 1000)`.
- `getMinWeight = dist / (encoder.maxSpeed / 3.6)`.
- `isTimeDependent = speedCalculator.isTimeDependent()`.

**ShortestWeighting:** weight = distance; min weight = distance.

**PriorityWeighting** (`fastest` when the encoder supports it):
- `weight = fastest / priority`, with priority from the decimal encoded value `<enc>.priority`.
- Max priority = BEST = 15/10 = 1.5.
- `getMinWeight = (1/1.5) * fastestMin`.
- PriorityCode values: EXCLUDE 0, REACH_DESTINATION 1, VERY_BAD 3, BAD 5, AVOID_MORE 6, AVOID 8, SLIGHT_AVOID 9, UNCHANGED 10, SLIGHT_PREFER 11, PREFER 12, VERY_NICE 13, BEST 15. The factor is value/10.

**ShortFastest:**
- `time * time_factor(1) + dist * distance_factor(0.07)`.
- Min weight is the analogous combination.

**Other weightings:**
- `curvature` exists.
- DefaultWeightingFactory maps profile weighting names `custom`, `shortest`, `fastest`, `curvature`, `short_fastest`. The u-turn cost comes from hint `u_turn_costs`, default -1 (infinite).

### TurnCostProvider (`DefaultTurnCostProvider`)

- Invalid in/out edge: 0.
- `in == out`: uTurnCosts (∞ by default). This overrides any stored cost.
- Otherwise: `turnCostStorage.get(turnCostEnc, from, via, to)`.
- `millis = 1000 * weight`.
- `NO_TURN_COST_PROVIDER` returns 0.

### QueryGraphWeighting

- At a virtual via node: a u-turn between the same virtual edge ids is ∞, anything else is 0.
- Otherwise, virtual edges are mapped to `closestEdges[(e - firstVirtualEdgeId) / 2]` before calling the inner turn weight.
- Turn millis = `1000 * turnWeight`.

### Adjusted weightings

- `AbstractAdjustedWeighting` delegates everything, including the ORS TD methods.
- `BlockAreaWeighting`: ∞ if `blockArea.intersects(edge)`; created only by FlexSolver for the `block_area` hint.
- `AvoidEdgesWeighting`: multiplies by `edgePenaltyFactor` (default 5.0) for edges in the set.
- `TimeDependentAccessWeighting` (ORS): ∞ unless `TimeDependentAccessEdgeFilter.accept(edge, time)`.

### CustomWeighting (`C/routing/weighting/custom/CustomWeighting.java`)

**Edge weight:**
```
weight = seconds / priority + distance * distanceInfluence
```
- `seconds = dist / speed * 3.6`, plus `headingPenalty` if the edge is unfavored.
- ∞ if there is no base access in that direction (loops read forward), or if speed is 0.
- Throws if `speed > maxSpeed`.
- `distanceInfluence` is given in s/km and divided by 1000. `DEFAULT_DISTANCE_INFLUENCE = 70`.

**Other methods:**
- `getMinWeight = d / maxSpeed_ms / maxPriority + d * distInfluence`.
- `calcEdgeMillis = round(seconds * 1000)`. It does not use the SpeedCalculator, so CustomWeighting is never time-dependent.

### CustomModelParser (`CustomModelParser.java`, `ExpressionVisitor.java`, `W/json/Statement.java`, `W/util/CustomModel.java`)

**Statements:**
- Keywords: `if`, `else_if`, `else`.
- Operations: `multiply_by` emits `value *= X`; `limit_to` emits `value = Math.min(value, X)`.
- Each statement becomes Java: `if (cond) {op;}` / `else if (...)` / `else {op;}`.

**Generated methods:**
- `getSpeed`:
  ```java
  double value = super.getRawSpeed(edge, reverse);
  // <variable declarations>
  // <statements>
  return Math.min(value, globalMaxSpeed);
  ```
- `getPriority`:
  ```java
  double value = super.getRawPriority(edge, reverse);  // 1 if no priority EV
  // <variable declarations>
  // <statements>
  return value;
  ```

**Variable declarations:**
- Each encoded value used becomes `T x = reverse ? edge.getReverse(enc) : edge.get(enc)`.
- Type mapping: Enum → Enum, String → int (index), Decimal → double, Int → int, Boolean → boolean.
- ORS MOD: a `SparseEncodedValue` becomes `Object x = ((SparseEncodedValue) x_enc).get(edge.getEdge())`.

**Allowed in conditions:**
- Identifiers: encoded-value names, all-uppercase constants, `edge`, `Math`, and `in_<area>`.
- Literals and parentheses.
- Unary `!` only.
- Any Janino binary operation (`&&`, `||`, `==`, `!=`, `<`, `<=`, `>`, `>=`, arithmetic).
- Method calls: only `ordinal`, `getDistance`, `getName`, `contains`, `sqrt`, `abs`, and only on the targets `edge.` or `Math.`.

**Rewrites:**
- `enc == ENUMCONST` becomes `EncClass.ENUMCONST`. Only `==`/`!=` are allowed for enums.
- A string literal compared to a string encoded value becomes the value's index (0 becomes -1); only `==`/`!=` are allowed.
- `in_x` becomes `CustomWeightingHelper.in(this.in_x, edge)`: a bbox check, then rectangle shortcut, then polygon intersection with the edge geometry. Areas must be Polygon/MultiPolygon features with no properties and no bbox.

**Compilation:**
- Janino `SimpleCompiler`. The class template is parsed, then the method bodies are injected with a `DeepCopier`.
- Cache key: `customModel.toString() + maxSpeed + maxPriority`.
- LRU cache of size `graphhopper.custom_weighting.cache_size` (default 1000), plus an internal cache for `__internal` models.
- The key is limited to 100k characters.

**Max values (`CustomModel.findMax`):**
- Statements are split into if-blocks.
- If a block's condition is `"true"`, apply its op to the current max.
- Otherwise take the max over all branches; if the block has no `else`, also include the unchanged value.

**Merge and LM constraints:**
- `merge`: query statements are appended to the profile's; `distance_influence` is overridden only if set; duplicate area names throw.
- `checkLMConstraints`: a query's `distance_influence` must be `>=` the base value; `multiply_by` factors must be ≤ 1; max priority ≤ 1.
- `heading_penalty` default is 300.

### SpeedCalculator (ORS-specific, `C/routing/util/`)

- Interface `SpeedCalculator`: `getSpeed(edge, reverse, long time)` and `isTimeDependent()`.
- `DefaultSpeedCalculator`: reads `avg_speed`; not time-dependent.
- `AbstractAdjustedSpeedCalculator`: decorator.
- `ConditionalSpeedCalculator`:
  - Applies when `time != -1` and the edge has the `<enc>.conditional_speed` flag.
  - Parses the `maxspeed:conditional` string (stored in `ConditionalEdgesMap`) with `ConditionalRestrictionParser`.
  - Iterates restrictions from last to first; the first match by `TimeDependentConditionalEvaluator` (OpeningHours rules: year, date, week, weekday, time; with extended >24 h times) wins.
  - Returns `maxspeed * 0.9`.
  - Always time-dependent.
- `DateTimeHelper` (`C/util/DateTimeHelper.java`): **the time zone is hard-coded to `Europe/Berlin`**; it takes the edge base-node location but ignores it.
- `TimeDependentAccessEdgeFilter`: evaluates `<enc>.conditional_access` restrictions. The value `"yes"` allows; if nothing matches it returns `!lastMatchValue`.
- `FerrySpeedCalculator` is an import-time helper with an OSM MOD: it uses `exact_distance`/`estimated_distance` and `maxspeed/1.4`.

---

## 5. Router / GraphHopper.route flow (`C/GraphHopper.java`, `C/routing/Router.java`)

### Entry and solver selection

`GraphHopper.route` calls `createRouter()`, which builds:
- `chGraphs`: profile → `RoutingCHGraph`.
- `landmarks`: profile → `LandmarkStorage`, including cross-profile `preparation_profile`.
- It then calls `doCreateRouter` (overridable).

`Router.route`:
1. Validation: no legacy `vehicle`/`weighting`/`turn_costs`/`edge_based` hints; at least one point; points inside the bounds; headings count/range; point hints and curbsides count; no `block_area` together with `custom_model`.
2. `createSolver`:
   - **CHSolver** if CH is enabled and not `ch.disable`. Its weighting is the CH graph's weighting; request hints are ignored. It rejects `heading`, `pass_through`, `block_area`, `custom_model` and `round_trip`.
   - **LMSolver** if LM is enabled and not `lm.disable`.
   - **FlexSolver** otherwise.
3. `solver.init()`: resolve the profile, check compatibility (curbsides need turn costs; a finite `u_turn_costs` needs a turn-cost profile), create the weighting (`DefaultWeightingFactory`, plus `BlockAreaWeighting` for flex).
4. Dispatch: `round_trip` → `routeRoundTrip` (flex only); `alternative_route` → `routeAlt` (2 points only, no pass-through or curbside); otherwise `routeVia`.

**ORS changes in Router:**
- `edgeFilterFactory` injection: the snap filter and `AlgorithmOptions.edgeFilter` are passed to `algo.setEdgeFilter`.
- `checkMaxSearchDistances` (radius per point).
- `PathProcessorFactory` / `PathProcessor`.
- `data.date` hint.
- `time` hint (long, default -1) for TD routing.

**Algorithm options:**
- Flex/LM traversal mode: `EDGE_BASED` if `profile.isTurnCosts()`.
- Algorithm defaults:
  - Flex: `astarbi` (also when empty).
  - LM: `astarbi`.
  - CH node-based: `dijkstrabi`.
  - CH edge-based: `astarbi`.
- The `AlgorithmOptions` class default is `dijkstrabi`, but the request algorithm overrides it.

**RouterConfig defaults:**

| Setting | Default |
|---|---|
| `maxVisitedNodes` | `Integer.MAX_VALUE` |
| `maxRoundTripRetries` | 3 |
| `nonChMaxWaypointDistance` | `Integer.MAX_VALUE` |
| `calcPoints` | true |
| `simplifyResponse` | true |
| `elevationWayPointMaxDistance` | `Double.MAX_VALUE` |
| `activeLandmarkCount` | 8 |

### ViaRouting (`C/routing/ViaRouting.java`)

**Snapping, in order of precedence:**
1. A heading present: `HeadingEdgeFilter(directedSnapFilter, heading, point)`.
2. A point hint: `NameSimilarityEdgeFilter(strictFilter, hint, point, 100)`.
3. Snap preventions: `SnapPreventionEdgeFilter`.
4. Fallback: `DefaultSnapFilter` (not in a subnetwork and finite in either direction).

**Per leg:**
- `fromHeading` is applied only to leg 0. `toHeading = heading[leg+1] + 180`, wrapped above 360.
- `HeadingResolver` marks as unfavored every edge whose first segment deviates more than 100° from the heading.
- `pass_through` unfavors the previous leg's final edge.
- Curbsides: `DirectionResolver` determines source out-edge and target in-edge. `force_curbside` defaults to true; an impossible constraint throws.
- Then `pathCalculator.calcPaths(fromSnap.closestNode, toSnap.closestNode, time, restrictions)`.

**FlexiblePathCalculator:**
- Marks unfavored virtual edges (`unfavorVirtualEdge` on both directions).
- Uses `calcPath(..., srcOut, tgtIn)` when restricted (bidirectional algorithms only), otherwise `calcPaths` or `calcPaths(..., at)`.
- Clears the unfavored status afterwards.

### PathMerger (`C/util/PathMerger.java`)

**Per path:**
- Sum time, distance and weight.
- Instructions via `InstructionsFromEdges` (with the ORS PathProcessor). The Finish of every leg except the last becomes a `ViaInstruction(viaCount)`.
- Points via `path.calcPoints()`; the last point is dropped for all legs but the last.
- Path details via `PathDetailsFromEdges`. Each interval index advances by the edge's `PILLAR_AND_ADJ` geometry size.

**After all paths:**
- `pathProcessor.processPoints` (ORS).
- Ascend/descend for 3D.
- `updateInstructionsWithContext`:
  - The first instruction becomes U_TURN_UNKNOWN if the favored heading differs from it by between 170° and 190°.
  - After a via point, the next instruction becomes U_TURN_UNKNOWN if `|last_heading - heading| % 360` is between 179 and 181.
- Simplification: `PathSimplification` runs Douglas-Peucker per partition interval (instructions and each path detail).
  - `way_point_max_distance` hint default 1 m; `elevation_way_point_max_distance` default `Double.MAX_VALUE`.
  - Plane projection; segments of 500 points; points marked NaN, then compacted.
  - Only when `simplifyResponse && wpmd > 0`.

**Path details** (`C/util/details/PathDetailsBuilderFactory.java`):
- `average_speed`, `street_name`, `edge_id`, `edge_key`, `time`, `weight`, `distance`.
- Any `*priority` decimal encoded value.
- `max_speed`, `max_width`, `max_height`, `max_weight`, `max_axle_load`, `max_length`.
- Boolean: `roundabout`, `road_class_link`, `get_off_bike`.
- Enum: road_class, road_environment, surface, smoothness, road_access, bike/foot network, toll, track_type, hazmat (all three), country.
- Int: mtb/hike/horse rating, lanes.

### Path (`C/routing/Path.java`)

- Fields: edge id list, fromNode, endNode, distance, time, weight, found.
- ORS: an `IntArrayList times` gets one entry per `addTime` call, including the meeting-point turn time, so it is not strictly aligned with edges.
- `forEveryEdge`: walk from `fromNode` with `getEdgeIteratorState(edgeId, tmpNode)`; re-fetch oriented from the base node.
- `calcPoints`: the from-node point, then `PILLAR_AND_ADJ` per edge.

### Instructions (`C/routing/InstructionsFromEdges.java`, `InstructionsHelper.java`, `InstructionsOutgoingEdges.java`, `W/util/Instruction.java`, `RoundaboutInstruction.java`, `C/util/AngleCalc.java`)

**Sign codes:**

| Sign | Code |
|---|---|
| UNKNOWN | -99 |
| U_TURN_UNKNOWN | -98 |
| U_TURN_LEFT | -8 |
| KEEP_LEFT | -7 |
| LEAVE_ROUNDABOUT | -6 |
| TURN_SHARP_LEFT | -3 |
| TURN_LEFT | -2 |
| TURN_SLIGHT_LEFT | -1 |
| CONTINUE_ON_STREET | 0 |
| TURN_SLIGHT_RIGHT | 1 |
| TURN_RIGHT | 2 |
| TURN_SHARP_RIGHT | 3 |
| FINISH | 4 |
| REACHED_VIA | 5 |
| USE_ROUNDABOUT | 6 |
| KEEP_RIGHT | 7 |
| U_TURN_RIGHT | 8 |
| IGNORE | `Integer.MIN_VALUE` |

**Angle maths (AngleCalc):**
- `calcOrientation = atan2(dLat, cos(midLat) * dLon)`. The "non-exact" variant uses a fast atan2 approximation: `0.1963*r^3 - 0.9817*r + π/4` (or 3π/4).
- `alignOrientation` shifts by ±2π so the value lies within π of the base orientation.
- `calcAzimuth = toDeg(round4(π/2 - orientation))` normalised to [0, 360).

**Sign from delta (radians):**
- `|δ| < 0.2` (about 11°): continue.
- `< 0.8` (about 40°): slight.
- `< 1.8` (about 103°): turn.
- Otherwise: sharp.
- `δ > 0` means left.

**Decision flow (`getTurn`):**
1. Allowed alternatives are other out-edges with access, excluding the previous node and the next node. Visible alternatives include edges with no access.
2. If `allowedTurns <= 1`: emit the sign only if `|sign| > 1` and `visibleTurns > 1`; otherwise IGNORE.
3. If `|sign| > 1`: IGNORE when the name is the same and the alternatives are slower by factor 2; otherwise emit the sign.
4. Otherwise, at a fork (another allowed edge with `|sign| <= 1`), when the name changed, or the other continuation has the previous name, or the alternatives are not slower:
   - For motorway, trunk, primary, secondary or tertiary: IGNORE if this edge keeps the previous road class and link flag while the other continuation changes them.
   - If `|δ| < 0.1`, `|δ_other| > 0.15` and the name is the same: CONTINUE.
   - Otherwise KEEP_LEFT if `δ_other < δ`, else KEEP_RIGHT.
5. Otherwise, if the alternatives are not slower and (`|δ| > 0.6` or the route is leaving the current street): emit the sign.
6. Otherwise IGNORE.

**Supporting rules:**
- Speed comparison uses `max_speed`, falling back to `avg_speed`. If the path speed changes or is below 1, the alternatives are treated as not slower.
- Name similarity is equality, but two empty names are not similar.

**U-turn merge:**
- Conditions:
  - The previous instruction is shorter than 35 m (`MAX_U_TURN_DISTANCE`).
  - Both turns have the same side and are slight, normal or sharp turns.
  - The edge is one-way (fwd access ≠ bwd access).
  - The name equals the instruction name before the previous one.
  - The orientation difference is between 0.9π and 1.1π.
- When all hold: the previous instruction becomes U_TURN_LEFT/RIGHT.

**Roundabouts:**
- On entry: a `RoundaboutInstruction`. If an out-edge at the entry node is not a roundabout edge and does not lead to the previous node, the exit count is incremented.
- The rotation direction comes from the entry turn delta.
- Every adjacent node that has a non-roundabout out-edge counts as an exit.
- On exit: `radian = δ_in_out`, rotation from the exit delta, `setExited`.
- **ORS MOD:** an extra `CONTINUE_ON_STREET` instruction with the exit street name is added immediately after leaving the roundabout.
- `turn_angle = π*clockwise - radian`, where clockwise is ±1; it is NaN if inconsistent.

**Bookkeeping:**
- The first instruction is `CONTINUE_ON_STREET` with extra info `heading`.
- Finish carries extra info `last_heading`.
- Instruction time = `weighting.calcEdgeMillis(edge, false)` per edge. The ORS `times` array is passed in but not used.

---

## 6. QueryGraph (`C/routing/querygraph/`)

### Overlay construction (`QueryOverlayBuilder`)

**Phase 1:**
- Skip snaps that land exactly on a tower node.
- Orient the closest edge so that `base <= adj`. Equal base and adj (loops) are decided by the latitude of the first vs last pillar. When the edge is reversed, the way index is recomputed (pillar vs off-pillar case).
- Group snaps by edge.

**Phase 2, per edge:**
- Sort snaps by way index, then by plane distance to the pillar point.
- Identical snapped points reuse the previous node.
- Each new virtual node (id `baseNodes + i`) gets one pair of edges for the segment `prevNode → virt`, plus a final pair `lastVirt → adj`.
- Virtual edge id = `baseEdges + numVirtualEdges/2`.
- Edge states are stored as `VirtualEdgeIteratorState`, with the original edge key, a copy of the flags, the name, the point list, and distance computed with `DistancePlaneProjection`.

**Layout:**
- Four entries per virtual node at internal positions `i*4 + {BASE_SNAP 0, SNAP_BASE 1, SNAP_ADJ 2, ADJ_SNAP 3}`. Edges are duplicated so that position arithmetic works.
- The reverse-edge position of p is p^1.
- `closestEdges[i]` is the original edge for virtual node i.

### Edge changes and iteration

**`EdgeChangeBuilder`:** for each real node adjacent to a virtual node, add the virtual edge(s) and remove the original closest edge.

**`QueryGraph`:**
- The explorer at a virtual node returns `[SNAP_BASE, SNAP_ADJ]`.
- The explorer at a changed real node returns a precomputed list: the virtual edges plus the base edges minus the removed ones.
- Otherwise it delegates to the base graph.
- `wrapWeighting` returns a `QueryGraphWeighting`.
- Unfavor/clear support for heading.

### Edge filters

- `DefaultSnapFilter`.
- `AccessFilter` in/out/all.
- ORS `AccessEdgeFilter` / `ConditionalAccessEdgeFilter`: access is also granted if the `conditional_access` flag is set.
- `CHEdgeFilter.ALL_EDGES`.
- ORS `TimeDependentEdgeFilter`.

### CH over virtual edges (`QueryRoutingCHGraph`)

- Virtual CH edge id = `virtualEdge + (chEdges - baseEdges)`, which shifts it past all shortcuts. The internal id is `2*(e - chEdges)`.
- Virtual nodes report `level = Integer.MAX_VALUE`, and `CHLevelEdgeFilter` accepts every virtual edge.
- At changed real nodes, separate out/in lists are built:
  - Virtual edges, with `fwd/bwd = calcEdgeWeightWithAccess`.
  - Every shortcut of the node.
  - Base edges minus the removed ones.
  - All of these are wrapped as `VirtualCHEdgeIteratorState`; `isShortcut` means `origEdge == NO_EDGE`.
- The iterator does not filter by direction. Infinite weights are skipped by the algorithms.
- Turn weight: 0 if the CH graph has no turn costs; otherwise `QueryGraphWeighting.calcTurnWeight`.
- ORS:
  - `QueryRoutingCoreGraph extends QueryRoutingCHGraph` adds per-edge `timeFwd/timeBwd`.
  - Methods made protected: `buildVirtualEdgesAtRealNodes`, `buildVirtualCHEdgeState`, `shiftVirtualEdgeIDForCH`.
  - `getQueryOverlay()` and `getQueryGraph()` getters added.

---

## 7. Subnetworks (`C/routing/subnetwork/PrepareRoutingSubnetworks.java`)

- `prepare.min_network_size` default 200.
- Run per profile with `u_turn_costs = 0`.
- Uses `EdgeBasedTarjanSCC`: the edge filter is the finite turn+edge weight.
- Marks, as `<profile>_subnetwork = true` on the stored-direction edge:
  - every component other than the largest that has fewer than `2*minNetworkSize` edge keys, and
  - all single-edge components.
- Inaccessible edges are not marked.
- Throws if more than half of the directed edges would be marked.

---

## 8. Fork-specific changes (ORS / HeiGIT) at a glance

**TD routing:**
- `TDDijkstra`, `TDAStar`, `TDPathExtractor`.
- `SPTEntry.time`.
- `calcEdgeWeight` / `calcEdgeMillis` with an enter-time argument.
- `GHUtility.calcWeightWithTurnWeightWithAccess(..., edgeEnterTime)`.
- The `time` and `arrival` hints.
- `PathCalculator.calcPaths(from, to, at, restrictions)`.
- `RoutingAlgorithm.calcPath(from, to, at)`; bidirectional algorithms throw.
- `Path.times`.

**Conditional data:**
- `ConditionalEdges` / `ConditionalEdgesMap` (edge → string index), stored in `GraphHopperStorage` (`getConditionalAccess/Speed`).
- `ConditionalSpeedCalculator` (×0.9), `TimeDependentAccessEdgeFilter`, `TimeDependentAccessWeighting`, `TimeDependentConditionalEvaluator`, `TimePoint`, `DateTimeHelper` (Europe/Berlin).
- `ConditionalAccessEdgeFilter`, `AccessEdgeFilter`.
- `reader/osm/conditional/*`.

**Speed calculators:** `SpeedCalculator`, `DefaultSpeedCalculator`, `AbstractAdjustedSpeedCalculator`; the weighting's speed comes from `speedCalculator`.

**Core / CALT:**
- CHStorage type `"core"`: `S_TIME`, `shortcutCore`, `coreNodeCount`.
- `CHConfig` / `CHProfile` type constant `TYPE_CORE`; `GraphHopperStorage.getIsochroneGraph` (type `"isocore"`).
- `CHStorageBuilder.addShortcutCore`, `RoutingCHEdgeIteratorState.getTime`, `QueryRoutingCoreGraph`, `CHEdgeIteratorState.get/setTime`.
- PrepareCH hooks: `doNotContract`, `finishContractionHook`, `getCHStore`, protected `contractNode`, public fields.
- `CHPreparationGraph` made public.
- LM storage, preparation and handler hooks (Section 3).
- The disabled turn-cost check in `AbstractBidirCHAlgo`.
- ALT patch `approximatorOffset = 2*factor`, effectively dead for AStarBidirection.

**Matrix and isochrones:** `Dijkstra.reverseDirection`; `SPTEntry.originalEdge`; `EdgeIteratorStateHelper` / `EdgeKeys.getOriginalEdge` (virtual → real edge).

**Extensibility:**
- `setEdgeFilter` on algorithms; `AlgorithmOptions.edgeFilter`; `EdgeFilterFactory`.
- `PathProcessor` / `PathProcessorFactory` (`processPathEdge`, `processPoints`).
- `GraphExtension`, `ExtendedStorageSequence`, `GraphStorageFactory`.
- GraphHopper hooks: `loadORS()`, `postProcessingHook()`, `createOSMReader()`, `adjustLMWeighting()`, protected `createCHConfigs`.
- `checkMaxSearchDistances` in Router.

**CustomModel:** SparseEncodedValue support.

**Instructions:** the extra `CONTINUE_ON_STREET` after a roundabout exit.

**Misc:** `GHUtility.inORS` flag; Jackson 3 GeoJsonMapper; `FerrySpeedCalculator` maxspeed handling; new encoded values `MaxAxleLoad`, `MaxHeight`, `MaxLength`, `MaxWeight`, `MaxWidth`, `Country`.

### Behaviour to reproduce exactly

1. Node-based CH `meanDegree` uses integer division, and the cast happens before `*100`.
2. The witness-search heaps use float keys.
3. Prepare-graph base weights are floats; CH shortcut weights are integer milli-units with a 0.001 minimum.
4. LM weights are truncated to unsigned shorts, the factor is rounded to 1e-6 when saved, and the heuristic uses `(maxInt - 1) * factor`.
5. The ORS ALT stopping offset is overridden and never applies to AStarBidirection.
6. Turn time at the bidirectional meeting point goes into `Path.times` as an extra element.
