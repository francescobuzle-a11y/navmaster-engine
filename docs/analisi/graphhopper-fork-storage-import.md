# GraphHopper fork GIScience v4.16.0 — storage, import OSM, valori codificati, indice spaziale

Analisi del codice sorgente (tag v4.16.0, commit e45cce9). Le modifiche del fork sono marcate
nel sorgente con `// ORS-GH MOD START/END`. Percorsi relativi a `core/src/main/java/com/graphhopper`.

## 1. DataAccess (`storage/*DataAccess.java`, `GHDirectory`, `StorableProperties`)

- Segmenti: default `1<<20` (1 MiB), minimo `1<<7`, arrotondati per difetto a potenza di 2.
  `bufferIndex = pos >>> segmentSizePower`, `index = pos & (segSize-1)`.
- Dati **little-endian**. Un `int` non può stare a cavallo di due segmenti (eccezione); uno `short` sì.
- File: `HEADER_OFFSET = 100`, intestazione scritta con `RandomAccessFile` quindi **big-endian**:
  `00 02 'G' 'H'` | long length | int segmentSize | 20 int di header utente (byte 16..95) | 4 inutilizzati | segmenti.
- Varianti: RAM (`byte[][]`), RAM_INT (`int[][]`, usata per nodes/edges/turn_costs/location_index/CH),
  MMAP (`FileChannel.map` per segmento a offset `100 + i*seg`).
- Un file per DataAccess, nome minuscolo senza estensione. Tipo default `graph.dataaccess.default_type = RAM_STORE`.
- `properties`: testo UTF-8 `chiave=valore`, max un segmento (32 KiB). Chiavi: `graph.encoded_values`,
  `graph.flag_encoders`, `graph.dimension`, `graph.ch.profiles`, `datareader.import.date`, `datareader.data.date`,
  `prepare.ch.done`, `graph.profiles.<p>.version`.
- Versioni (`util/Constants`): NODE=8, EDGE=21, SHORTCUT=8, GEOMETRY=6, LOCATION_IDX=5, STRING_IDX=6.

## 2. BaseGraph (`storage/BaseGraphNodesAndEdges.java`, `BaseGraph.java`, `TurnCostStorage.java`, `search/StringIndex.java`)

**Record nodo** (`nodes`): `N_EDGE_REF@0` (testa lista archi, -1 = nessuno) | `N_LAT@4 = (int)(lat*1e7)` troncato |
`N_LON@8` | `N_ELE@12` solo con altimetria, `(int)(ele*1000)` mm | `N_TC` solo con costi di svolta (testa lista in `turn_costs`).
Dimensioni: 12 (2D), 16 (3D o turn costs), 20 (3D + turn costs).
Header: [0] versione, [4] entryBytes, [8] nodeCount, [12..24] bbox minLon,maxLon,minLat,maxLat, [28]/[32] minEle/maxEle, [36] frozen.

**Record arco** (`edges`): `E_NODEA@0` | `E_NODEB@4` | `E_LINKA@8` (arco successivo nella lista di nodeA) |
`E_LINKB@12` | `E_FLAGS@16` (`intsForFlags` int) | `E_DIST` = `round(dist*1000)` int in mm, clamp a `Integer.MAX_VALUE/1000` m |
`E_GEO` (riferimento geometria, unità int) | `E_NAME` (puntatore StringIndex). Dimensione = 28 + 4·intsForFlags.

**Adiacenza**: lista concatenata singola per nodo, inserimento in testa:
`linkA = edgeRef(nodeA); edgeRef(nodeA) = edge; se nodeA != nodeB: linkB = edgeRef(nodeB); edgeRef(nodeB) = edge`.
Iterando da `base`: se `base == nodeA` → `adj = nodeB`, `reverse=false`, prossimo = linkA; altrimenti `adj = nodeA`,
`reverse=true`, prossimo = linkB. Chiave arco = `edge<<1 | reverse`.

**Geometria** (`geometry`): `maxGeoRef` parte da 4; geoRef 0 = nessun pillar. Record a byte `geoRef*4`:
int count, poi count × (lat int, lon int[, ele int]) in ordine nodeA→nodeB. Se scritta da iteratore invertito,
la PointList viene invertita prima. Modalità lettura: TOWER_ONLY, PILLAR_ONLY, BASE_AND_PILLAR, PILLAR_AND_ADJ, ALL.

**Costi di svolta** (`turn_costs`, 16 byte): `FROM_EDGE@0`, `TO_EDGE@4`, `FLAGS@8`, `NEXT@12` (-1 fine).
Lista per nodo via, coda in fondo; coppia esistente → OR dei flag. `TurnCost.create(name, max)`:
`UnsignedDecimalEncodedValue("<enc>$turn_cost", bits=countBitValue(max), factor 1, default 0, useMaximumAsInfinity=true)`.

**StringIndex** (`string_index_keys`, `string_index_vals`): nome arco come `{"name": "<name>"}`; puntatore da 1 (0 = vuoto);
u8 count; per chiave: short keyIdx, u8 len, bytes; valori duplicati: short `-keyIdx` + int delta. Stringhe troncate a 255 byte.
Il nome è `name:<lang>` o `name`, più `", " + ref`, con `;` → `, `.

**CHStorage** (`nodes_<type>_<name>`, `shortcuts_<type>_<name>`, type = "ch" | "core"): shortcut = nodeA<<1|fwd, nodeB<<1|bwd,
weight int (×1000), skip1, skip2, [origFirst, origLast se edge-based], [time int se core — ORS]. Nodo: level, lastShortcut.
ORS aggiunge `coreNodeCount` nell'header dei nodi [12].

## 3. Valori codificati (`routing/ev/*`, `routing/util/EncodingManager.java`)

Allocatore (`InitializerConfig`):
```java
int dataIndex=-1, shift=32, nextShift=32, bitMask=0;
void next(int usedBits){ shift=nextShift;
  if ((shift-1+usedBits)/32 > (shift-1)/32) { dataIndex++; shift=0; }
  bitMask = (int)((1L<<usedBits)-1) << shift; nextShift = shift+usedBits; }
```
- Valori impacchettati in sequenza, mai a cavallo di un int da 32 bit. `intsForFlags = ceil(requiredBits/32)`.
- Due direzioni: `next(bits)` due volte (fwd poi bwd), ciascuna con proprio dataIndex/shift/mask.
- Get: `(ints[idx] & mask) >>> shift`; set: azzera i bit e OR di `value << shift`. Bit ammessi 1..31.
- Decimal: `round(v/factor)`; `value == defaultValue` → memorizza 0; 0 letto → `defaultValue`; con `useMaximumAsInfinity` il massimo = +∞.
- Boolean: 1 bit. Enum: `bits = 32 - nlz(n-1)`, memorizza l'ordinale. String: indice+1, 0 = null.
- Ordine di allocazione in `Builder.build()`: parser di relazione espliciti → `graph.encoded_values` → EV espliciti
  (`<profile>_subnetwork`) → default (roundabout, road_class, road_class_link, road_environment, max_speed 5 bit×5 due direzioni, road_access)
  → bike/foot network, get_off_bike, smoothness → encoder (`<enc>$access` bool 2 dir, `<enc>$average_speed` 5 bit ×5, ORS `conditional_access/speed`)
  → turn-cost in config separata. I flag di relazione sono sempre 2 int.
- Precisione dimensioni: max_height 11 bit ×0.01, max_weight 11 bit ×0.1, max_axle_load 9 bit ×0.1, max_width (fork) 10 bit ×0.01; default +∞.

## 4. Import OSM (`reader/osm/WaySegmentParser.java`, `OSMNodeData.java`, `OSMReader.java`)

Sequenza `GraphHopper.process`: `importOSM` → `cleanUp` (sottoreti) → `postImport` (interpolazione ponti/gallerie/traghetti) →
`postProcessing` (indice spaziale, ORS `postProcessingHook`, LM, CH) → `flush`.

**Passata 1 (way e relazioni)**: `acceptWay` richiede ≥ 2 nodi, tag, accettazione da almeno un encoder.
Id nodo in `GHLongIntBTree`: `END_NODE=0`, `INTERMEDIATE=1`; seconda occorrenza → `CONNECTION_NODE=2` (END+END) o `JUNCTION_NODE=-2`.
Relazioni: route → flag per way membro; restriction → from/to way id in `osmWayIdSet`.

**Passata 2, nodi**: JUNCTION e CONNECTION diventano torri (id sequenziale, codice `-tower-3`); END e INTERMEDIATE pillar in
`tmp_pillar_info` (codice `pillar+3`). Altimetria da `eleProvider.getEle(node)`. Nodi barriera (ORS: bloccanti per almeno un encoder)
tengono i tag salvo che siano incroci. Hook ORS `processNode`.

**Passata 2, way**: lista di `SegmentNode` → `preprocessWay(first, last, way)` (ORS: sidewalk, tag dei nodi sulla way, `onProcessWay`,
distanza, paese) → `splitWayAtJunctionsAndEmptySections` (taglio alle torri; nodi mancanti tagliano) → `splitLoopSegments`
(loop di 2 nodi scartato; loop chiuso diviso in [0..n-2] e [n-2..n-1]) → `splitSegmentAtSplitNodes` (nodo barriera copiato con
id OSM negativo artificiale → arco di lunghezza zero che porta i tag del nodo) → `handleSegment` (pillar agli estremi diventano torri).

**`OSMReader.addEdge`**: smoothing altimetrico opzionale (media entro 150 m) → campionamento archi lunghi (3D) → Douglas-Peucker
(default 1 m, blocchi di 500 punti, opzionale 3D) → distanza haversine con clamp [0.001 m, (MAX_INT-1)/1000] →
`setArtificialWayTags`: `estimated_distance`, `duration:seconds`, `country`, `country_rule`, `custom_areas` (AreaIndex JTS STRtree sul
punto medio) → `acceptWay` → `handleWayTags` → tag dei nodi: `isBarrier` azzera accesso fwd/bwd → crea arco → pillar `pointList[1..n-2]`
→ `applyWayTags` (nome) → mappa arco→way id se in una restrizione → ORS `onProcessEdge`.

**Divieti di svolta**: `restriction` / `restriction:<veicolo>` + `except`. NOT = no_left_turn, no_right_turn, no_straight_on, no_u_turn,
no_entry; ONLY = only_right_turn, only_left_turn, only_straight_on. Solo via **nodo** (torre). NOT → +∞ su (from, via, to);
ONLY → +∞ su ogni uscita con `wayId != to` e `edge != from`.

**Interpolazione altimetrica** (`reader/dem/*Interpolator`): per `road_environment ∈ {TUNNEL, BRIDGE, FERRY}`; BFS sugli archi
interpolabili; quota dei nodi interni da 1 (copia) / 2 (lineare) / 3 (piano) / >3 (inverso del quadrato della distanza) nodi esterni.

## 5. LocationIndexTree e QueryGraph

- `IndexStructureInfo.create(bounds, minRes=300 m)`: `maxDist = max(dLat/360·C, dLon/360·circ(min|lat|))`, `tmp = (maxDist/minRes)²/4`;
  finché `tmp > 1`: 16 se ≥16, altrimenti 4; infine 4. Chiave Morton (lon bit pari, lat bit dispari).
- Costruzione: ogni segmento di ogni arco rasterizzato con Amanatides–Woo; id arco nelle foglie, dedup consecutivi.
- Formato (`location_index`, radice a int 1): header [0]=5, [4]=checksum `nodes ^ edges`, [8]=minRes. Foglia con 1 arco: `-(edge)-1`;
  con n>1: `endPtr` seguito dagli id.
- `findClosest`: anelli di celle a distanza di Chebyshev 0..3; `traverseEdge` → TOWER (entro 0.1 m), EDGE (proiezione interna al segmento),
  PILLAR; distanza piana normalizzata; stop anticipato se best < `calculateRMin`. Snap: `closestNode`, `closestEdge`, `wayIndex`, `Position`.
- QueryGraph: snap su TOWER non crea nulla; altrimenti nodo virtuale `baseNodes + k` con 4 stati `k*4 + {BASE_SNAP, SNAP_BASE, SNAP_ADJ, ADJ_SNAP}`,
  id arco virtuale `baseEdges + idx/2`; i nodi reali adiacenti hanno una lista di adiacenza sostitutiva; u-turn ai nodi virtuali = +∞.

## 6. Sottoreti (`routing/subnetwork/PrepareRoutingSubnetworks.java`)

`prepare.min_network_size` default 200. `EdgeBasedTarjanSCC` sulle chiavi arco; ogni componente tranne la più grande con
meno di `2*minNetworkSize` chiavi viene marcata (bit `<profile>_subnetwork`, una direzione). Gli archi **non** vengono cancellati:
`DefaultSnapFilter` esclude quelli marcati o non finiti in nessuna direzione. Se oltre metà degli archi viene marcata → eccezione.

## 7. Altimetria (`reader/dem/*`)

Tile in `GHDirectory(cacheDir, MMAP)`, short a `2*(y*w+x)`, riga 0 = nord; valori fuori [-1000, 12000] → `Short.MIN_VALUE`;
nearest-neighbour o bilineare. Provider: SRTM (1°, 1201²), SRTMGL1 (3601), Skadi (3601, ±90), CGIAR (5°, 6000², GeoTIFF), GMTED;
`MultiSourceElevationProvider` = CGIAR tra -56 e 60, altrimenti GMTED.

## 8. Modifiche del fork rilevanti per lo storage

Nuove classi: `ConditionalEdges`, `ConditionalEdgesMap`, `ConditionalIndex`/`NameIndex` (file `conditional_access[_<enc>]`,
`conditional_speed[_<enc>]`), `GraphExtension`/`ExtendedStorageSequence`/`GraphStorageFactory` (storage plugin ORS), `TDDijkstra`, `TDAStar`,
`TimeDependentAccessWeighting`, `SpeedCalculator` e derivati, `ConditionalSpeedInspector`.
Modifiche: `Access` con PERMITTED/RESTRICTED; `Weighting` con `edgeEnterTime`; CHStorage tipo "core" con campo time e `coreNodeCount`;
hook in `GraphHopper` (`createOSMReader`, `loadORS`, `postProcessingHook`, …); `MaxWidth` a 10 bit ×0.01; `ReaderElement.getTagValues`
con valori multipli `;`; `SPTEntry.originalEdge`; fuso orario fisso Europe/Berlin in `DateTimeHelper`.
