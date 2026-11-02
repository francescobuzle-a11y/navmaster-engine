# navmaster-engine — architettura

Motore di routing scritto da zero in Kotlin che riproduce la logica di calcolo di
openrouteservice (ORS 10.2) e del suo fork di GraphHopper (GIScience v4.16), con l'obiettivo
di essere più compatto ed efficiente su scala continentale.

Le analisi dei due progetti sorgente sono in [`docs/analisi/`](analisi/).

## Vincoli di progetto

| Vincolo | Scelta |
|---|---|
| Deve girare sul server (continenti) **e** dentro l'app Android NavMaster (singole regioni) | Kotlin/JVM, target JVM 17, modulo `engine-core` senza dipendenze da server |
| Grafi da decine di GB | Storage a segmenti memory-mapped, nessun oggetto per nodo/arco, collezioni primitive (hppc) |
| Compatibilità con il client ORS già presente in OsmAnd | `engine-server` espone `/ors/v2/directions/{profile}` con lo stesso JSON/GeoJSON |
| Licenza | Il codice segue da vicino ORS/GraphHopper (LGPL/Apache): il progetto resta sotto licenza compatibile (LGPL-3.0) |

## Moduli

```
engine-core    grafo, import OSM, valori codificati, profili, pesi, algoritmi, servizi
engine-cli     import <pbf> <graph-dir>, route, benchmark
engine-server  HTTP (Ktor) compatibile ORS v2
```

## Strati di `engine-core`

```
 osm/        OsmWay, OsmValues (parsing tag), lettore PBF (2 passate)
 profile/    VehicleProfile → CarProfile, HgvProfile: accesso, velocità, priorità, barriere
             WayAttributeParser → WayAttributes (dimensioni, accessi hgv, hazmat, pedaggi, …)
             SpeedTables (generate dai JSON di ORS)
 storage/    DataAccess (RAM / MMAP, segmenti, little-endian)
             EncodedValue* (bit packing per arco, due direzioni)
             BaseGraph (nodi, archi, geometria, bbox), TurnCostStorage, LocationIndex
 weighting/  Fastest, Shortest, Priority(recommended), LimitedAccess, filtri archi (HGV, avoid_*)
 routing/    Dijkstra/A* bidirezionali, CH, Core (core-ALT), Landmarks, istruzioni
 api/        modello richiesta/risposta ORS, codici errore
```

## Decisioni che differiscono da GraphHopper/ORS (per efficienza)

1. **Un solo formato di file** con intestazione fissa e versione (`GRAPH_FORMAT_VERSION`); i dati sono
   little-endian anche nell'intestazione (GraphHopper mescola big e little endian).
2. **Coda di priorità con decrease-key** (heap indicizzato su array) invece di `PriorityQueue` con
   `remove()` lineare.
3. **Nessuna allocazione per nodo visitato**: le ricerche usano array paralleli (peso, genitore,
   arco) indicizzati per nodo/arco invece di oggetti `SPTEntry`.
4. **Core-ALT nativo**: il "core" è un attributo del formato CH, non un adattamento a posteriori.
5. **Costruzione parallela** del grafo e delle preparazioni per regione.

Ogni scelta va misurata: il modulo `engine-cli` include un benchmark riproducibile e i confronti
con ORS vanno documentati con numeri, non promesse.

## Fasi

| Fase | Contenuto | Stato |
|---|---|---|
| 1a | Tag OSM, tabelle velocità, profili auto/camion, attributi per arco | ✅ |
| 1b | Storage: DataAccess (RAM/MMAP), valori codificati, BaseGraph, geometria, nomi | ✅ |
| 1c | Import PBF a due passate, torri/pillar, barriere sdoppiate, raccolta divieti di svolta | ✅ Heidelberg: 221k nodi OSM → 17k nodi / 21k archi in 0,7 s |
| 1d | Indice spaziale a griglia, snapping, pesi fastest/shortest/recommended + accesso limitato, filtri HGV e avoid_features, Dijkstra bidirezionale, ricostruzione percorso | ✅ 52 test |
| 1e | Istruzioni (porting di `InstructionsFromEdges`: svolte, rotatorie con uscite, keep left/right, inversioni), passi ORS con testi it/en e manovre, via-point, risposta JSON/GeoJSON con `metadata`, codici errore 2xxx, server Ktor `/ors/v2/directions|health|status` | ✅ compatibile con il client ORS di OsmAnd |
| 1f | Divieti di svolta (turn cost storage, ricerca edge-based, u-turn vietati) | ✅ Heidelberg: 453/476 relazioni applicate, 3 percorsi su 12 cambiano |
| 1g | Via-point, bearings, percorsi alternativi, round trip | |
| 2a | Contraction Hierarchies node-based: preparazione (ordinamento euristico GraphHopper), storage, query con stall-on-demand, scompattamento | ✅ Heidelberg: 10.059 shortcut, preparazione 0,15 s, query 0,05 ms vs 2,7 ms (−98 % nodi visitati), pesi identici |
| 2b | Core: archi con restrizioni, pedaggi, autostrade, velocità > 80, divieti di svolta non contratti; query CH fuori dal core + Dijkstra filtrato dentro, node- o edge-based | ✅ Heidelberg: core 14 % dei nodi, pesi identici al Dijkstra filtrato (3 scenari), −76 % nodi visitati. Vedi [verifiche-stranezze.md](analisi/verifiche-stranezze.md) per i due errori trovati dai test |
| 2c | Landmark nel core (core-ALT): 16 landmark "farthest", distanze per i nodi del core, A* bidirezionale bilanciato in fase 2 con arresto esatto (`key_f + key_b ≥ best`); selezione algoritmo per richiesta come `setSpeedups` | ✅ Heidelberg: 229 nodi visitati vs 968 (Core) vs 4.081 (Dijkstra), pesi identici |
| — | Benchmark Lombardia (936.496 nodi, 1.127.802 archi; import 27 s, preparazioni ~80 s, 354 MB): percorsi di ~100 km, JVM calda, 130–150 coppie casuali. CH camion **1,1 ms**, CH auto **0,5 ms**; camion con restrizioni (4 m, 40 t) e divieti di svolta: Core+landmark **10,5 ms** (max 140), Core senza landmark 567 ms, Dijkstra edge-based 361 ms | ✅ |
| 2d | Integrazione nell'app NavMaster (`francescobuzle-a11y/navmaster`, Kotlin/MapLibre/Ferrostar) al posto di GraphHopper: jar unico (`androidJar`, protobuf/osmpbf rilocati), `NmEngine`/`NmRouting`/`OrsGuide` nell'app, pacchetti `nm.tar.gz` per Paese (`grafo-nm.yml`) | ✅ provato sul tablet (Milano→Bergamo, 1,7 s a freddo, 0,4 s a caldo). Il fork OsmAnd (build 98) resta solo come banco di prova |
| 2e | Dati di guida (file `guidance`, formato 3): limiti di velocità per verso e per camion, corsie e `turn:lanes`, destinazioni, ref, gallerie/ponti, senso unico, svincoli, caselli/portali/dogane; estensione API `navmaster.roads` e strade intorno a un punto | ✅ |
| 2f | Formato 4: autovelox/Tutor/fotocamere semaforo (file `points`, nodi e relazioni `enforcement`), categoria ADR, divieti a orario (`:conditional`), cartelli sulle corsie, relazioni `destination_sign`, altre strade all'incrocio (`branches`) e punti lungo il percorso in `navmaster.roads`; percorsi alternativi `alternative_routes` (plateau, edge-based con divieti di svolta); `import --mmap`; grafi per Paese via `grafo-nm.yml` dell'app | ✅ |
| — | Grafi dei 48 Paesi d'Europa (+ zona di prova) costruiti sui runner GitHub (`grafo-nm.yml`): con il core ridotto (solo restrizioni del mezzo e divieti di svolta, nodi che creerebbero > 200 shortcut lasciati nel core, niente core per `shortest`) Germania 46 min, Francia 63, Regno Unito 31, Italia 31, Spagna 32; il core alla ORS (autostrade/pedaggi/velocità nel core) esplodeva (Austria 15 M shortcut, 2 h). Risultati identici: `avoid_features`/`maximum_speed`/`shortest`+restrizioni usano la ricerca piena | ✅ 2026-11-02 |
| 2g | Viaggi tra Paesi: dall'app tramite `engine-server` con il grafo Europa (impostazione «Server NavMaster», vedi [server.md](server.md)); sul tablet resta un grafo per Paese | ✅ lato app; server da installare |
| 3 | Matrici (RPHAST), isocrone, snapping API | |
| 1h | `avoid_polygons`, `maximum_speed`, `extra_info` (tollways, waycategory, waytype, surface, roadaccessrestrictions, osmid, avgspeed), opzione `--no-width-from-carriageway`, chiave API, Docker | ✅ |
| — | Benchmark su estratto regionale/nazionale, confronto numerico con un'istanza ORS | Lombardia in corso |
| 4 | Profili bici/pedoni/sedia a rotelle, dati esterni (confini, altimetria), traffico | |

## Ambiente di sviluppo

Mac M1, 8 GB RAM, poco disco: i test usano `data/heidelberg.test.pbf` (2,4 MB, lo stesso di ORS).
I grafi nazionali/continentali vanno costruiti su un server.
