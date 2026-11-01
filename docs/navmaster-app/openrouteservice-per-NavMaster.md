# openrouteservice → NavMaster Truck

Analisi della logica di calcolo di openrouteservice (ORS) e di come riusarla in NavMaster.

| | |
|---|---|
| **Sorgente analizzata** | `GIScience/openrouteservice`, commit `52fee39` (21/10/2026), versione `10.2.0-SNAPSHOT`. La cartella `Downloads/openrouteservice-main` è identica, file per file. |
| **Dimensioni** | Circa 66.000 righe Java in 5 moduli Maven |
| **Applicazione di destinazione** | `Desktop/NavMaster`: un fork Android di OsmAnd ("NavMaster Truck") personalizzato da `navmaster/apply.sh` al momento della build |

> **Metodo.** Il repository contiene solo sorgenti, nessun `.jar` o `.class`. Gli strumenti REA servono per artefatti compilati, quindi ho letto direttamente il codice, che è più preciso. Le affermazioni più importanti le ho verificate a mano sui file.

---

## 1. Sintesi

1. **ORS è un server Java 25 / Spring Boot 4 costruito sopra un fork di GraphHopper 4.16.** Importa un file OSM `.pbf`, prepara un grafo (CH, Core-ALT, Landmarks) e risponde via REST su `/ors/v2/{directions|isochrones|matrix|snap|export|match}/{profile}`.
2. **Il profilo che conta per NavMaster è `driving-hgv`.** I limiti del camion si scelgono a ogni richiesta: altezza, larghezza, lunghezza, peso, carico per asse, merci pericolose (hazmat), più il tipo di veicolo (hgv, bus, delivery, …).
3. **Non si può incorporare ORS dentro l'app Android.** Richiede Java 25 e Spring Boot, grafi da centinaia di MB fino a decine di GB, molta RAM, GeoTools e l'SDK AWS. Il modo corretto è un **server ORS** (self-hosted in Docker oppure l'API pubblica HeiGIT) interrogato dall'app.
4. **OsmAnd contiene già un client ORS** (`OsmAnd/src/net/osmand/plus/onlinerouting/engine/OrsEngine.java`), che supporta `driving-hgv`. Però **invia solo le coordinate**: i limiti del camion configurati nel profilo NavMaster vengono ignorati, e un camion da 4 m può essere mandato sotto un ponte da 3,5 m.
5. **Ho preparato una patch** ([navmaster_ors_patch.py](navmaster_ors_patch.py)) nello stesso stile di `apply.sh`.
   - Cosa fa:
     - prende altezza, larghezza, lunghezza, peso e hazmat dal profilo TRUCK di OsmAnd;
     - li invia a ORS come `options.profile_params.restrictions`, insieme ad `avoid_features` per pedaggi, traghetti e autostrade;
     - chiede le istruzioni in italiano;
     - corregge le rotonde, che ora riportano il numero dell'uscita.
   - Stato delle verifiche:
     - si applica senza errori ai sorgenti attuali di OsmAnd (prova a secco eseguita);
     - **non** l'ho compilata, perché serve la build Android completa.

---

## 2. Com'è fatta NavMaster (per capire dove agganciarsi)

- **Come viene costruita l'app**
  - `fase1_setup_osmand.sh` clona OsmAnd e OsmAnd-resources.
  - `navmaster-build.yml` (GitHub Actions) esegue `apply.sh`, poi `./gradlew assembleNightlyFreeLegacyFatDebug`, quindi fa gli screenshot nell'emulatore e pubblica l'APK.
- **Personalizzazioni applicate da `apply.sh` → `patch.py`**: patch testuali su upstream.
  - nome dell'app e icone
  - stile mappa `navmaster.render.xml`
  - barra di manovra verde
  - widget per il camion
  - ricerca per indirizzo
  - profilo TRUCK predefinito
  - `JunctionViewLayer` per la vista degli svincoli
  - default di navigazione per il camion
- **Routing di oggi:** NavMaster usa il motore **offline** di OsmAnd (`BinaryRoutePlanner` con il profilo `truck` di `routing.xml`).
- **Parametri camion che l'utente già imposta in OsmAnd** (`routing.xml`): `height`, `width`, `length`, `weight`, `hazmat_category`, `avoid_toll`, `avoid_ferries`, `avoid_motorway`. Sono salvati per profilo con `OsmandSettings.getCustomRoutingProperty(...)`.

La cartella `iGO_PAL` (contenuti, licenze e codici di attivazione di iGO) è software di terze parti e non fa parte del codice di NavMaster. Non l'ho analizzata e non va usata in questa integrazione.

---

## 3. Come calcola openrouteservice

### 3.1 Pipeline complessiva

```
 OSM .pbf ──► ORSOSMReader ──► TagParser / EncodedValue ──► Grafo GraphHopper
 (+ altimetria SRTM, confini, CSV)        │                       │
                                          ▼                       ▼
                     storage extra (green/noise/shadow/traffic)   Preparazione:
                                                                  CH · Core(+LM) · LM · FastIsochrones
                                                                          │
 Richiesta REST ──► DTO + validazione ──► RoutingRequest ──► scelta algoritmo ──► Peso + filtri archi
                                                                          │
                                         ExtraInfoProcessor ◄── percorso ◄┘
                                                │
                     RouteResultBuilder (geometria, segmenti, manovre, istruzioni i18n) ──► JSON/GeoJSON/GPX
```

### 3.2 Costruzione del grafo (una volta, offline)

**Avvio.** `RoutingProfileManager` carica i profili attivi in parallelo, con `init_threads` thread. Per ogni profilo, `RoutingProfile.initGraphHopper()` esegue questi passi:
1. **Gestione grafi** (`ORSGraphManager`): se è configurato un repository (HTTP, filesystem o S3), scarica il grafo precostruito `.ghz` invece di ricalcolarlo.
2. **Configurazione**: `ORSGraphHopperConfig` crea i profili GraphHopper `<veicolo>_{fastest,shortest,recommended}[_with_turn_costs]`.
3. **Import del file OSM** (`ORSOSMReader`):
   - copia i tag dei nodi sulla way (per esempio un `maxheight` messo sul nodo di un ponte);
   - calcola l'indice di pendenza dall'altimetria;
   - assegna il paese tramite i poligoni dei confini.
4. **Codifica dei tag in attributi per arco** (encoded values). Quelli rilevanti per il camion:
   - **dimensioni**: `max_height`, `max_width`, `max_length`, `max_weight`, `max_axle_load`
   - **accesso per tipo di veicolo**: `hgv_access`, `bus_access`, `goods_access`, `delivery_access`, …
   - **merci pericolose**: `hazmat_access`
   - **classificazione e tipo di strada**: `highway`, `way_type`, `way_surface`
   - **altro**: `toll`, `border`, `country`

   Il codice è in `ors-engine/.../config/profile/BuildProperties.java:100-291` e nei parser di `.../graphhopper/extensions/util/parsers/`.
5. **Velocità camion** (`HeavyVehicleFlagEncoder`):

   | Tipo di strada | Velocità (km/h) |
   |---|---|
   | motorway / trunk | 80 |
   | primary | 60 |
   | link (rampe) | 50 |
   | track grade1 → grade5 | 40 → 10 |

   Massimo assoluto: 90 km/h. Con `use_acceleration` viene modellata anche l'accelerazione. I limiti per paese sono in `resources/services/routing/speed_limits/*.json`.
6. **Preparazione** (`ORSGraphHopper.postProcessing`): CH, poi **Core**, poi Core-Landmarks, poi isocrone veloci (se configurate).

### 3.3 L'idea chiave di ORS: il "Core" (core-ALT)

**Il problema.** Le Contraction Hierarchies (CH) sono velocissime ma "congelano" i pesi. Non si possono applicare a ogni richiesta i limiti di altezza o di peso di *quel* camion.

**La soluzione di ORS** (`.../extensions/core/PrepareCore`):
1. Durante la contrazione, tutti gli archi "problematici" vengono marcati come **core**, cioè restano non contratti. Sono gli archi con restrizioni HGV, pedaggi, confini, condizionali, traffico, velocità oltre `maximum_speed_lower_bound`, o svolte vietate.
2. I filtri usati per marcarli: `HeavyVehicleCoreEdgeFilter`, `AvoidFeaturesCoreEdgeFilter`, `AvoidBordersCoreEdgeFilter`, `TurnRestrictionsCoreEdgeFilter`, …
3. Sugli archi del core il peso usato in contrazione è infinito, quindi nessuna scorciatoia CH li attraversa.

**La ricerca** (`AbstractCoreRoutingAlgorithm`) procede in due fasi:
1. Ricerca CH ascendente fuori dal core: velocissima, valida per qualunque camion.
2. Dentro il core: A* bidirezionale con landmark (`CoreALT`), applicando **solo lì** i filtri della richiesta.

**Risultato:** routing per camion con dimensioni arbitrarie a una velocità vicina a quella delle CH.

### 3.4 Scelta dell'algoritmo per ogni richiesta (`RoutingRequest.java:386-516`, `TemporaryUtilShelter.getFlexibilityMode`)

| Situazione della richiesta | Modalità | Algoritmo |
|---|---|---|
| Nessuna opzione speciale | STATIC | CH (`dijkstrabi`), altrimenti Core, altrimenti ALT |
| Restrizioni camion, `vehicle_type` diverso da hgv, `avoid_features` / borders / countries, `maximum_speed` | PREPROCESSED | **Core-ALT** |
| `avoid_polygons`, bearings, percorsi alternativi, custom model, soft weightings | NON_DECREASING | solo ALT (A* con landmark) |
| Partenza o arrivo a un orario dato, con condizionali o traffico | time-dependent | `td_astar` |
| Round trip | | CH e Core disattivati, algoritmo `round_trip` |

### 3.5 Peso (costo) e filtri (`ORSWeightingFactory`, `ORSEdgeFilterFactory`)

**Il peso viene costruito a strati:**

| Strato | Cosa fa |
|---|---|
| Costi di svolta | Applicati se il profilo li prevede (`DefaultTurnCostProvider`) |
| Peso base | `fastest` (tempo, `ORSFastestWeighting`), `shortest` (distanza), oppure `custom` |
| `recommended` | Moltiplica il peso per 2^(priorità della strada) (`ORSPriorityWeighting`). Per l'HGV, "fastest" e "recommended" diventano entrambi `recommended` |
| Soft weightings (opzionali) | green, quiet, shadow, steepness, avoid_hills, heat stress |
| `LimitedAccessWeighting` (sempre) | Penalità ×10 per `access=destination` / `private`, ×1,5 per `customers`, ×1,2 per `highway=service` |

**Filtri sugli archi.** Un arco escluso non viene mai attraversato. Per il camion, `HeavyVehicleEdgeFilter.accept()` scarta l'arco in tre casi:
- il tipo di veicolo (hgv, bus, …) non ha accesso;
- `hazmat=true` e l'arco vieta le merci pericolose;
- per ciascuna dimensione, il limite dell'arco è maggiore di 0 **e** minore del valore del camion (per esempio `maxheight` 3,5 con camion da 4,0).

Gli altri filtri sono `AvoidFeaturesEdgeFilter` (autostrade, pedaggi, traghetti, guadi), `AvoidBordersEdgeFilter` e `AvoidAreasEdgeFilter` (poligoni).

### 3.6 Dal percorso alla risposta

1. `ExtraInfoProcessor` aggiunge le informazioni per tratto richieste con `extra_info`: `tollways`, `waycategory`, `surface`, `steepness`, `roadaccessrestrictions`, `countryinfo`, …
2. `RouteResultBuilder` costruisce segmenti, passi e manovre, con 18 tipi di istruzione (0 = svolta a sinistra … 7 = entra in rotonda con `exit_number`, 12/13 = mantieni sinistra/destra).
3. I testi delle istruzioni sono localizzati da `resources/locales/ors_*.resources`. L'italiano c'è (`ors_it-IT`).

### 3.7 Gli altri servizi (utili per funzioni future di NavMaster)

| Servizio | Logica | Uso possibile in NavMaster |
|---|---|---|
| **Matrix** `/v2/matrix` | RPHAST su CH. Con opzioni camion usa una matrice Core (CH fuori dal core, Dijkstra molti-a-molti dentro). Altrimenti un Dijkstra per ogni origine | Ordinare le consegne, scegliere il deposito più vicino |
| **Isochrones** `/v2/isochrones` | Dijkstra limitato al costo massimo, poi una nuvola di punti con buffer, poi l'inviluppo concavo JTS. In alternativa le *fast isochrones*: partizionamento Inertial Flow con max-flow Edmonds-Karp, celle, eccentricità precalcolata | "Dove arrivo entro le ore di guida residue", con le restrizioni camion |
| **Snap** `/v2/snap` | Ricerca della strada più vicina con filtro sottorete | Agganciare i punti GPS |
| **Match** `/v2/match` | Map matching HMM di GraphHopper | Ricostruire le tracce GPS |
| **Export** `/v2/export` | Grafo dentro un bounding box | Debug e analisi |

---

## 4. Come applicarlo a NavMaster: opzioni

| | A. Server ORS e client OsmAnd esteso **(consigliata)** | B. API pubblica HeiGIT | C. Portare la logica nel motore offline di OsmAnd |
|---|---|---|---|
| **Lavoro** | Basso: la patch è già pronta più un server Docker | Minimo: solo una chiave API | Alto |
| **Funziona offline** | No | No | Sì |
| **Restrizioni camion** | Complete e configurabili | Complete | Dipendono dai dati in `routing.xml` |
| **Controllo** | Totale (limiti, area geografica, aggiornamenti) | Quote giornaliere e limiti di distanza imposti dal servizio | Totale |
| **Licenze** | Nessun vincolo per l'app: comunica via HTTP | Rispettare i termini HeiGIT | OsmAnd è GPLv3, ORS è LGPL/GPL: compatibili, ma il codice derivato resta GPL |

**Raccomandazione:** partire con **A**, usando B solo per i primi test, e tenere il motore offline di OsmAnd come **ripiego** quando manca la rete. OsmAnd ha già la logica "online engine con approssimazione", quindi la navigazione passo-passo, la voce e il ricalcolo restano quelli di NavMaster.

Dall'opzione C conviene comunque riprendere tre idee concrete:
1. La tabella delle velocità per camion (§3.2 punto 5).
2. Il trasferimento dei tag `maxheight`/`maxweight` dai nodi alle way. In OSM i ponti hanno spesso il limite sul nodo.
3. Le penalità `LimitedAccessWeighting` (×10 per `destination`/`private`).

---

## 5. Piano di integrazione (opzione A)

### Passo 1: server ORS per camion

`docker-compose.yml` del repository ORS (porta 8080→8082). Esempio di `ors-config.yml`:

```yaml
ors:
  engine:
    profile_default:
      build:
        source_file: /home/ors/files/italy-latest.osm.pbf   # da download.geofabrik.de
      service:
        maximum_distance: 3000000                  # predefinito 100 km!
        maximum_distance_dynamic_weights: 3000000  # le restrizioni camion usano questo limite
        maximum_distance_avoid_areas: 3000000
    profiles:
      driving-hgv:
        enabled: true            # predefinito: CH + Core(recommended,shortest) + HeavyVehicle(restrictions: true)
```

> ⚠️ **Limite di distanza.** Di default ORS rifiuta i percorsi oltre **100 km**: `maximum_distance` e `maximum_distance_dynamic_weights` valgono 100000 in `application.yml`. Per un camion va alzato, altrimenti si ottiene l'errore 2004.

Controlli dopo l'avvio:
- `GET http://server:8080/ors/v2/health` deve rispondere `{"status":"ready"}`. Finché il grafo non è pronto risponde 503 e le richieste restano in attesa.
- `GET /ors/v2/status` mostra i profili, i limiti e la data OSM.

Note operative:
- Per l'Europa intera servono molta RAM e un tempo di build lungo: si comincia dall'Italia.
- Per gli aggiornamenti periodici si usa il *graph repository* (`graph_management`) oppure si ricostruisce con `REBUILD_GRAPHS=True`.
- Il server deve stare dietro HTTPS con un reverse proxy, e serve una chiave: ORS self-hosted non ha né autenticazione né limiti di frequenza.

### Passo 2: patch NavMaster ([navmaster_ors_patch.py](navmaster_ors_patch.py))

Il blocco va incollato in `patch.py` dentro `apply.sh`, prima del `print()` finale. Esempio della richiesta che produce:

```json
POST {url}/driving-hgv/geojson
{
  "language": "it",
  "coordinates": [[9.19,45.46],[11.34,44.49]],
  "options": {
    "vehicle_type": "hgv",
    "profile_params": { "restrictions": { "height": 4.0, "width": 2.5, "length": 12, "weight": 40, "hazmat": true } },
    "avoid_features": ["tollways"]
  }
}
```

Parametri del profilo OsmAnd usati:

| Profilo TRUCK in OsmAnd | Opzione inviata a ORS |
|---|---|
| `height` / `width` / `length` (m) | `restrictions.height` / `width` / `length` |
| `weight` (t) | `restrictions.weight` |
| `hazmat_category` diverso da 0 | `restrictions.hazmat: true` |
| `avoid_toll` / `avoid_ferries` / `avoid_motorway` | `avoid_features`: `tollways` / `ferries` / `highways` |

Altre modifiche della patch:
- **rotonde**: il tipo 7 con `exit_number` diventa `TurnType.getExitTurn(n, …)`, quindi la voce dice "prendi la 2ª uscita";
- **istruzioni** in italiano.

Il client OsmAnd ha anche altri limiti, che la patch non tocca:
- il tipo 14 nel client è mappato come UNKNOWN, ma ora in ORS è PT_ENTER (innocuo per il camion);
- `axleload` non è un parametro di OsmAnd;
- per i valori in unità imperiali (piedi/libbre) va verificata la conversione.

### Passo 3: configurazione nell'app

1. In NavMaster: *Impostazioni → Navigazione → Tipo di navigazione → Motore online → openrouteservice*.
2. Impostare l'URL del server (`https://server/ors/v2/directions/`), il veicolo HGV e l'eventuale chiave.
3. Per farlo diventare il default, si può aggiungere un'altra patch in `apply.sh` che registri il motore al primo avvio, come fa `startOsrmEngine()` in `OnlineRoutingHelper`.

### Passo 4: verifica

Fare un percorso di prova con un ponte basso noto: con `height` 4,0 il percorso deve cambiare rispetto a `height` 0.

---

## 6. Difetti trovati in ORS (verificati sul codice)

Utili da sapere se si gestisce un'istanza propria.

| Area | Difetto | Dove | Effetto |
|---|---|---|---|
| Matrix | `new StatusCodeException(MatrixErrorCodes.UNKNOWN)` usa 6099 come *codice HTTP* | `ors-api/.../services/MatrixService.java:51` | Un errore imprevisto nella matrice fa fallire anche il gestore degli errori |
| Isochrones | Le intersezioni di secondo livello vengono create e poi scartate (manca `result.add`) | `ors-engine/.../isochrones/IsochroneUtility.java:62-76` | Le intersezioni multiple risultano sempre vuote |
| Matrix | `maximum_routes_flexible` non viene mai applicato (sempre `getMaximumRoutes(false)`) | `MatrixService.java:134`, `requests/matrix/MatrixRequest.java:128` | Il limite di 25 per le matrici "flessibili" è ignorato |
| API | Il flag `endpoints.*.enabled` compare solo in `/status` | controller | Non si può davvero disattivare un endpoint |
| Health | `/v2/health` non controlla `hasFailed()` | `HealthAPI` | Un'inizializzazione fallita non viene segnalata come errore |
| Traffico | Il fuso orario è fisso su Europe/Berlin | `RoutingRequest.java:472` | Rilevante solo con i dati di traffico HERE |

---

## 7. Mappa del codice ORS (riferimento rapido)

| Modulo / package | Contenuto |
|---|---|
| `ors-api/.../controllers` | `RoutingAPI`, `IsochronesAPI`, `MatrixAPI`, `SnappingAPI`, `ExportAPI`, `MatchingAPI`, `HealthAPI`, `StatusAPI` |
| `ors-api/.../services` | Conversione DTO → richiesta del motore, validazione dei limiti, `EngineService` (avvio asincrono), `GraphService` (aggiornamento dei grafi a caldo) |
| `ors-api/.../errors` | `CommonResponseEntityExceptionHandler`. Codici di errore: routing 2xxx, isocrone 3xxx, matrice 6xxx, export 7xxx, snap 8xxx, match 9xxx |
| `ors-engine/.../routing` | `RoutingProfileManager`, `RoutingProfile`, `RoutingRequest`, `RouteResultBuilder`, istruzioni |
| `.../graphhopper/extensions` | `ORSGraphHopper`, `ORSOSMReader`, `ORSRouter`, weighting, edge filter, flag encoder, `core/` (Core-ALT), `manage/` (repository dei grafi) |
| `com.graphhopper.routing.ev` | Encoded values di ORS: accessi, hazmat, confini, superfici, sedia a rotelle, dati dinamici |
| `ors-engine/.../matrix`, `isochrones`, `fastisochrones` | RPHAST, matrice Core, inviluppo concavo, Inertial Flow |
| Configurazione | `ors-api/src/main/resources/application.yml` (default completi). Ordine di ricerca del file: argomento da riga di comando, `ORS_CONFIG_LOCATION`, `./ors-config.yml`, `~/.config/openrouteservice/`, `/etc/openrouteservice/` |
| Deploy | `Dockerfile` (immagini `publish` e `slim` distroless), `docker-entrypoint.sh`, `docker-compose.yml` |
| Test | Test unitari, `-Papitests` (Spring Boot con `heidelberg.test.pbf`), `ors-test-scenarios` (Testcontainers), `ors-benchmark` (Gatling) |
