# TomTom GO (camion): motore di percorso, mappe offline e idee per NavMaster

Analisi del 15–16 novembre 2026 di TomTom GO 3.6.326 (`com.tomtom.gplay.navapp`, l'app che con
l'abbonamento camion diventa "GO Expert"), fatta con gli strumenti REA, `strings`/`c++filt` sulle
librerie native e la lettura di nomi, dimensioni e intestazioni dei file di mappa sulla tablet di
prova. Prende il posto della bozza del 15 novembre: ne conserva le parti verificate e ne corregge
due errori (il formato di compressione e il ruolo della modalità ibrida, vedi §2.2 e §1.5).

**Come leggere il documento.** Ogni affermazione è marcata:

- **[V]** verificato: si vede direttamente in un file, una stringa, una classe o un comando ripetibile;
- **[D]** dedotto: conclusione ragionevole da più prove, ma non osservata in esecuzione;
- **[?]** ignoto: non stabilito, con il motivo.

**Limiti rispettati.** Non ho decompresso né decifrato i dati di mappa: dei file NDS ho letto solo
nomi, dimensioni e i primi 112 byte (l'intestazione del contenitore). Nessun codice TomTom è stato
copiato: dai binari vengono solo nomi di classi, di tabelle e di parametri, cioè architettura e
vocabolario. I concetti NDS citati (building block, update region, livelli, tile) sono quelli
pubblici dello standard Navigation Data Standard 2.x.

---

## Sintesi

1. **Il router TomTom è un A\* "esagerato" su una gerarchia di livelli della mappa, non CH né
   landmark.** [V] Le classi del motore formano una catena di "policy" (restrizioni del veicolo,
   orari, traffico, consumi, costo delle svolte, evitamenti, *livellamento delle tile*) sopra una
   ricerca A\* a etichette, compilata con `TPEnableExaggeratedAStar<true>`. [D] Vicino a partenza e
   arrivo cerca su tutte le strade; lontano sale ai livelli che contengono solo le strade importanti
   (i livelli NDS). Non ci sono shortcut precalcolati: la gerarchia sta già nei dati.
2. **Le mappe sono database SQLite NDS 2.4.6 compressi a pagine (ZIPVFS) con zstd.** [V] Ogni regione
   di aggiornamento (UR) ha un file per tipo di dato: `ROUTE.NDS` (routing), `BMD.NDS` (mappa
   disegnata), `NAME`, `POI`, `FTS` (ricerca), `LHPLUS` (corsie), `TMC`, `SHARED`. I file di routing
   e mappa sono `ZV-zstd`, gli indici `ZV-lz4hc`. Il report del 10 novembre diceva "tutto LZ4": è
   sbagliato per i dati pesanti.
3. **Gli attributi camion sono un livello separato della tile di routing.** [V] Il motore legge
   `SELECT truckAttributeLayer FROM routingAuxTileTable WHERE id = ?`, a parte rispetto ai dati base
   della tile, ai nomi (`nameAttributeLayer`) e all'ADAS (`adasAttributeLayer`). Una regione senza
   quel livello fa scattare l'avviso "nessun attributo camion" e il ripiego sul percorso auto.
4. **Le restrizioni sono condizioni componibili.** [V] Peso, peso per asse, lunghezza, larghezza,
   altezza, tipo di veicolo, merci pericolose (classi + codice galleria ADR), fasce orarie e perfino
   condizioni meteo (`kAppliesDuringRain/Fog/Snow`). Il router ha una policy dedicata per le
   restrizioni del veicolo e una per il dominio orario.
5. **Ogni continente ha regioni "di collegamento" piccolissime.** [V] Per l'Europa sono `UR_900`,
   `UR_901` e `UR_902` (`ROUTE.NDS` da 1,8 MB, 2,0 MB e 49 KB), installate insieme a quelle del
   Paese. [D] Servono a calcolare viaggi lunghi anche attraverso Paesi non scaricati.
6. **Gli aggiornamenti sono incrementali per regione e per tipo di dato.** [V] Ogni versione scarica
   file `.SXZ` di poche centinaia di byte o pochi KB per `UR_xxx/PR_000nn` (nn = building block).
   Il prodotto Europa sulla tablet ha storico fino alla versione 156.
7. **Ordine di grandezza dello spazio.** [V] Sulla tablet la mappa TomTom installata pesa 954 MB in
   tutto. Le due regioni grandi (molto probabilmente l'Italia [D]) hanno `ROUTE.NDS` da 120 + 209 MB
   e circa 760 MB di dati complessivi. Per l'Italia NavMaster scarica `nm.tar.gz` da 2,24 GB
   (compresso) più mappa, indirizzi e POI: circa 4,7 GB.

Cosa conviene emulare, in ordine (dettagli in §5): merci pericolose complete nel routing; divieti a
orario nel routing; "evita" come penalità invece che come divieto; costi di svolta per il camion;
copertura dei dati camion dichiarata per Paese; dorsale europea leggera per i Paesi non scaricati;
file del grafo a blocchi compressi e aggiornabili.

---

## 1. Il motore di calcolo del percorso

### 1.1 Dove sta

- [V] Pacchetto: `com.tomtom.gplay.navapp` 3.6.326, versionCode 2147483647, installato sulla tablet
  R8YY402LRNP (REA `inspect_adb_package`, Evidence `ev_41f48666…`). La copia installata è la build
  rifirmata `DEBUGGABLE` preparata il 10 novembre (vedi `DEVICE_ANALYSIS.md`), con la stessa versione
  dell'APK originale. Per questo non ho riscaricato l'APK dalla tablet: avrebbe duplicato 350 MB su
  un disco al 96 %. Ho analizzato l'APK originale già presente
  (`~/Downloads/TomTom GO for Truck/TomTom GO.apk`, sha256 `f6e9ded7…`, inventario REA
  `inspect_artifact`: 10 327 voci, Evidence `ev_35c7526a…`).
- [V] Il routing è in `lib/arm64-v8a/libNavKit.so` (62,6 MB, sha256 `e86f81ce…`), senza simboli:
  esporta solo `NavKitMain`, `NavKitTerminate`, `NavKitVersion` e due funzioni di avvio. Le
  informazioni vengono dai nomi RTTI delle classi C++ (5 887 stringhe `N…E` demangolate) e dai
  percorsi dei sorgenti rimasti nelle asserzioni (`/__w/as-nk1-navkit/as-nk1-navkit/Routing/Engine/…`,
  231 percorsi).
- [V] REA non ha un decompilatore configurato per questa libreria (Ghidra e Hopper sono candidati ma
  l'analisi completa di 62 MB non è stata avviata; IDA e Kuna non sono configurati). Quindi niente
  pseudocodice: i pesi numerici e la logica interna restano **[?]**.

### 1.2 Struttura della ricerca

- [V] La ricerca è un template con una catena di policy. Una delle istanze, come appare nei nomi RTTI
  (semplificata, dall'esterno verso l'interno):

  ```
  CRouteSearchAlgo
   └ CActiveSchedulerPolicy → CStopPolicy → CReportingPolicy → CTargetDetectionPolicy
     └ CTileLeveling                      ← livelli della mappa
       └ CAttractingPolylinePolicy → CAvoidPolicy → CTurnCostPolicy
         └ {CFastestPolicy | CEcoPolicy | CDistanceBasedPolicy | CThrillingPolicy}
           └ CConsumptionPolicy → CHandsFreePolicy → CTrafficPolicy
             └ CVehicleRestrictionsPolicy → CTimeDomainPolicy
               └ CRouteSearchBase<DataAccess::NDS::CGuidanceData>
  opzioni: TPEnableExaggeratedAStar<true>; varianti con Tag::TBackwardSearch, Tag::TAcceleration, Tag::TUsePolyline
  ```

  Ogni criterio di pianificazione (veloce, eco, corto, "thrilling") è un'istanza compilata a parte
  dello stesso motore.
- [V] Le etichette: `CLabelHandling<CLabelResources<CGuidanceData, CLabelAStar<CLabelReachability<CLabel<CBaseLabel<CKeyTraits>, unsigned int>>>>>`.
  È una ricerca a etichette (label-setting) con A\*; le etichette vengono da un `CLabelDispenser`, con
  limiti di memoria (`HeapSoftLimit`, `LabelStorage`).
- [V] "A\* esagerato": stringhe `ExaggeratedTuning`, `factorForEstimatesOnMotorways`, `A*cost:`. [D]
  L'euristica viene gonfiata (A\* pesato): la ricerca è più veloce ma non garantisce il percorso
  ottimo. NavMaster fa il contrario (CH e core-ALT esatti).
- [V] `CThrillingPolicy` (percorsi tortuosi) è l'unica catena **senza** `CTileLeveling`. [D] Quel
  criterio cerca solo sul livello di dettaglio, perché vuole proprio le strade minori.
- [V] Le varianti `MLimitedResources` con tempo, lunghezza ed energia (kg·m²/s²) sono la ricerca di
  raggiungibilità (isocrone e autonomia EV), con `CReachabilityTimeDetector`, `…LengthDetector` e
  `…EnergyDetector`.
- [?] Se la ricerca del percorso sia bidirezionale: c'è la stringa `Bidirectional` e una variante
  `TBackwardSearch`, ma non si vede dove sono usate.

### 1.3 Livelli e "box" attorno a partenza e arrivo

- [V] Stringhe del livellamento: `Leveling boxes around origin`, `Leveling boxes around destination`,
  `FRC filtering boxes around origin/destination`, `InitMaxFRC(): non-standard level configuration
  found - FRC filter disabled`, `BackwardLevels:`, `Leveling,LevelCountWeightedOnlyFirst,Uncapped,`.
- [V] Parametri di taratura (dal file `Routing/Engine/Interface/src/Tuning.cpp`; elenco completo nella
  cartella note, `parametri-taratura.txt`): `originBoxFactor`, `targetBoxFactor`,
  `originBoxFactorOnRetry`, `targetBoxFactorOnRetry`, `defaultExtendBoxFactorForFRCFilter`,
  `extendBoxFactorForFRCFilterWithAvoidFerries`, `coarsestMapLevelForShortest`,
  `coarsestMapLevelForCompromise`, `coarsestMapLevelForAvoidMotorways`, `finestLevelDefault`,
  `finestLevelForArcsPenalizedByAvoid{TollRoads,Ferries,Tunnels,Highways,Borders,Unpaved,…}`,
  `singleLevelQuality`.
- [D] Funziona così: attorno a partenza e arrivo c'è un riquadro dove la ricerca usa il livello più
  fine. Fuori dal riquadro può salire di livello, cioè usare solo le strade di classe funzionale
  (FRC) alta. Se non trova un percorso riprova con riquadri più grandi (`…OnRetry`). Per "più breve"
  e "compromesso" il livello più grossolano è limitato, perché sulla sola rete principale il più
  breve sarebbe sbagliato. Con un "evita" attivo, gli archi penalizzati vanno considerati fino a un
  livello fine dedicato.
- [V] Tabelle NDS dei livelli: `levelMetadataTable(levelNumber, buildingBlockId, coordShift, min/maxScaleDenominator)`
  e `tileContentIndexTable(updateRegionId, buildingBlockId, levelNumber, southWestTileId, numRows, numColumns, tileContentIndex)`.
  La seconda dice quali tile esistono a ogni livello per ogni regione.

### 1.4 Costo, svolte, evitamenti, alternative

- [V] Funzioni di costo delle svolte distinte per mezzo: `CStandardCarTurnCostFunction`,
  `CStandardCarTurnCostFunctionIQR2`, `CTruckTurnCostFunction`,
  `CTruckShortestRouteTurnCostFunction`, `CBicycleTurnCostFunction`, `CPedestrianTurnCostFunction`.
- [V] Parametri di penalità: `turnPenaltyForMinorCrossing`, `turnPenaltyForMinorMajorCrossing`,
  `reEnterFreewayPenalty`, `reEnterFreewayUTurnPenalty`, `maximalFollowFreewayMultiplier`,
  `roadClassPenalties{Major,Medium,Minor,Urban}`, `lengthPenaltyForMinorRoads`,
  `lengthPenaltyForMinorRoadsOverland`, `straightnessPenaltyFactorMin/Max`, `idealCurveRadius`,
  `ferryPenalties`, `maxFRCForImportantFerry`, `ecoPenaltyMultiplicator`, `factorForShortest`,
  `maxDistForShortest`, `fixedSpeedVehicleFactor`, `uphillSlopeModifier`, `downhillSlopeModifier`.
- [V] **Evitare non è vietare.** L'API distingue `addAvoid` da `addForbid` (`CostModelBuilder`). Il
  motore ha `penaltyFactorForAvoid` e `maximalLengthOfDiversionPathForAvoid{TollRoads,Ferries,Tunnels,…}`.
  [D] Una strada "da evitare" costa di più (fattore) e la deviazione per evitarla ha un tetto di
  lunghezza; solo "vieta" la esclude. Così "evita pedaggi" non dà mai "nessun percorso".
- [V] Alternative con due algoritmi: `AlternativeRoutes.AlgorithmA.{penalizePercentage, minimal/maximalPenalizePercentage, maxSharedLengthPercentage, lengthDependencyImpact}`
  e `AlgorithmB.maxSharedLengthPercentage`, più `altSearchPenalty` e `maxSharedArcsPercentage`. [D]
  L'algoritmo A è il metodo delle penalità: si rende più cara la strada già trovata e si ricalcola.

### 1.5 Restrizioni del veicolo e camion

- [V] Il modello del veicolo (Java, `com.tomtom.navkit.navigation.Vehicle`, dalla bozza precedente):
  tipo, categoria UE (N1/N2/N3/N3+O4), motore, altezza/larghezza/lunghezza in cm, peso e peso per
  asse in kg, velocità massima, uso commerciale, dotazioni (`TRAILER`, `SEMI_TRAILER`, `CARAVAN`,
  `SNOW_CHAINS`), classi di merci pericolose, codice galleria ADR, classe Euro.
- [V] Nel motore nativo le condizioni di una restrizione (descrittori protobuf di VehicleHorizon):
  `VehicleCondition` con `VehicleWeight.weightKg`, `VehicleWeightPerAxle.weightPerAxleKg`,
  `VehicleLength.lengthCm`, `VehicleWidth.widthCm`, `VehicleHeight.heightCm`, `VehicleType`;
  `TimeCondition`; `WeatherConditionType` = `kAppliesAlways`, `kAppliesDuringRain`,
  `kAppliesDuringFog`, `kAppliesDuringSnow`; `DangerousGoodsRestriction` con `adrTunnelCategory` e
  `hazardousLoad`.
- [V] Merci pericolose nel motore: `EHAZMATClass1…9`, `EHAZMATExplosive`, `EHAZMATGeneral`,
  `EHAZMATWater`, `kAdrTunnelCategoryA…E`, `kADRCodeB…E`, e il controllo `ADR tunnel restriction code
  is only allowed in conjunction with hazmat load`.
- [V] Tipi di restrizione in `RoadCategory`: tra gli altri `RESTRICTED_IN_FUZZY_TIME` e
  `NON_COMMERCIAL`. [D] "Fuzzy time" sono i divieti con orario vago ("in stagione", "durante le
  manifestazioni"): il motore li tratta come categoria evitabile, non come divieto certo.
- [V] Le policy `CVehicleRestrictionsPolicy` e `CTimeDomainPolicy` stanno in ogni catena di ricerca,
  e il costo dipende dall'orario di partenza (`setDepartureTime`, `departureTimeZone`,
  `regionTimeZoneTable`, `timeZoneOlsonIdTable`). [D] I divieti a orario (per esempio "camion vietati
  22–6") si valutano all'ora in cui il mezzo arriva sull'arco.
- [V] Se la mappa non ha attributi camion: schermata `NoTruckAttributesAvailableScreen`, stringa
  `navui_vehicle_no_truck_attributes_available`; il calcolo ricade sul profilo auto con un avviso.
- [V] Ripianificazione quando cambia il profilo (`PlanningReason.VEHICLE_PROFILE_CHANGED`).

### 1.6 Velocità, traffico, pedaggi

- [V] Profili di velocità storici: `speedProfileTable(profileId, profile)`, `LoadSpeedProfiles(UR=%u)`,
  `CRoutingDataRing::CSpeedProfileCache`, campi di arco `LineRec_FreeFlowSpeed` e
  `LineRec_SpeedProfileIndex`, impostazione `UseIQRoutes = true`, modalità `HDTraffic, HDFlow, AdaptedIQR`.
  [D] Ogni arco ha una velocità a flusso libero e un indice verso una tabella di profili settimanali
  condivisi ("IQ Routes"): l'arco non porta 168 valori, solo il numero del profilo.
- [V] Traffico separato dal grafo: `trafficcache.sqlite` (536 eventi TEC con posizioni OpenLR,
  `DEVICE_ANALYSIS.md`), `CTrafficPolicy`, `trafficRoutingTileTable(locationId, tileId, attrMapList)`
  per legare le località TMC agli archi.
- [V] Pedaggi con costo: `tollCostTable(gateIdFrom, gateIdTo, tollGatewayId, paymentConditions)`,
  `globalTollGatewayTable`, `CTollAmountAgent`, `vignette_countries`, `kTollFeeVignette`. [D] Il costo
  si calcola da casello a casello, anche tra regioni diverse.

### 1.7 Onboard, ibrido, online

- [V] `ComputationPolicy` = `ONBOARD_ONLY`, `HYBRID`, `ONLINE_ONLY`; nel motore
  `CHybridPlanningController` con callback onboard e online, `COnboardPlanningController`,
  `COnlineOnlyPlanningController`.
- [V] Sulla tablet `com.tomtom.navui.setting.feature.plan.use.online.routing = 3`. Nell'enum
  dell'app (`Wf.I.y`: `ONBOARD_ONLY`=0, `INITIAL_HYBRID`=1, `ONLINE_ONLY`=2, `HYBRID`=3) e nella sua
  conversione (`SettingsManagerImpl.J9`) il valore 3 diventa `HYBRID`. [D] L'app è quindi in modalità
  ibrida: calcola a bordo e in rete insieme e tiene il risultato migliore o quello che arriva.
- [?] Prova a rete spenta: non eseguita, per non disturbare la tablet condivisa. Che funzioni offline
  resta dedotto dai `ROUTE.NDS` installati e dalla modalità `ONBOARD_ONLY`.

---

## 2. Le mappe offline

### 2.1 Organizzazione sul dispositivo [V]

```
files/NdsMaps/WORLD/
├── DATA.MD5                       2 548 righe: md5 di ogni file di ogni regione del catalogo
└── DATA/
    ├── ROOT.NDS                   12 KB
    └── 00000028/                  fornitore 0x28 (TomTom)
        ├── 00022548/17049391/     prodotto "Europa", versione base
        │   ├── PRODUCT.NDS (2,4 MB) + PRODUCT.shm   ← .shm = memoria condivisa SQLite
        │   ├── UR_154/  ROUTE 120 MB · BMD 53 · POI 42 · FTS 36 · NAME 14 · LHPLUS 12 · TMC 6 · SHARED 0,7
        │   ├── UR_171/  ROUTE 209 MB · BMD 96 · POI 71 · FTS 51 · NAME 21 · LHPLUS 16 · TMC 9 · SHARED 1,1
        │   ├── UR_900/  ROUTE 1,8 MB · BMD 53 · LHPLUS 3,7 · POI 3,1 · NAME 0,1
        │   ├── UR_901/  ROUTE 2,0 MB · BMD 9,4
        │   └── UR_902/  ROUTE 49 KB · BMD 35 · LHPLUS 3,9 · POI 3,1
        ├── 00022549/…  UR_116, UR_122: LM.NDS (modelli 3D dei monumenti)
        ├── 00022561/…  UR_7: SAT_IMG.NDS (immagini satellitari)
        ├── 00022605/…  UR_1…14: BMD + NAME (panoramica del mondo, piccola)
        └── altri prodotti: solo PRODUCT.NDS (catalogo senza dati)
```

- [V] Il catalogo `DATA.MD5` elenca 14 prodotti (continenti e strati). Ogni prodotto stradale ha un
  insieme di UR numerate per area (Europa 100–194, Nord America 300–452, …). Ogni prodotto stradale
  ha anche **UR 9xx**: Europa 900/901/902, Nord America 903/912/914, altri 904/905/906–908/909/910/916–921.
- [D] Le UR 9xx sono regioni di collegamento con i livelli alti (la rete principale) di tutto il
  continente. Lo fanno pensare le dimensioni (2 MB di routing per l'intera Europa contro 120–209 MB di
  una regione), la presenza in ogni continente e il fatto che si installano con il Paese. Lo standard
  NDS permette questa separazione: i livelli possono stare in UR diverse, collegate da
  `globalGatewayTable(gatewayId, updateRegionId, tileId)`.
- [D] UR_154 e UR_171 sono molto probabilmente l'Italia (impostazioni `last.country.code = ITA`;
  report del 10 novembre). Il nome delle UR sta in `PRODUCT.NDS`, che non ho decompresso.

### 2.2 Formato dei file [V]

Primi byte (letti con `run-as … head -c 112`):

| File | Magia | Dimensione pagina (byte 16–17) |
|---|---|---|
| `ROOT.NDS` | `ZV-lz4hc` | 0x1000 = 4 KB |
| `PRODUCT.NDS` | `ZV-lz4hc` | 0x2000 = 8 KB |
| `UR_154/ROUTE.NDS` | `ZV-zstd\0` + `4` | 0x2000 = 8 KB |
| `UR_154/BMD.NDS` | `ZV-zstd\0` + `4` | 0x8000 = 32 KB |

- [V] Dopo la magia i campi coincidono con l'intestazione di un database SQLite (dimensione pagina,
  versioni di scrittura e lettura, `0x40 0x20 0x20` per le frazioni di payload), e accanto a
  `PRODUCT.NDS` c'è un `PRODUCT.shm`. È il formato ZIPVFS di SQLite: un database normale le cui pagine
  sono salvate compresse una per una. [D] Il `4` dopo `zstd` è probabilmente il livello di
  compressione.
- [D] Conseguenza pratica: il motore apre il file come un database SQLite qualsiasi e decomprime solo
  le pagine che legge. I dati restano compressi anche sul disco del telefono, non solo nel download.
  La scelta della pagina ha un senso: 8 KB per il routing (letture sparse) e 32 KB per la mappa
  disegnata (letture a blocchi). Anche `zstd` ha senso per i dati grandi: rende più di LZ4 e
  decomprime comunque in fretta.
- [V] `urBuildingBlockVersionTable` ha una colonna `encryptionKeyId` e c'è `encryptionKeyTable(keyIndex, key)`.
  [?] Se i blocchi installati siano cifrati non l'ho verificato: verificarlo vorrebbe dire aprire le
  pagine, cosa che esce dai limiti di questa analisi.

### 2.3 Cosa c'è dentro il building block di routing [V]

Nomi di tabelle e di strutture NDS 2.4.6 (`DataAccess::NDS::ds_2_4_6`, unpacker DataScript
`DSUnpack`) usati dal motore:

| Tabella / struttura | Contenuto (secondo lo standard NDS pubblico) |
|---|---|
| `routingTileTable(id, ndsData)` | la tile di routing: archi (`LinkList`), incroci (`SimpleIntersectionList`), attributi fissi (`FixedRoadAttributeSetList`) |
| `routingAuxTileTable(id, ndsData, nameAttributeLayer, truckAttributeLayer, adasAttributeLayer)` | **livelli opzionali** della stessa tile: nomi, camion, ADAS (pendenze, curvature) |
| `attributesRoutingTileTable(id, attributesData)` + `attributesMetadataTable(routingAttributeTypeAvailability)` | altri livelli di attributi e quali tipi di attributo esistono in questa regione |
| `RouteUpLinkList` / `RouteDownLinkList` | collegamenti tra lo stesso incrocio a livelli diversi |
| `Link2TileList`, `ExternalTileIdList` | archi che escono dalla tile |
| `speedProfileTable(profileId, profile)` | profili di velocità condivisi |
| `trafficRoutingTileTable`, `TMC.NDS` | legame località traffico ↔ archi |
| `tollCostTable`, `globalTollGatewayTable` | pedaggi da casello a casello |
| `globalGatewayTable` | collegamenti fra UR (confini delle regioni) |
| `levelMetadataTable`, `tileContentIndexTable` | livelli e tile presenti |
| `UsedVehicleTypeInformation`, `SpeedClassDistributionList` | metadati: tipi di veicolo con restrizioni, distribuzione delle classi di velocità |
| `updateRegionViewSetTable(disputantId)` | viste geopolitiche (confini contesi) |

[D] Il punto da copiare non è il formato ma il principio: **un arco ha un nucleo minimo sempre
presente e livelli di attributi facoltativi per tile**. Chi non guida un camion non legge il livello
camion; una regione senza dati camion lo dichiara (`routingAttributeTypeAvailability`) e il client lo
sa prima di calcolare.

### 2.4 Aggiornamenti [V]

```
files/sharedContent/mapupdates/00000028/00022548/17049391/
├── PRODUCT.NUC                         2,9 MB
├── 00000144/00000000.NUS, 00000143.NUS, 00000145.NUS   metadati del passo di versione (~0,4 MB ciascuno)
├── 00000144/00000000/UR_00902/PR_00000.SXZ             340 byte
├── 00000156/00000000/UR_00154/PR_00032.SXZ             3 848 byte
├── 00000156/00000000/UR_00154/PR_00088.SXZ             3 420 byte
└── 00000000/00000156/UR_00121/BB_00024.NDS …           installazione completa di una regione (0 → 156)
```

- [V] 545 file per il prodotto Europa. I delta di versione per UR e building block pesano da 250 byte
  a pochi KB. Tutti i delta di UR_154 e UR_171 sommano 26 KB. I passi di versione arrivano almeno a 156.
- [D] TomTom aggiorna la mappa a piccoli passi frequenti, per regione e per tipo di dato, senza
  riscaricare il Paese. Funziona perché i dati sono indirizzati per tile e per identificatori stabili:
  una strada cambiata tocca poche pagine.

### 2.5 Dati necessari al routing camion offline (riassunto)

[V] dove c'è un nome di tabella o di attributo, [D] per il ruolo:

1. grafo a tile per livello, con collegamenti tra livelli e fra regioni;
2. attributi base dell'arco: classe funzionale, forma della strada, senso unico, velocità a flusso
   libero, indice del profilo di velocità, pedaggio, traghetto, galleria, ponte;
3. livello camion: limiti fisici (peso, asse, altezza, larghezza, lunghezza), divieti per tipo di
   veicolo, merci pericolose e categoria ADR delle gallerie, ciascuno con condizioni (orario, meteo,
   "fuzzy time");
4. divieti e restrizioni di manovra (`TURN_RESTRICTION`, `MANEUVER_RESTRICTION`);
5. fusi orari per regione (per valutare gli orari);
6. tabelle pedaggi e traffico, a parte;
7. metadati della regione: quali tipi di attributo ci sono (la copertura), versione per building block.

---

## 3. Confronto con il motore NavMaster

| Tema | TomTom (osservato) | NavMaster (codice attuale) |
|---|---|---|
| Algoritmo | A\* a etichette con euristica gonfiata, gerarchia dai livelli NDS, riquadri attorno a partenza/arrivo e nuovo tentativo con riquadri più larghi [V/D] | CH per le richieste statiche; core con restrizioni + core-ALT (16 landmark, insiemi per "evita pedaggi"); Dijkstra edge-based come ripiego; risultati esatti |
| Precalcolo | nessun shortcut: la gerarchia è nei dati [D] | CH + core + landmark per Paese (preparazione in CI, Italia ≈ 31 min) |
| Ottimalità | non garantita (A\* esagerato) [D] | garantita; verificata contro ORS e con riferimenti esaustivi |
| Dati | NDS 2.4.6, SQLite + ZIPVFS/zstd, compressi anche su disco | file a segmenti mappati in memoria, non compressi su disco; `nm.tar.gz` solo per il download |
| Spazio Italia | ≈ 330 MB routing, ≈ 760 MB con mappa, ricerca, POI e corsie [V/D] | `nm.tar.gz` 2,24 GB compresso + mappa 1,4 GB + ricerca e POI ≈ 1 GB |
| Attributi camion | livello separato per tile, copertura dichiarata | campi nei flag dell'arco per tutti gli archi; nessuna dichiarazione di copertura |
| Merci pericolose | classi UN, EU acque, codice galleria ADR, nel routing | un solo booleano `hazmat_forbidden` nel routing; categoria ADR della galleria solo nei dati di guida (`GuidanceStore`, byte 68) |
| Divieti a orario | `CTimeDomainPolicy` + orario di partenza + fusi orari | `*:conditional` letti e salvati come testo per la guida; ignorati dal routing |
| Meteo | condizioni pioggia/nebbia/neve sulle restrizioni | no |
| Svolte | funzioni dedicate (auto, camion, camion più breve) + penalità per incroci e rientri in autostrada | solo divieti di svolta e inversioni vietate (`TurnCostProvider`): costo 0 o infinito |
| Evitare | penalità con tetto di deviazione; "vietare" a parte | `avoid_features` = filtro rigido (come ORS) |
| Velocità | flusso libero + profili storici settimanali + traffico | tabelle velocità ORS per tipo di strada + `maxspeed`; niente orario, niente traffico |
| Pedaggi | costo da casello a casello, vignette | solo "strada a pedaggio sì/no" (`toll`, `toll:hgv`) |
| Fra Paesi | un grafo continentale per livelli; UR di collegamento leggere | un grafo per Paese, valichi precalcolati, combinazione esatta; per i Paesi non scaricati il server |
| Aggiornamenti | delta per regione e building block (byte–KB) | ricostruzione completa e nuovo download del Paese |
| Alternative | penalità (algoritmo A) + algoritmo B, limiti di sovrapposizione | plateau edge-based |
| Camper | stesso modello veicolo con dotazione `CARAVAN` e misure | profilo auto con misure (`DimensionsEdgeFilter`), senza divieti HGV |

Dove NavMaster è già avanti o alla pari: percorso esatto e verificato; Paesi combinati con valichi
scelti in modo esatto; camper con misure senza ereditare i divieti dei camion; coerenza tra
pianificazione e guida (stesso provider per i ricalcoli). L'A\* "esagerato" di TomTom è un
compromesso imposto da un grafo continentale senza precalcolo: **non va copiato**.

Dove TomTom è avanti, in ordine di impatto sul camionista: merci pericolose e divieti a orario nel
calcolo; "evita" che non blocca mai; svolte realistiche per un mezzo lungo; spazio e aggiornamenti;
viaggi lunghi senza scaricare ogni Paese.

---

## 4. Verifiche ancora utili

- Prova a rete spenta su TomTom (modalità aereo + "solo a bordo") con un percorso camion noto, per
  confermare il calcolo offline e vedere l'avviso sugli attributi camion. Va fatta quando la tablet è
  libera.
- Ripartizione delle dimensioni dei file di `nm` per l'Italia (grafo, geometria, CH, core, landmark,
  guida): serve a decidere il punto 6 della lista qui sotto. Non misurata qui (disco del Mac quasi
  pieno, grafi non presenti in locale).
- Mappatura PR/BB → tipo di blocco negli aggiornamenti (per esempio `BB_00000` ≈ 0,7 MB come
  `SHARED.NDS`): solo per curiosità, non serve a NavMaster.

---

## 5. Cosa emulare in NavMaster (ordinato per rapporto beneficio/costo)

Costo: B = giorni, M = una o due settimane, A = più settimane o un nuovo formato. Beneficio per il
camionista: ★ basso, ★★ medio, ★★★ alto.

| # | Cosa | Perché (da TomTom) | Come in NavMaster | Costo | Beneficio |
|---|---|---|---|---|---|
| 1 | **Merci pericolose complete nel routing** — ✅ fatto il 16/11 (§8) | classi + codice galleria ADR nel calcolo (§1.5) | il grafo ha già la categoria ADR della galleria (`Guidance.adrTunnel`) ma solo nella guida: portarla nei flag del routing (3 bit) e confrontarla col codice galleria del veicolo (B…E) in `HeavyVehicleEdgeFilter`; tenere `hazmat_forbidden` per `hazmat=no`; aggiungere `hazmat:water=no` per l'inquinante delle acque. Nell'app collegare `hazmatWater` e il codice galleria anche senza ADR "NONE" (punto 7 della bozza: `routeSpec()` oggi inoltra `hazmat` solo con codice ≠ NONE) | B | ★★★ |
| 2 | **Divieti a orario nel routing** | `CTimeDomainPolicy`, orario di partenza, fusi orari (§1.5) | i `*:conditional` sono già letti: tradurre il sottoinsieme comune di `opening_hours` (giorni, fasce, festivi) in identificativi di condizione per arco; valutarli all'orario stimato di arrivo sull'arco; mettere gli archi condizionati nel core, come già succede per le restrizioni. Parametro `departure` nella richiesta | M | ★★★ |
| 3 | **"Evita" come penalità, "vieta" a parte** — ✅ fatto il 16/11 (§8) | `addAvoid`/`addForbid`, `penaltyFactorForAvoid`, `maximalLengthOfDiversionPathFor…` (§1.4) | per pedaggi, traghetti e autostrade un moltiplicatore del peso (≥ 1) invece del filtro, con un'opzione "vieta" per il filtro attuale. I limiti dei landmark del set base restano ammissibili perché il peso può solo crescere. Toglie gli errori "nessun percorso" (isole, valichi solo a pedaggio). È una differenza voluta da ORS: va decisa con l'utente | B–M | ★★ |
| 4 | **Costi di svolta per il camion** | `CTruckTurnCostFunction`, `turnPenaltyForMinor(Major)Crossing`, `reEnterFreewayPenalty` (§1.4) | penalità in secondi per angolo di svolta (svolte strette, a sinistra attraverso una strada principale) e uscita-rientro in autostrada, calcolate dalla geometria all'import e salvate nel `TurnCostStorage`. Serve la ricerca edge-based: già usata col core. Da tarare con prove sul percorso | M | ★★ |
| 5 | **Copertura dei dati camion dichiarata** | `routingAttributeTypeAvailability`, avviso "nessun attributo camion" (§1.5, §2.3) | all'import contare per Paese le strade principali con `maxweight`/`maxheight`/`hgv` e salvarlo nel manifest; nell'app un avviso quando il percorso passa per zone a copertura bassa. Si possono marcare anche gli archi del percorso senza alcun dato camion | B | ★★ |
| 6 | **File del grafo a blocchi compressi** | SQLite + ZIPVFS/zstd, pagine da 8–32 KB, compressione anche su disco (§2.2) | comprimere a blocchi i file freddi (geometria, nomi, guida, punti) con zstd e una piccola cache di blocchi decompressi; lasciare non compressi i file caldi (nodi, archi, CH, core, landmark). Sul tablet ogni lettura mappata è già una chiamata nativa (memoria `android-mmap-reads`): misurare prima | M | ★★ (spazio) |
| 7 | **Dorsale europea leggera** | UR 900–902 da pochi MB per tutta l'Europa (§2.1) | un grafo "dorsale" con solo autostrade e strade principali dei 48 Paesi (con limiti camion e valichi), da usare per la parte centrale dei viaggi attraverso Paesi non scaricati invece del server. Si aggancia ai grafi dei Paesi ai valichi già calcolati. Il percorso nel Paese non scaricato sarebbe solo indicativo, da dichiarare all'utente | A | ★★★ (viaggi internazionali offline) |
| 8 | **Aggiornamenti incrementali** | delta per regione e tipo di dato da pochi KB (§2.4) | oggi una ricostruzione rinumera nodi e archi e cambia tutto. Un primo passo economico: ordine deterministico e download a blocchi con hash (stile zsync), così si scaricano solo i blocchi cambiati dei file freddi. I veri delta richiedono identificatori stabili, cioè un nuovo formato | A | ★★ (traffico dati) |
| 9 | **Restrizioni con condizioni meteo e "orario vago"** | `kAppliesDuringRain/Fog/Snow`, `RESTRICTED_IN_FUZZY_TIME` (§1.5) | leggere `maxspeed:wet`, `*:conditional` con `wet`/`snow` e i casi non interpretabili come "evitabile" invece che ignorati; utile soprattutto per i limiti in guida | B | ★ |
| 10 | **Alternative col metodo delle penalità** | `AlgorithmA` (§1.4) | ripiego quando il plateau non trova alternative valide sui viaggi lunghi | B–M | ★ |
| 11 | **Profili di velocità storici** | `speedProfileTable` + indice per arco (§1.6) | OSM non ha questi dati: andrebbero raccolti dalle tracce dei propri mezzi (come la "personal network" di TomTom). Da tenere come idea | A | ★★ (se ci fossero i dati) |
| 12 | **Pedaggi con costo** | `tollCostTable` da casello a casello (§1.6) | servono tabelle tariffarie per Paese, non presenti in OSM in modo completo | A | ★ |

Da **non** emulare: l'A\* con euristica gonfiata e la ricerca per riquadri (NavMaster ha già
precalcolo esatto e tempi da millisecondi); il formato NDS in sé (licenza a pagamento e strumenti
dell'associazione NDS, e il grafo NavMaster nasce da OSM).

---

## 7. I limiti per camion delle mappe TomTom e come applicarli in NavMaster

Tutti i tipi di limite che il motore e l'app TomTom conoscono, ricavati da tre fonti indipendenti:
- le enumerazioni Java `LegalRestriction.RestrictionType` e `PhysicalRestriction.RestrictionType`, `HazmatCategory`, `AdrTunnelCode`, `Equipment`, `VehicleCategory` e `RoadCategory`;
- i descrittori protobuf `VehicleHorizon` del motore nativo (`VehicleCondition`, `TimeCondition`, `WeatherCondition`, `DangerousGoods`);
- le stringhe dell'interfaccia (`navui_map_context_popup_*_restriction`, `navui_route_bar_dialog_*`, `navui_routerelaxednotification_*`).

| Limite TomTom | Prova [V] | Dati OSM | NavMaster prima | NavMaster ora / da fare |
|---|---|---|---|---|
| Altezza | `PhysicalRestriction.HEIGHT`, `VehicleHeight.heightCm`, «Height restriction» | `maxheight`, `maxheight:physical` | calcolo + avvisi | invariato |
| Larghezza | `WIDTH`, `VehicleWidth.widthCm` | `maxwidth`, `maxwidth:physical` (`width` facoltativo) | calcolo + avvisi | invariato |
| Lunghezza | `LENGTH`, `VehicleLength.lengthCm` | `maxlength` | calcolo + avvisi | invariato |
| Peso totale | `LegalRestriction.WEIGHT`, `VehicleWeight.weightKg` | `maxweight`, `maxgcweight` | calcolo + avvisi | da aggiungere: `maxweightrating`, `maxweight:hgv` |
| Peso per asse | `WEIGHT_PER_AXLE`, `VehicleWeightPerAxle` | `maxaxleload` | calcolo + avvisi | da aggiungere: `maxbogieweight` (asse tandem) |
| Numero di assi | `NUM_AXLES` | `maxaxles` (rarissimo) | no | utilità bassa: dati quasi assenti |
| Divieto camion / tipo di veicolo | `BIG_VEHICLE`, `VehicleType`, «No trucks allowed», categorie N1/N2/N3 | `hgv`, `goods`, `bus`, `access`, `motor_vehicle` | calcolo (`hgv_blocked`) | invariato |
| Divieto dipendente dal peso | `VehicleCondition` dentro un divieto | `hgv:conditional=no @ (weight>7.5)`, `maxweight:conditional` | solo guida (testo) | da fare insieme ai divieti a orario (voce 2 di §5) |
| Merci pericolose, classi UN | `HazmatCategory` 1…9, «Dangerous cargo restriction — UN %s» | `hazmat=no` | un booleano | ✅ `hazmat=no` vale per ogni carico (§8) |
| Galleria ADR B…E | `AdrTunnelCode`, `kAdrTunnelCategoryA…E`, «ADR tunnel restriction — Tunnel category %s» | `hazmat:adr_tunnel_category`, `hazmat:B..E=no` | vietata ogni galleria ADR | ✅ categoria contro il codice del carico (§8) |
| Merci inquinanti per le acque | `EU_HARM_WATER`, «Goods harmful to water (HW)» | `hazmat:water=no` | no | ✅ (§8) |
| Fasce orarie | `TimeCondition`, `CTimeDomainPolicy`, orario di partenza | `*:conditional @ (Mo-Fr 07:00-19:00)` | solo guida | da fare (voce 2 di §5) |
| Orario "vago" | `RESTRICTED_IN_FUZZY_TIME`, «restricted in fuzzy time» come categoria evitabile | condizionali non interpretabili | ignorati | da fare: trattarli come "evitabili" con la penalità |
| Meteo | `kAppliesDuringRain/Fog/Snow` | `maxspeed:wet`, `*:conditional @ wet/snow` | no | solo guida (limiti) |
| Rimorchio, semirimorchio | `Equipment.TRAILER/SEMI_TRAILER`, «Route is not suitable for your trailer» | `hgv_articulated=no`, `trailer=no` | il profilo dell'app ha `trailer` solo per la geometria delle svolte | da fare: leggere i due tag e inviare il rimorchio |
| Uso commerciale | `NON_COMMERCIAL`, `isCommercial` | `goods=no`, `hgv=delivery` | in parte (tipo `delivery`) | invariato |
| Zone a basse emissioni | `LOW_EMISSION_ZONE` evitabile, `EuroEmissionClass` | `boundary=low_emission_zone` (poligoni) | avvisi nell'app | da fare: poligoni come penalità secondo la classe Euro |
| Velocità massima del mezzo | `speedInMetersPerHour` | — | `maximum_speed` | invariato |
| Svolte e manovre vietate | `TURN_RESTRICTION`, `MANEUVER_RESTRICTION` | `restriction:hgv`, `type=restriction` | calcolo (costi di svolta) | invariato |
| Gallerie, confini, non asfaltate, traghetti | `AvoidCategory.TUNNEL/BORDER_CROSSING/UNPAVED/FERRY` | `tunnel`, `surface`, `route=ferry` | solo traghetti (e autostrade, pedaggi) | da fare: "evita gallerie" (utile ai trasporti ADR senza codice) |
| Pedaggi e vignette | `TOLL`, `ETC_TRANSPONDER_ONLY_TOLL_ROAD`, `kTollFeeVignette`, `tollCostTable` | `toll`, `toll:hgv`, `barrier=toll_booth`, `highway=toll_gantry` | solo il tag della way | ✅ anche caselli e portali (§8); costo: da fare |
| Percorso "rilassato" | «Route does not comply with your vehicle profile», «Your destination is on a road that is not suitable…» | — | errore "nessun percorso" o aggancio lontano | in parte: con la penalità gli evitamenti non bloccano più (§8). Per i limiti fisici e legali, da fare: un secondo calcolo senza il limite che non si può rispettare, con l'avviso |

[D] Il modello di TomTom distingue limiti **fisici** (il mezzo non passa: altezza, larghezza, peso
per asse) da limiti **legali** (vietato: peso, tipo, carico, orari). Ognuno ha condizioni che si
combinano (mezzo, orario, meteo, carico). Quando non esiste un percorso conforme, TomTom non dà
errore: calcola un percorso "rilassato" e lo dichiara. NavMaster ha già nell'app l'elenco dei limiti
lungo il percorso con quelli che bloccano (`LimitsIndex`). Il passo che manca è il percorso
rilassato dal motore.

---

## 8. Lavoro fatto il 16 novembre 2026 (motore e app, nessun commit)

### Merci pericolose complete (voce 1)
- Grafo: tre nuovi valori per arco, `adr_tunnel` (3 bit, B = 2 … E = 5, la lettera più restrittiva
  fra `hazmat:adr_tunnel_category` e `hazmat:X=no`), `hazmat_general` (`hazmat=no`), `hazmat_water`
  (`hazmat:water=no`). Sono registrati **dopo** i profili: nei grafi di prima tutti gli altri bit
  restano dove sono. `GraphEncoding.forExisting` apre questi grafi col loro schema, verificato sui
  grafi pubblicati di Slovenia, Croazia, Ungheria e Slovacchia.
- Filtro (`hazmatBlocked` in `EdgeFilters.kt`): con il codice del carico una galleria di categoria X
  è vietata ai codici ≤ X; `hazmat=no` è vietata a ogni carico; `hazmat:water=no` alle merci
  inquinanti per le acque. Senza codice resta la regola di ORS, più le gallerie che hanno solo
  `hazmat:adr_tunnel_category`. Sui grafi vecchi la categoria viene dai dati di guida.
- Core: regola 3, con le gallerie ADR e i due divieti nel core del camion.
- API: `restrictions.adr_tunnel_code` e `restrictions.hazmat_water`. App: `RouteSpec` e `NmRouting`
  li inviano. `LimitsIndex` non segnala più "divieto merci pericolose" sulle gallerie ammesse dal
  codice (le giudica per categoria) e mostra il divieto per le acque.
- Prove: `HazmatFilterTest` (categorie B…E, `hazmat=no`, acque, grafi vecchi, lettura del codice).

### Alternative sui viaggi lunghi e tra Paesi (indagine e correzione)
- Cause trovate:
  - l'app chiedeva alternative solo sotto i 60 km (`NmRouting`);
  - il motore le rifiutava oltre i 100 km (errore 2004);
  - i viaggi tra Paesi davano sempre un percorso solo (`CrossBorder` prendeva il primo pezzo e una
    sola coppia di valichi);
  - il plateau di ORS usa la ricerca piena, che sui viaggi lunghi esaurisce la memoria del tablet.
- Correzione: `LongAlternatives`, il metodo dei punti di passaggio.
  - Dopo il percorso migliore, si calcolano candidati per un punto a metà strada spostato di lato
    (±12 % e ±22 % della distanza in linea d'aria), agganciato alla strada principale più vicina,
    senza inversione al punto.
  - Ogni candidato usa core, landmark e valichi. Il punto di passaggio poi sparisce dalle istruzioni
    (si riusa `joinSteps` dei valichi).
  - Un candidato si tiene se è al più +30 % di tempo e se ha in comune col primo al più il 60 %
    (75 % oltre i 150 km).
  - Scatta oltre `maximum_distance_alternative_routes` (nell'app 60 km) e in ogni viaggio tra Paesi.
    Il plateau resta per i viaggi brevi.
- Prove su grafi veri (`LongAlternativesRealTest`, Mac):

  | Viaggio | Percorsi (minuti) | Calcolo |
  |---|---|---|
  | Lubiana → Bratislava (camion) | 377 / 412 / 432 | 5,6 s |
  | Capodistria → Košice | 646 / 713 / 828 | 7,6 s |
  | Zagabria → Budapest senza autostrade e pedaggi | 432 / 445 / 450 | 14 s |
  | Capodistria → Maribor (solo Slovenia) | nessuna alternativa entro +30 % | — |

  Sul tablet sarà più lento: oltre 35 s non si comincia un altro candidato.

### "No autostrade" e "no pedaggi" non rispettati (indagine e correzione)
- Cause trovate (il motore applicava gli evitamenti in ogni ramo, e l'app li passava):
  - un casello è un nodo e non bloccava nulla; in Italia 3 287 way autostradali non hanno il tag
    `toll`, e le rampe dei caselli quasi mai;
  - con il mezzo già in autostrada l'aggancio scartava l'autostrada e partiva da una strada vicina:
    ricalcoli continui;
  - le scelte venivano fissate all'avvio del viaggio, quindi cambiarle durante la guida non valeva;
  - le superstrade (`trunk` + `motorroad=yes`) non sono autostrade (resta così: le statali veloci
    sono ammesse, ed è scritto sotto l'interruttore).
- Correzioni:
  - "evita" come **penalità ×30** (`AvoidPenaltyWeighting`, `navmaster.avoid_penalty`, inviata
    dall'app). Le strade da evitare restano usabili solo se non c'è altra via, così il viaggio non
    fallisce né parte da una strada irraggiungibile. Si usano solo i landmark dell'insieme completo,
    gli unici ammissibili;
  - per non pagare in tempo, prima si calcola col divieto e la penalità scatta solo quando serve:
    il mezzo è su una strada da evitare (l'aggancio senza evitamenti cade su di essa, quello col
    divieto oltre 15 m) oppure col divieto non c'è percorso. Zagabria → Budapest senza autostrade e
    pedaggi, grafo caldo: 2,0 s col divieto, 6,0 s con la sola penalità, 2,0 s con la soluzione mista;
  - all'import, l'arco di una strada principale che passa da un casello diventa pedaggio; da un
    portale, pedaggio per i camion (`ImportOptions.tollNodes`; `--no-toll-nodes` nel confronto con
    ORS). Questo vale **dopo la ricostruzione dei grafi** (`grafo-nm.yml`);
  - nell'app il cambio di "senza pedaggi" o "senza autostrade" ricalcola il viaggio in corso o le
    schede aperte. La scheda dice "⚠ pedaggio inevitabile" o "⚠ autostrada inevitabile" invece di
    "✓ senza pedaggi", e conta anche i caselli. Le didascalie delle impostazioni sono aggiornate.
- Prove: `AvoidPenaltyTest` su Heidelberg. Sugli stessi agganci la penalità non è mai peggio del
  divieto; da un punto su un'autostrada aggancia l'autostrada, mentre il divieto no.

### Non ancora fatto o non verificato
- Nessuna prova sul tablet: serve una build dell'app con il motore nuovo (il jar del motore in
  `app/libs` è stato rimesso com'era).
- I caselli come pedaggio e la regola 3 del core valgono solo coi grafi ricostruiti. Fino ad allora
  la penalità, le alternative e il codice galleria (dai dati di guida) funzionano anche coi grafi
  pubblicati.

---

## 6. Prove e riferimenti

**Evidence REA:** `ev_41f48666…` (pacchetto installato, `inspect_adb_package`), `ev_35c7526a…` e
`ev_8ee8a123…` (inventario APK, `inspect_artifact`). `list_adb_directory` non può leggere la cartella
privata dell'app (`Permission denied`): per la cartella mappe ho usato `adb shell run-as`, possibile
perché la build installata è `DEBUGGABLE`.

**Elenchi completi** (cartella note di questa sessione,
`~/Library/Application Support/Claude/side-session-notes/local_91110752-babf-4985-9ff1-f8ce335a3206/tomtom-rea/`):
`classi-routing-nds.txt` (525 classi `Routing::`, `DataAccess::NDS::`, `VehicleHorizon::`),
`parametri-taratura.txt` (157 nomi), `query-sql-nds.txt` (44 query sulle tabelle NDS),
`file-mappa-tablet.txt` (91 file con dimensione), `aggiornamenti-europa.txt` (545 file di
aggiornamento), `percorsi-sorgente.txt` (231 percorsi dei sorgenti NavKit).

**Comandi per ripetere** (cartella `~/Downloads/TomTom GO for Truck/extracted/lib/arm64-v8a/`):

```bash
strings -n 6 libNavKit.so | grep -E '^N[0-9]+[A-Za-z]' | sed 's/^/__ZTS/' | c++filt | sed 's/^typeinfo name for //' | sort -u
```

```bash
adb -s R8YY402LRNP shell "run-as com.tomtom.gplay.navapp find files/NdsMaps -type f -exec ls -l {} \;"
```

```bash
adb -s R8YY402LRNP exec-out run-as com.tomtom.gplay.navapp head -c 112 files/NdsMaps/WORLD/DATA/00000028/00022548/17049391/UR_154/ROUTE.NDS | xxd
```

**Materiale precedente:** `~/Downloads/TomTom GO for Truck/REVERSE_ENGINEERING_REPORT.md` (API Java,
modello veicolo, autovelox), `DEVICE_ANALYSIS.md` (dati sul dispositivo; la frase "NDS compressi con
LZ4-HC" vale solo per ROOT e PRODUCT), `jadx_out/sources/com/tomtom/navkit/navigation/{CostModelBuilder,Vehicle,ComputationPolicy}.java`.

**NavMaster:** `engine-core/src/main/kotlin/it/navmaster/engine/weighting/EdgeFilters.kt`
(`HeavyVehicleEdgeFilter`, `DimensionsEdgeFilter`, `AvoidFeaturesEdgeFilter`),
`weighting/TurnCostProvider.kt`, `profile/Guidance.kt` (ADR galleria e `*:conditional`, solo guida),
`storage/GuidanceStore.kt`, `ch/CoreRouting.kt`, `ch/CoreLandmarks.kt`, `api/CrossBorder.kt`;
nell'app `routing/NmRouting.kt`, `routing/RouteSpec.kt`, `vehicle/VehicleProfile.kt`. Dimensioni dei
pacchetti da `catalog.json` della release `dati-catalogo` (8 novembre 2026).
