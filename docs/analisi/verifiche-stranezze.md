# Verifiche sulle stranezze trovate in openrouteservice

Esito delle verifiche sul codice sorgente di ORS 10.2 e del fork GraphHopper v4.16, con la decisione presa
per `navmaster-engine`.

| # | Stranezza | Esito della verifica | Decisione nel motore |
|---|---|---|---|
| 1 | Il tag OSM `width` (larghezza della carreggiata) è usato come limite di larghezza | **Confermata.** `OSMMaxWidthParser` legge `maxwidth`, `maxwidth:physical` e poi `width`; nessun commento né test lo giustifica, la documentazione ORS non lo cita. Nell'estratto di Heidelberg 156 archi hanno una larghezza < 2,55 m, quasi tutti da `width`. Effetto: un bilico da 2,55 m viene escluso da strade che hanno solo la carreggiata misurata. | Mantenuto per fedeltà come default, con l'opzione `--no-width-from-carriageway` (`ImportOptions.widthFromCarriageway=false`) che usa solo `maxwidth`/`maxwidth:physical`. A Heidelberg: 209 archi con limite usando `width`, 14 senza. **Per NavMaster si usa l'opzione.** |
| 2 | `motor_vehicle=no` + `hgv=yes` | **La mia prima lettura era sbagliata.** In ORS la lista delle chiavi di accesso del camion è `[hgv, motor_vehicle, vehicle, access, motorcar, …]` (il fork aggiunge prima le chiavi del modo HGV): `hgv=yes` prevale → la strada è percorribile. Per contro `access=no` + `motorcar=yes` resta chiusa al camion (ma non all'auto). La documentazione ORS riporta un ordine diverso (`hgv, motorcar, motor_vehicle, vehicle, access`), inesatto. | Corretto in `HgvProfile.restrictions`; test aggiornati. |
| 3 | Velocità autostrada camion: 80 nel codice Java, 85 nel JSON | **85 vince**: `SpeedLimitHandler` sovrascrive la mappa Java con `heavyvehicle.json`. Anche superfici: cement/compacted 60, fine_gravel 50. Le velocità documentate da ORS differiscono in alcuni punti dal JSON (trunk 60 nei documenti, 80 nel JSON; GB). | Tabelle generate dal JSON (`SpeedTables.kt`): coincidono con il comportamento reale di ORS. |
| 4 | Pedaggi per profilo | `toll=yes` vale per tutti; `toll:hgv`/`toll:N2`/`toll:N3` solo per il camion; `toll=no` esplicito; altrimenti assente. Le regole per paese (`country_rules`) sono disattivate di default. | Riprodotto in `AvoidFeaturesEdgeFilter`. |
| 5 | Le richieste "statiche" per il camion ignorano i divieti di svolta | **Confermata e rilevante.** Con `turn_costs: true` ORS crea i profili `heavyvehicle_*_with_turn_costs`, ma le CH sono sempre node-based: una richiesta senza restrizioni/avoid va sulla CH e **ignora i divieti di svolta e le inversioni**. Solo con restrizioni, `avoid_*`, `maximum_speed` o `force_turn_costs: true` si passa al Core (edge-based). I test API di ORS aggiungono `avoid_features: ["ferries"]` proprio per forzare il Core quando testano i divieti. | Riprodotto come default (`RouteOptions.allowCH`), documentato. **Per NavMaster si consiglia l'equivalente di `force_turn_costs: true`**: sempre Core/edge-based per il camion. |
| 6 | Inversioni a U | Vietate sui percorsi edge-based (`u_turn_costs = -1` di default, mai impostato dall'API directions); le matrici hanno lo stesso default. | Riprodotto (`TurnCostProvider`). |
| 7 | Istruzioni di guida | ORS non produce mai i tipi `UTURN (9)` e `EXIT_ROUNDABOUT (8)`: le inversioni diventano `CONTINUE`, dopo ogni rotatoria c'è sempre un passo `CONTINUE` con il nome della strada d'uscita. `exit_bearings` non è mai valorizzato. `roundabout_exits` non ha effetto. | Da riprodurre nella fase 1e, con la possibilità di emettere `UTURN` quando il client lo supporta. |
| 8 | Bug ORS: matrice | `MatrixService` lancia `StatusCodeException(6099)` usando 6099 come codice **HTTP**: il gestore degli errori fallisce. | Non riprodotto. |
| 9 | Bug ORS: intersezioni isocrone | Le intersezioni di secondo livello vengono calcolate e scartate. | Non riprodotto. |
| 10 | Bug ORS: `maximum_routes_flexible` | Mai applicato (si usa sempre `getMaximumRoutes(false)`). | Il nostro limite "flessibile" verrà applicato. |

## Dettaglio sul Core (trovato implementando)

Durante la preparazione del Core gli archi ristretti devono essere esclusi anche dalla **ricerca dei
testimoni**, non solo dagli shortcut: altrimenti un testimone che passa per un arco ristretto impedisce la
creazione dello shortcut che la richiesta filtrata avrebbe dovuto usare, e il Core restituisce percorsi
subottimali. ORS lo fa con `RestrictedEdgesWeighting` (peso infinito sugli archi del core in contrazione);
`navmaster-engine` esclude quegli archi dal grafo di preparazione (`CHPreparation(filter = !isCoreEdge)`).
Il test `CoreTest` ha scoperto il problema confrontando il Core con il Dijkstra filtrato su coppie casuali.

Analogamente, gli shortcut fra due nodi del core devono essere registrati su **entrambi** gli estremi,
perché nel core i nodi hanno tutti lo stesso livello e la ricerca di fase 2 li percorre in ogni direzione.
