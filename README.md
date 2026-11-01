# navmaster-engine

Motore di routing per camion e auto scritto da zero in Kotlin, che riproduce la logica di calcolo di
[openrouteservice](https://github.com/GIScience/openrouteservice) (e del suo fork di GraphHopper) ed espone
la stessa API HTTP, così il client openrouteservice già presente in NavMaster/OsmAnd funziona senza modifiche.

- Profili: `driving-car`, `driving-hgv` (limiti di altezza, larghezza, lunghezza, peso, carico per asse,
  merci pericolose, tipi di mezzo pesante, divieti di svolta, pedaggi, `avoid_features`, `avoid_polygons`,
  `maximum_speed`, `extra_info`, istruzioni in italiano e inglese).
- Accelerazione: Contraction Hierarchies per le richieste senza restrizioni, "Core" per quelle con restrizioni.
- Documentazione: [docs/ARCHITETTURA.md](docs/ARCHITETTURA.md); analisi dei sorgenti originali in `docs/analisi/`.

## Requisiti

Java 17 (JDK). Gradle è incluso (`./gradlew`). Per i test serve `data/heidelberg.test.pbf` (nel repository).

## Costruire il grafo di una regione

```bash
curl -L -o data/lombardia.osm.pbf https://download.openstreetmap.fr/extracts/europe/italy/lombardia-latest.osm.pbf
./gradlew :engine-cli:installDist -q
JAVA_OPTS="-Xmx5g" engine-cli/build/install/engine-cli/bin/engine-cli import data/lombardia.osm.pbf graphs/lombardia --no-width-from-carriageway
```

`--no-width-from-carriageway` evita di usare il tag OSM `width` (larghezza della carreggiata) come limite di
larghezza per i camion, cosa che openrouteservice fa e che esclude molte strade normali
(vedi [docs/analisi/verifiche-stranezze.md](docs/analisi/verifiche-stranezze.md)). Senza l'opzione il
comportamento è identico a openrouteservice.

Altri comandi: `engine-cli info graphs/lombardia`, `engine-cli route graphs/lombardia 45.4642,9.19 45.6983,9.6773 --height=4 --weight=40`.

## Avviare il server

```bash
./gradlew :engine-server:installDist -q
JAVA_OPTS="-Xmx3g" engine-server/build/install/engine-server/bin/engine-server --graph=graphs/lombardia --port=8082 --api-key=scegli-una-chiave
```

Senza `--api-key` il server accetta tutte le richieste. Endpoint:

| Metodo | Percorso | Note |
|---|---|---|
| POST | `/ors/v2/directions/{profile}` | risposta JSON (`/json`) o `/geojson` |
| GET | `/ors/v2/directions/{profile}?start=lon,lat&end=lon,lat` | GeoJSON |
| GET | `/ors/v2/health`, `/ors/v2/status` | stato |

Con Docker: `docker compose up` (costruisce il grafo al primo avvio da `data/lombardia.osm.pbf`).

## Prova con NavMaster (tablet sulla stessa Wi-Fi)

1. Trova l'IP del Mac (`ipconfig getifaddr en0`) e avvia il server come sopra.
2. Nell'app: *Impostazioni → Navigazione → Tipo di navigazione → Motore online → openrouteservice*,
   URL `http://IP-DEL-MAC:8082/ors/v2/directions/`, veicolo `driving-hgv`, chiave API se impostata.
3. Con la patch `docs/navmaster-app/navmaster_ors_patch.py` applicata alla build, l'app invia anche
   altezza/peso/larghezza/lunghezza del profilo camion; senza patch calcola come camion generico.

## Test

```bash
./gradlew test
```
