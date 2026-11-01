# Server NavMaster (engine-server)

Lo stesso motore del tablet, su un server, con un grafo di tutta l'Europa: serve all'app per i
viaggi che attraversano un confine o passano per un Paese non scaricato (Impostazioni › Percorso ›
Server NavMaster). Parla l'API di openrouteservice (`/ors/v2/directions/{profile}`) e aggiunge i
fatti sulle strade (`navmaster.roads`, `navmaster.points`) come il motore sul tablet.

## Requisiti

- Linux con Java 17 (es. Ubuntu 24.04, `apt install openjdk-17-jre-headless osmium-tool`).
- Memoria: il grafo Europa (strade per veicoli) richiede circa 20–24 GB di RAM per l'import con
  `--mmap` e 6–8 GB per servire (file memory-mapped). Oracle Cloud «Always Free» (Ampere A1,
  4 OCPU / 24 GB) basta per l'import se fatto con `--mmap`; per servire bastano 12 GB.
- Disco: PBF Europa ~30 GB, filtrato ~10 GB, grafo ~25 GB.

## Costruire il grafo Europa

```bash
curl -L -o europe.osm.pbf https://download.geofabrik.de/europe-latest.osm.pbf
osmium tags-filter europe.osm.pbf w/highway=motorway,motorway_link,trunk,trunk_link,primary,primary_link,secondary,secondary_link,tertiary,tertiary_link,unclassified,residential,living_street,service,road,track,busway w/route=ferry,shuttle_train r/type=restriction -o roads.osm.pbf
git clone https://github.com/francescobuzle-a11y/navmaster-engine.git && cd navmaster-engine
./gradlew :engine-cli:installDist :engine-server:installDist -q
JAVA_OPTS="-Xmx20g" engine-cli/build/install/engine-cli/bin/engine-cli import ../roads.osm.pbf /srv/navmaster/europe --mmap
```

## Avviare il server

```bash
JAVA_OPTS="-Xmx8g" engine-server/build/install/engine-server/bin/engine-server --graph=/srv/navmaster/europe --port=8082 --api-key=UNA-CHIAVE-LUNGA
```

Come servizio systemd (`/etc/systemd/system/navmaster.service`):

```ini
[Unit]
Description=NavMaster engine-server
After=network.target

[Service]
User=navmaster
Environment=JAVA_OPTS=-Xmx8g
ExecStart=/opt/navmaster-engine/engine-server/build/install/engine-server/bin/engine-server --graph=/srv/navmaster/europe --port=8082 --api-key=UNA-CHIAVE-LUNGA
Restart=always

[Install]
WantedBy=multi-user.target
```

Davanti al server mettere HTTPS (Caddy: `reverse_proxy localhost:8082`). Nell'app: Impostazioni ›
Percorso › Server NavMaster = `https://il-tuo-dominio`, e la chiave in «Chiavi dei servizi» alla
voce openrouteservice (il server la legge dall'intestazione `Authorization`).

## Aggiornare il grafo

Ripetere l'import in una cartella nuova (`/srv/navmaster/europe-YYYYMMDD`), poi cambiare `--graph`
e riavviare il servizio: il grafo vecchio si può cancellare.
