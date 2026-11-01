#!/bin/bash
# NavMaster - server openrouteservice per camion (profilo driving-hgv)
# Per Ubuntu 22.04/24.04, ARM64 (Oracle Ampere A1) o x86_64.
#
# Uso, sul server:
#   sudo bash setup-server.sh
# Opzioni (variabili d'ambiente):
#   REGION_URL  file OSM da scaricare     (default: Italia da Geofabrik)
#   DOMAIN      nome per HTTPS             (default: <ip-pubblico>.sslip.io, gratuito)
#
# Risultato: https://DOMAIN/ors/v2/directions/  protetto da chiave API (header Authorization)
set -euo pipefail

REGION_URL="${REGION_URL:-https://download.geofabrik.de/europe/italy-latest.osm.pbf}"
ORS_IMAGE="openrouteservice/openrouteservice:v10.1.0"
CADDY_IMAGE="caddy:2"
BASE=/opt/navmaster-ors

[ "$(id -u)" -eq 0 ] || { echo "Esegui con sudo: sudo bash $0"; exit 1; }

echo "== 1/6 Pacchetti (Docker, Compose)"
apt-get update -q
DEBIAN_FRONTEND=noninteractive apt-get install -y -q docker.io docker-compose-v2 curl openssl
systemctl enable --now docker

echo "== 2/6 Firewall del sistema (porte 80 e 443)"
# Le immagini Ubuntu di Oracle Cloud bloccano tutto tranne SSH con iptables.
iptables -C INPUT -p tcp -m multiport --dports 80,443 -j ACCEPT 2>/dev/null \
  || iptables -I INPUT 1 -p tcp -m multiport --dports 80,443 -j ACCEPT
if command -v netfilter-persistent >/dev/null; then netfilter-persistent save; fi

echo "== 3/6 Cartelle, dominio e chiave API"
mkdir -p "$BASE/ors-docker/config" "$BASE/ors-docker/files"
cd "$BASE"
if [ -z "${DOMAIN:-}" ]; then
  PUBLIC_IP="$(curl -fsS https://api.ipify.org)"
  DOMAIN="$(echo "$PUBLIC_IP" | tr . -).sslip.io"
fi
if [ -f .env ] && grep -q '^API_KEY=' .env; then
  API_KEY="$(grep '^API_KEY=' .env | cut -d= -f2)"
else
  API_KEY="$(openssl rand -hex 24)"
fi
# Memoria Java: RAM totale meno 3 GB per sistema e Caddy (minimo 2 GB)
TOTAL_GB=$(( $(awk '/MemTotal/ {print $2}' /proc/meminfo) / 1024 / 1024 ))
XMX_GB=$(( TOTAL_GB > 5 ? TOTAL_GB - 3 : 2 ))
cat > .env <<EOF
DOMAIN=$DOMAIN
API_KEY=$API_KEY
ORS_XMX=${XMX_GB}g
EOF
chmod 600 .env

echo "== 4/6 Mappa OSM: $REGION_URL"
if [ ! -s ors-docker/files/region.osm.pbf ]; then
  curl -fL --retry 3 -o ors-docker/files/region.osm.pbf.part "$REGION_URL"
  mv ors-docker/files/region.osm.pbf.part ors-docker/files/region.osm.pbf
fi

echo "== 5/6 Configurazione openrouteservice e HTTPS"
cat > ors-docker/config/ors-config.yml <<'EOF'
ors:
  engine:
    profile_default:
      build:
        source_file: /home/ors/files/region.osm.pbf
      service:
        # i valori predefiniti di ORS sono 100 km: troppo poco per un camion
        maximum_distance: 3000000
        maximum_distance_dynamic_weights: 3000000
        maximum_distance_avoid_areas: 3000000
        maximum_distance_alternative_routes: 3000000
        maximum_waypoints: 50
    profiles:
      driving-hgv:
        enabled: true
EOF

cat > Caddyfile <<'EOF'
{$DOMAIN} {
	encode gzip
	# stato del server, senza chiave
	handle /ors/v2/health {
		reverse_proxy ors:8082
	}
	# tutto il resto solo con la chiave (NavMaster la manda nell'header Authorization)
	@autorizzato header Authorization "{$API_KEY}"
	handle @autorizzato {
		reverse_proxy ors:8082
	}
	handle {
		respond "Unauthorized" 401
	}
}
EOF

cat > docker-compose.yml <<EOF
services:
  ors:
    image: $ORS_IMAGE
    restart: unless-stopped
    volumes:
      - ./ors-docker:/home/ors
    environment:
      REBUILD_GRAPHS: "False"
      CONTAINER_LOG_LEVEL: INFO
      XMS: 2g
      XMX: \${ORS_XMX}
  caddy:
    image: $CADDY_IMAGE
    restart: unless-stopped
    depends_on: [ors]
    ports:
      - "80:80"
      - "443:443"
    environment:
      DOMAIN: \${DOMAIN}
      API_KEY: \${API_KEY}
    volumes:
      - ./Caddyfile:/etc/caddy/Caddyfile:ro
      - caddy_data:/data
      - caddy_config:/config
volumes:
  caddy_data:
  caddy_config:
EOF

# Aggiornamento mappa (da lanciare a mano, per esempio una volta al mese).
# Il server non risponde mentre ricostruisce il grafo.
cat > aggiorna-mappa.sh <<EOF
#!/bin/bash
set -euo pipefail
cd "$BASE"
curl -fL --retry 3 -o ors-docker/files/region.osm.pbf.part "$REGION_URL"
mv ors-docker/files/region.osm.pbf.part ors-docker/files/region.osm.pbf
docker compose stop ors
rm -rf ors-docker/graphs/*
docker compose up -d ors
echo "Ricostruzione avviata: controlla con  docker compose logs -f ors"
EOF
chmod +x aggiorna-mappa.sh

echo "== 6/6 Avvio"
docker compose pull
docker compose up -d

cat <<EOF

=====================================================================
 Avviato. La prima costruzione del grafo richiede da una a qualche ora.
 Avanzamento:   cd $BASE && docker compose logs -f ors
 Pronto quando: curl https://$DOMAIN/ors/v2/health   ->  {"status":"ready"}

 Da inserire in NavMaster (motore online openrouteservice):
   URL:        https://$DOMAIN/ors/v2/directions/
   Chiave API: $API_KEY
   Veicolo:    HGV (driving-hgv)
 (la chiave e' salvata anche in $BASE/.env)
=====================================================================
EOF
