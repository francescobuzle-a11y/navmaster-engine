#!/usr/bin/env bash
# Il modello del terreno (DEM) per le pendenze del camion: scarica le celle di 1°×1° che coprono un riquadro
# e le scrive come celle .hgt SRTM (N46E011.hgt: 1201×1201 interi a 16 bit big-endian, il campione (0,0)
# sull'angolo nord-ovest, ultima riga e colonna in comune con le celle vicine) per
# `engine-cli import ... --dem=<cartella>` (modello navmaster soltanto). Nella cartella scrive anche
# SOURCE con la fonte usata (la CLI la mette nel file `profiles` del grafo, `dem_source`).
#
# uso: scarica-dem.sh <glo90|skadi> <minlon,minlat,maxlon,maxlat | file.osm.pbf> <cartella>
#
# glo90 (predefinito nelle CI): Copernicus GLO-90 (DSM 3″, TanDEM-X 2011-2015) dal bucket pubblico
#   https://copernicus-dem-90m.s3.amazonaws.com (le celle di terra in tileList.txt), convertito con GDAL:
#   gdalwarp sul riquadro allargato di mezzo campione (lon-1/2400 … lon+1+1/2400), 1201×1201, bilineare,
#   Int16 (quote al metro), poi gdal_translate in SRTMHGT; un avviso di allineamento di GDAL fa fallire il
#   passo, e il bordo comune di due celle vicine deve coincidere (check_edges). Il mosaico (gdalbuildvrt) è
#   alla risoluzione più fine delle celle e bilineare: sopra i 50° N le celle GLO-90 sono più strette in
#   longitudine (800 colonne fra 50° e 60°, 1200 sotto), e la risoluzione media di gdalbuildvrt le
#   ricampionerebbe al vicino più prossimo. Se il download, GDAL o la prova dei bordi falliscono si passa a skadi.
#   Attribuzione da mettere nei crediti dell'app: «produced using Copernicus WorldDEM-90 © DLR e.V.
#   2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European
#   Union and ESA; all rights reserved» (licenza COPDEM:
#   https://documentation.dataspace.copernicus.eu/APIs/SentinelHub/Data/DEM/resources/license/License-COPDEM-30.pdf;
#   da riscontrare con l'allegato GLO-90).
# Se anche la riserva fallisce: un avviso chiaro e uscita 3 (nessun DEM a metà). Chi chiama decide: i
# flussi grafo-nm e prova-velocita costruiscono allora il Paese senza DEM (solo le curve) con un ::warning::.
# skadi (riserva, senza GDAL): le celle .hgt.gz di Tilezen (https://elevation-tiles-prod.s3.amazonaws.com/skadi,
#   3601×3601, composito: EU-DEM in Europa, SRTM altrove, ultimo aggiornamento 2016); attribuzioni in
#   https://github.com/tilezen/joerd/blob/master/docs/attribution.md.
set -euo pipefail

SRC=${1:?uso: scarica-dem.sh <glo90|skadi> <bbox|file.osm.pbf> <cartella>}
AREA=${2:?riquadro o file .osm.pbf}
OUT=${3:?cartella di uscita}
JOBS=${DEM_JOBS:-$(nproc 2>/dev/null || echo 4)}

mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
WORK=$(mktemp -d "${TMPDIR:-/tmp}/dem.XXXXXX")
trap 'rm -rf "$WORK"' EXIT
export WORK OUT

# il riquadro: dato, o letto dal file OSM (tutti i nodi), allargato di ~500 m per la media delle quote ai capi
if [ -f "$AREA" ]; then
  BBOX=$(osmium fileinfo -e -g data.bbox "$AREA" | tr -d '() ')
else
  BBOX=$AREA
fi
echo "DEM $SRC: riquadro $BBOX"

# le celle del riquadro: "lat lon nome" (nome SRTM, es. N46E011)
python3 -I - "$BBOX" > "$WORK/cells.txt" <<'PY'
import math, sys
w, s, e, n = [float(x) for x in sys.argv[1].split(",")]
m = 0.005
for lat in range(math.floor(s - m), math.floor(n + m) + 1):
    for lon in range(math.floor(w - m), math.floor(e + m) + 1):
        name = ("N" if lat >= 0 else "S") + "%02d" % abs(lat) + ("E" if lon >= 0 else "W") + "%03d" % abs(lon)
        print(lat, lon, name)
PY
echo "celle nel riquadro: $(wc -l < "$WORK/cells.txt")"

glo90() {
  local base=https://copernicus-dem-90m.s3.amazonaws.com
  command -v gdalwarp >/dev/null && command -v gdal_translate >/dev/null && command -v gdalbuildvrt >/dev/null || { echo "GDAL assente"; return 1; }
  # dentro un `if` bash non applica set -e: ogni passo controlla da sé
  curl -fsSL --retry 5 --retry-delay 5 -o "$WORK/tileList.txt" "$base/tileList.txt" || return 1
  # le celle di terra del riquadro: "lat lon nome nomeCopernicus"
  python3 -I - "$WORK/cells.txt" "$WORK/tileList.txt" > "$WORK/land.txt" <<'PY'
import sys
have = {l.strip() for l in open(sys.argv[2]) if l.strip()}
for line in open(sys.argv[1]):
    lat, lon, name = line.split()
    lat, lon = int(lat), int(lon)
    cop = "Copernicus_DSM_COG_30_%s%02d_00_%s%03d_00_DEM" % ("N" if lat >= 0 else "S", abs(lat), "E" if lon >= 0 else "W", abs(lon))
    if cop in have:
        print(lat, lon, name, cop)
PY
  [ $? -eq 0 ] || return 1
  echo "celle di terra (GLO-90): $(wc -l < "$WORK/land.txt")"
  [ -s "$WORK/land.txt" ] || { echo "nessuna cella GLO-90 nel riquadro"; return 1; }
  mkdir -p "$WORK/tif" "$WORK/tmp"
  export COP=$base
  # shellcheck disable=SC2016
  awk '{print $4}' "$WORK/land.txt" | xargs -P 8 -n 1 bash -c 'curl -fsS --retry 5 --retry-delay 5 --retry-all-errors -o "$WORK/tif/$1.tif" "$COP/$1/$1.tif"' _ || return 1
  ls "$WORK"/tif/*.tif > "$WORK/tifs.txt" || return 1
  echo "scaricate $(wc -l < "$WORK/tifs.txt") celle ($(du -sh "$WORK/tif" | cut -f1))"
  # un mosaico solo: le celle vicine danno la riga e la colonna in comune (e il mezzo campione oltre il bordo);
  # fuori dalle celle (mare) nessun dato, escluso dalla bilineare di gdalwarp
  gdalbuildvrt -q -resolution highest -r bilinear -vrtnodata -32768 -input_file_list "$WORK/tifs.txt" "$WORK/dem.vrt" || return 1
  export WORK OUT
  # shellcheck disable=SC2016
  awk '{print $1, $2, $3}' "$WORK/land.txt" | xargs -P "$JOBS" -L1 bash -c '
    set -euo pipefail
    lat=$1; lon=$2; name=$3
    read -r W S E N < <(python3 -I -c "import sys; la, lo = int(sys.argv[1]), int(sys.argv[2]); h = 1 / 2400; print(\"%.10f %.10f %.10f %.10f\" % (lo - h, la - h, lo + 1 + h, la + 1 + h))" "$lat" "$lon")
    gdalwarp -q -overwrite -te "$W" "$S" "$E" "$N" -ts 1201 1201 -r bilinear -ot Int16 -dstnodata -32768 -of GTiff "$WORK/dem.vrt" "$WORK/tmp/$name.tif"
    if ! gdal_translate -of SRTMHGT "$WORK/tmp/$name.tif" "$OUT/$name.hgt" > /dev/null 2> "$WORK/tmp/$name.err"; then cat "$WORK/tmp/$name.err" >&2; exit 1; fi
    if grep -qi "align" "$WORK/tmp/$name.err"; then echo "$name: cella non allineata" >&2; cat "$WORK/tmp/$name.err" >&2; exit 1; fi
    rm -f "$WORK/tmp/$name.tif" "$WORK/tmp/$name.err" "$OUT/$name.hgt.aux.xml"
  ' _ || return 1
  rm -rf "$WORK/tif" "$WORK/tmp" "$WORK/dem.vrt"
  echo "copernicus-glo90" > "$OUT/SOURCE"
}

skadi() {
  local base=https://elevation-tiles-prod.s3.amazonaws.com/skadi
  mkdir -p "$WORK/gz"
  export WORK OUT
  # shellcheck disable=SC2016
  export SKADI=$base
  awk '{print $3}' "$WORK/cells.txt" | xargs -P 8 -n 1 bash -c '
    set -euo pipefail
    name=$1; dir=${name:0:3}
    code=$(curl -sS --retry 5 --retry-delay 5 --retry-all-errors -w "%{http_code}" -o "$WORK/gz/$name.hgt.gz" "$SKADI/$dir/$name.hgt.gz" || true)
    case "$code" in
      200) gunzip -c "$WORK/gz/$name.hgt.gz" > "$OUT/$name.hgt"; rm -f "$WORK/gz/$name.hgt.gz" ;;
      403|404) rm -f "$WORK/gz/$name.hgt.gz" ;; # mare: la cella non c è
      *) echo "skadi $name: risposta $code" >&2; exit 1 ;;
    esac
  ' _ || return 1
  [ -n "$(find "$OUT" -maxdepth 1 -name '*.hgt' -print -quit)" ] || { echo "skadi: nessuna cella nel riquadro"; return 1; }
  echo "skadi" > "$OUT/SOURCE"
}

# il bordo comune di due celle vicine (colonna 1200 della cella ovest = colonna 0 della cella est, e così
# l'ultima riga di una cella con la prima della cella a sud): la prova che le celle sono allineate
check_edges() {
  python3 -I - "$OUT" <<'PY'
import os, re, struct, sys
d = sys.argv[1]
tiles = {}
for f in os.listdir(d):
    m = re.fullmatch(r"([NS])(\d\d)([EW])(\d\d\d)\.hgt", f)
    if m:
        lat = int(m.group(2)) * (1 if m.group(1) == "N" else -1)
        lon = int(m.group(4)) * (1 if m.group(3) == "E" else -1)
        tiles[(lat, lon)] = os.path.join(d, f)
def side(p):
    return int(round((os.path.getsize(p) // 2) ** 0.5))
def samples(p, cells):
    """i campioni (riga, colonna) della cella, letti uno per uno"""
    n = side(p)
    with open(p, "rb") as fh:
        out = []
        for r, c in cells:
            fh.seek((r * n + c) * 2)
            out.append(struct.unpack(">h", fh.read(2))[0])
        return out
pairs = bad = total = 0
worst = 0
for (lat, lon), p in sorted(tiles.items()):
    for (dlat, dlon) in ((0, 1), (-1, 0)):
        q = tiles.get((lat + dlat, lon + dlon))
        if q is None:
            continue
        n = side(p)
        if side(q) != n:
            continue
        pairs += 1
        if dlon:
            a = samples(p, [(k, n - 1) for k in range(n)]); b = samples(q, [(k, 0) for k in range(n)])
        else:
            a = samples(p, [(n - 1, k) for k in range(n)]); b = samples(q, [(0, k) for k in range(n)])
        for x, y in zip(a, b):
            if x == -32768 or y == -32768:
                continue
            total += 1
            diff = abs(x - y)
            worst = max(worst, diff)
            if diff > 1:
                bad += 1
print("bordi comuni: %d coppie, %d campioni, %d diversi di più di 1 m (massimo %d m)" % (pairs, total, bad, worst))
if total and bad > total * 0.01:
    sys.exit("celle non allineate: i bordi comuni non coincidono")
PY
}

clean() { rm -f "$OUT"/*.hgt "$OUT"/*.aux.xml "$OUT/SOURCE"; }

# nessun DEM: la cartella vuota, un avviso chiaro, uscita 3 (il chiamante decide se costruire senza pendenze)
fail() {
  clean
  echo "ATTENZIONE: DEM non disponibile per $AREA ($*): nessuna cella .hgt in $OUT; il grafo avrebbe solo le curve, senza salite né discese" >&2
  exit 3
}

# dentro un `if` bash non applica set -e: ogni funzione controlla da sé i suoi passi
case "$SRC" in
  glo90)
    if glo90 && check_edges; then :; else
      echo "GLO-90 non riuscito (download, GDAL o bordi comuni): riserva skadi"
      clean
      if skadi && check_edges; then :; else fail "GLO-90 e skadi falliti"; fi
    fi
    ;;
  skadi) if skadi && check_edges; then :; else fail "skadi fallito"; fi ;;
  *) echo "fonte sconosciuta: $SRC (glo90 o skadi)" >&2; exit 2 ;;
esac

COUNT=$(find "$OUT" -maxdepth 1 -name '*.hgt' | wc -l)
[ "$COUNT" -gt 0 ] || fail "nessuna cella .hgt scritta"
echo "DEM pronto: $COUNT celle $(cat "$OUT/SOURCE") in $OUT ($(du -sh "$OUT" | cut -f1))"
