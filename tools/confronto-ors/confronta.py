#!/usr/bin/env python3
"""Confronto richiesta per richiesta tra openrouteservice e navmaster-engine sullo stesso estratto.

Solo libreria standard. Genera coppie (e terne) di punti casuali nel riquadro di Heidelberg, manda lo
stesso corpo JSON a entrambi i server (`POST /ors/v2/directions/{profilo}/geojson`) per una matrice di
casi (auto, camion, restrizioni, avoid_features, maximum_speed, preferenze, bearings,
continue_straight, alternative) e confronta distanza e durata del riepilogo, numero di tratte e di
passi, somiglianza della geometria (frazione di punti entro 30 m dalla linea dell'altro) e i codici
di errore. Scrive un rapporto Markdown (stampato anche sul registro) e le coppie grezze in JSON.

Uso:
    python3 tools/confronto-ors/confronta.py --ors http://localhost:8080 --engine http://localhost:8082 \
        --n 30 --seed 42 --out confronto-out

Esce con codice diverso da zero solo per errori dello script o server irraggiungibili all'avvio,
mai per differenze nei risultati.
"""
import argparse
import json
import math
import os
import random
import statistics
import sys
import time
import urllib.error
import urllib.request

# riquadro approssimato dell'estratto heidelberg.test.pbf (lon, lat)
BBOX = (8.60, 49.35, 8.80, 49.50)
MIN_DIST_M = 1500.0      # distanza minima in linea d'aria tra punti consecutivi
NEAR_M = 30.0            # soglia di vicinanza per la somiglianza della geometria
TIMEOUT_S = 120

HGV_RESTRICTIONS = {"height": 4, "width": 2.55, "length": 16.5, "weight": 27, "axleload": 11.5}


def body_plain(coords, **extra):
    b = {"coordinates": coords}
    b.update(extra)
    return b


# (nome, profilo, numero di punti, costruttore del corpo). Il costruttore riceve le coordinate e un
# generatore casuale (per i bearings).
CASES = [
    ("car-base", "driving-car", 2, lambda c, r: body_plain(c)),
    ("hgv-base", "driving-hgv", 2, lambda c, r: body_plain(c)),
    ("hgv-restrizioni", "driving-hgv", 2, lambda c, r: body_plain(
        c, options={"vehicle_type": "hgv", "profile_params": {"restrictions": dict(HGV_RESTRICTIONS)}})),
    ("hgv-evita-pedaggi", "driving-hgv", 2, lambda c, r: body_plain(c, options={"avoid_features": ["tollways"]})),
    ("car-evita-autostrade", "driving-car", 2, lambda c, r: body_plain(c, options={"avoid_features": ["highways"]})),
    ("car-evita-traghetti", "driving-car", 2, lambda c, r: body_plain(c, options={"avoid_features": ["ferries"]})),
    ("car-velocita-max-80", "driving-car", 2, lambda c, r: body_plain(c, maximum_speed=80)),
    ("car-shortest", "driving-car", 2, lambda c, r: body_plain(c, preference="shortest")),
    ("hgv-shortest", "driving-hgv", 2, lambda c, r: body_plain(c, preference="shortest")),
    ("car-recommended", "driving-car", 2, lambda c, r: body_plain(c, preference="recommended")),
    ("hgv-recommended", "driving-hgv", 2, lambda c, r: body_plain(c, preference="recommended")),
    ("car-bearings", "driving-car", 2, lambda c, r: body_plain(c, bearings=[[r.randrange(0, 360), 30], []])),
    ("car-continua-dritto", "driving-car", 3, lambda c, r: body_plain(c, continue_straight=True)),
    ("car-alternative", "driving-car", 2, lambda c, r: body_plain(
        c, alternative_routes={"target_count": 3, "weight_factor": 1.4, "share_factor": 0.6})),
]


# ----------------------------------------------------------------------------------------- geometria

def haversine_m(lon1, lat1, lon2, lat2):
    rl1, rl2 = math.radians(lat1), math.radians(lat2)
    dlat = rl2 - rl1
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(rl1) * math.cos(rl2) * math.sin(dlon / 2) ** 2
    return 2 * 6371008.8 * math.asin(math.sqrt(a))


class LocalXY:
    """Proiezione equirettangolare locale in metri attorno a una latitudine di riferimento."""

    def __init__(self, lat0):
        self.kx = 111320.0 * math.cos(math.radians(lat0))
        self.ky = 110540.0

    def xy(self, p):
        return p[0] * self.kx, p[1] * self.ky


def seg_dist2(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    cx, cy = ax + t * dx, ay + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


class SegmentGrid:
    """Indice a griglia dei segmenti di una polilinea, per la distanza punto-linea in tempo quasi lineare."""

    def __init__(self, pts_xy, cell=100.0):
        self.cell = cell
        self.segs = []
        self.cells = {}
        for i in range(len(pts_xy) - 1):
            ax, ay = pts_xy[i]
            bx, by = pts_xy[i + 1]
            self.segs.append((ax, ay, bx, by))
            for cx in range(int(min(ax, bx) // cell), int(max(ax, bx) // cell) + 1):
                for cy in range(int(min(ay, by) // cell), int(max(ay, by) // cell) + 1):
                    self.cells.setdefault((cx, cy), []).append(i)
        if len(pts_xy) == 1:
            x, y = pts_xy[0]
            self.segs.append((x, y, x, y))
            self.cells.setdefault((int(x // cell), int(y // cell)), []).append(0)

    def within(self, px, py, radius):
        r2 = radius * radius
        cx0, cy0 = int(px // self.cell), int(py // self.cell)
        reach = int(radius // self.cell) + 1
        for cx in range(cx0 - reach, cx0 + reach + 1):
            for cy in range(cy0 - reach, cy0 + reach + 1):
                for i in self.cells.get((cx, cy), ()):
                    if seg_dist2(px, py, *self.segs[i]) <= r2:
                        return True
        return False


def overlap_fraction(pts_a, pts_b, proj, radius=NEAR_M):
    """Frazione dei punti di A entro `radius` metri dalla polilinea B (None se una è vuota)."""
    if not pts_a or not pts_b:
        return None
    grid = SegmentGrid([proj.xy(p) for p in pts_b])
    hits = sum(1 for p in pts_a if grid.within(*proj.xy(p), radius))
    return hits / len(pts_a)


# ---------------------------------------------------------------------------------------------- HTTP

def post(base, profile, body):
    url = "%s/ors/v2/directions/%s/geojson" % (base.rstrip("/"), profile)
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST", headers={
        "Content-Type": "application/json", "Accept": "application/geo+json, application/json"})
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT_S) as resp:
            text = resp.read().decode("utf-8", "replace")
            status = resp.status
    except urllib.error.HTTPError as e:
        text = e.read().decode("utf-8", "replace")
        status = e.code
    except Exception as e:  # trasporto, timeout
        return {"http": 0, "error_code": "transport", "message": str(e)[:300], "ms": int((time.time() - t0) * 1000), "raw": None}
    ms = int((time.time() - t0) * 1000)
    try:
        parsed = json.loads(text)
    except ValueError:
        parsed = None
    out = {"http": status, "ms": ms, "raw": parsed if parsed is not None else text[:2000]}
    if status != 200 or not isinstance(parsed, dict):
        err = (parsed or {}).get("error") if isinstance(parsed, dict) else None
        if isinstance(err, dict):
            out["error_code"] = err.get("code", status)
            out["message"] = str(err.get("message", ""))[:300]
        else:
            out["error_code"] = status
            out["message"] = text[:300]
        return out
    out.update(summarize(parsed))
    return out


def summarize(geojson):
    feats = geojson.get("features") or []
    if not feats:
        return {"error_code": "no-features", "message": "risposta 200 senza features"}
    f = feats[0]
    props = f.get("properties") or {}
    summary = props.get("summary") or {}
    segments = props.get("segments") or []
    coords = (f.get("geometry") or {}).get("coordinates") or []
    return {
        "routes": len(feats),
        "distance": summary.get("distance"),
        "duration": summary.get("duration"),
        "segments": len(segments),
        "steps": sum(len(s.get("steps") or []) for s in segments),
        "points": len(coords),
        "coords": coords,
    }


def health(base):
    try:
        with urllib.request.urlopen(base.rstrip("/") + "/ors/v2/health", timeout=10) as r:
            return json.loads(r.read().decode()).get("status") == "ready"
    except Exception:
        return False


# ----------------------------------------------------------------------------------------- richieste

def random_point(rnd):
    return [round(rnd.uniform(BBOX[0], BBOX[2]), 6), round(rnd.uniform(BBOX[1], BBOX[3]), 6)]


def random_coords(rnd, n):
    while True:
        pts = [random_point(rnd) for _ in range(n)]
        if all(haversine_m(*pts[i], *pts[i + 1]) >= MIN_DIST_M for i in range(n - 1)):
            return pts


def rel(a, b):
    if a is None or b is None or b == 0:
        return None
    return (a - b) / b


def fmt_pct(x, digits=1):
    return "–" if x is None else ("%+." + str(digits) + "f%%") % (100 * x)


def fmt_frac(x):
    return "–" if x is None else "%.0f%%" % (100 * x)


def median(xs):
    return statistics.median(xs) if xs else None


def run_case(name, profile, npts, make_body, n, rnd, ors, engine, out_dir):
    results = []
    case_dir = os.path.join(out_dir, "coppie", name)
    os.makedirs(case_dir, exist_ok=True)
    for i in range(n):
        coords = random_coords(rnd, npts)
        body = make_body(coords, rnd)
        r_ors = post(ors, profile, body)
        r_eng = post(engine, profile, body)
        row = compare(i, profile, body, r_ors, r_eng)
        results.append(row)
        with open(os.path.join(case_dir, "%03d.json" % i), "w") as fh:
            json.dump({"case": name, "profile": profile, "request": body,
                       "ors": r_ors.get("raw"), "engine": r_eng.get("raw"), "confronto": row_public(row)}, fh)
        print("  %s #%02d: %s" % (name, i, row["esito"]), flush=True)
    return results


def compare(i, profile, body, a, b):
    row = {"i": i, "profile": profile, "body": body,
           "ors_err": a.get("error_code"), "eng_err": b.get("error_code"),
           "ors_msg": a.get("message"), "eng_msg": b.get("message"),
           "ors_ms": a.get("ms"), "eng_ms": b.get("ms")}
    a_ok, b_ok = row["ors_err"] is None, row["eng_err"] is None
    if a_ok and b_ok:
        proj = LocalXY(body["coordinates"][0][1])
        row.update({
            "esito": "entrambi",
            "ors": {k: a.get(k) for k in ("distance", "duration", "segments", "steps", "points", "routes")},
            "eng": {k: b.get(k) for k in ("distance", "duration", "segments", "steps", "points", "routes")},
            "d_dist": rel(b.get("distance"), a.get("distance")),
            "d_dur": rel(b.get("duration"), a.get("duration")),
            "d_steps": (b.get("steps") or 0) - (a.get("steps") or 0),
            "same_segments": a.get("segments") == b.get("segments"),
            "ov_eng_in_ors": overlap_fraction(b.get("coords"), a.get("coords"), proj),
            "ov_ors_in_eng": overlap_fraction(a.get("coords"), b.get("coords"), proj),
        })
        worst = [abs(row["d_dist"] or 0), abs(row["d_dur"] or 0)]
        for ov in (row["ov_eng_in_ors"], row["ov_ors_in_eng"]):
            if ov is not None:
                worst.append(1 - ov)
        row["punteggio"] = max(worst)
    elif not a_ok and not b_ok:
        row["esito"] = "entrambi-errore" + ("" if str(row["ors_err"]) == str(row["eng_err"]) else "-diverso")
        row["punteggio"] = 0.0 if str(row["ors_err"]) == str(row["eng_err"]) else 0.5
    elif a_ok:
        row["esito"] = "solo-motore-errore"
        row["punteggio"] = 2.0
    else:
        row["esito"] = "solo-ors-errore"
        row["punteggio"] = 2.0
    return row


def row_public(row):
    return {k: v for k, v in row.items() if k != "body"}


# ------------------------------------------------------------------------------------------ rapporto

def curl_line(base, profile, body):
    return "curl -s -X POST '%s/ors/v2/directions/%s/geojson' -H 'Content-Type: application/json' -d '%s'" % (
        base, profile, json.dumps(body, separators=(",", ":")))


def case_report(name, profile, rows, ors, engine):
    both = [r for r in rows if r["esito"] == "entrambi"]
    n = len(rows)
    lines = ["### %s (`%s`)" % (name, profile), ""]
    lines.append("| esito | n |")
    lines.append("|---|---:|")
    for key, label in (("entrambi", "entrambi rispondono"), ("entrambi-errore", "entrambi rifiutano, stesso codice"),
                       ("entrambi-errore-diverso", "entrambi rifiutano, codice diverso"),
                       ("solo-ors-errore", "solo ORS rifiuta"), ("solo-motore-errore", "solo il motore rifiuta")):
        c = sum(1 for r in rows if r["esito"] == key)
        if c:
            lines.append("| %s | %d |" % (label, c))
    lines.append("| totale richieste | %d |" % n)
    lines.append("")
    if both:
        dd = [abs(r["d_dist"]) for r in both if r["d_dist"] is not None]
        du = [abs(r["d_dur"]) for r in both if r["d_dur"] is not None]
        ds = [abs(r["d_steps"]) for r in both]
        o1 = [r["ov_eng_in_ors"] for r in both if r["ov_eng_in_ors"] is not None]
        o2 = [r["ov_ors_in_eng"] for r in both if r["ov_ors_in_eng"] is not None]
        lines.append("| misura (motore rispetto a ORS) | n | entro 1% | entro 5% | mediana | massimo |")
        lines.append("|---|---:|---:|---:|---:|---:|")
        for label, xs in (("distanza", dd), ("durata", du)):
            if xs:
                lines.append("| %s | %d | %d | %d | %.2f%% | %.2f%% |" % (
                    label, len(xs), sum(1 for x in xs if x <= 0.01), sum(1 for x in xs if x <= 0.05),
                    100 * median(xs), 100 * max(xs)))
        lines.append("| tratte (stesso numero) | %d | %d | | | |" % (len(both), sum(1 for r in both if r["same_segments"])))
        lines.append("| passi (differenza assoluta) | %d | %d | %d | %.1f | %d |" % (
            len(ds), sum(1 for x in ds if x == 0), sum(1 for x in ds if x <= 2), median(ds), max(ds)))
        lines.append("")
        lines.append("| geometria (entro %d m) | n | ≥ 99%% | ≥ 95%% | mediana | minimo |" % NEAR_M)
        lines.append("|---|---:|---:|---:|---:|---:|")
        for label, xs in (("punti del motore sulla linea ORS", o1), ("punti ORS sulla linea del motore", o2)):
            if xs:
                lines.append("| %s | %d | %d | %d | %s | %s |" % (
                    label, len(xs), sum(1 for x in xs if x >= 0.99), sum(1 for x in xs if x >= 0.95),
                    fmt_frac(median(xs)), fmt_frac(min(xs))))
        if any((r["ors"].get("routes") or 1) > 1 or (r["eng"].get("routes") or 1) > 1 for r in both):
            lines.append("")
            lines.append("Percorsi alternativi restituiti (ORS / motore): " + ", ".join(
                "%d/%d" % (r["ors"].get("routes") or 0, r["eng"].get("routes") or 0) for r in both))
        ms_o = [r["ors_ms"] for r in both]
        ms_e = [r["eng_ms"] for r in both]
        lines.append("")
        lines.append("Tempo di risposta mediano: ORS %d ms, motore %d ms." % (median(ms_o), median(ms_e)))
        lines.append("")
    errs = {}
    for r in rows:
        if r["esito"] != "entrambi":
            k = (str(r["ors_err"]), str(r["eng_err"]))
            errs[k] = errs.get(k, 0) + 1
    if errs:
        lines.append("| codice ORS | codice motore | n | esempio di messaggio |")
        lines.append("|---|---|---:|---|")
        for (ko, ke), c in sorted(errs.items(), key=lambda kv: -kv[1]):
            ex = next(r for r in rows if str(r["ors_err"]) == ko and str(r["eng_err"]) == ke)
            msg = (ex["ors_msg"] if ex["ors_err"] is not None else ex["eng_msg"]) or ""
            lines.append("| %s | %s | %d | %s |" % (ko, ke, c, msg.replace("|", "\\|")[:120]))
        lines.append("")
    worst = sorted((r for r in rows if r["esito"] not in ("entrambi-errore",)), key=lambda r: -r["punteggio"])[:5]
    worst = [r for r in worst if r["punteggio"] > 0.01]
    if worst:
        lines.append("Le 5 richieste peggiori:")
        lines.append("")
        for r in worst:
            if r["esito"] == "entrambi":
                desc = "distanza %s, durata %s, passi %+d, geometria motore→ORS %s, ORS→motore %s (ORS %.0f m / %.0f s, motore %.0f m / %.0f s)" % (
                    fmt_pct(r["d_dist"], 2), fmt_pct(r["d_dur"], 2), r["d_steps"], fmt_frac(r["ov_eng_in_ors"]),
                    fmt_frac(r["ov_ors_in_eng"]), r["ors"]["distance"] or 0, r["ors"]["duration"] or 0,
                    r["eng"]["distance"] or 0, r["eng"]["duration"] or 0)
            else:
                desc = "%s: ORS %s (%s), motore %s (%s)" % (r["esito"], r["ors_err"], r["ors_msg"] or "ok", r["eng_err"], r["eng_msg"] or "ok")
            lines.append("- **#%02d** %s" % (r["i"], desc))
            lines.append("  - ORS: `%s`" % curl_line(ors, r["profile"], r["body"]))
            lines.append("  - motore: `%s`" % curl_line(engine, r["profile"], r["body"]))
        lines.append("")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ors", default="http://localhost:8080", help="base URL di openrouteservice")
    ap.add_argument("--engine", default="http://localhost:8082", help="base URL di navmaster-engine")
    ap.add_argument("--n", type=int, default=30, help="richieste per caso")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="confronto-out")
    ap.add_argument("--cases", default="", help="sottoinsieme di casi separati da virgola (vuoto = tutti)")
    args = ap.parse_args()

    for label, base in (("ORS", args.ors), ("motore", args.engine)):
        if not health(base):
            print("%s non pronto su %s/ors/v2/health" % (label, base), file=sys.stderr)
            return 2

    os.makedirs(args.out, exist_ok=True)
    wanted = [c.strip() for c in args.cases.split(",") if c.strip()]
    cases = [c for c in CASES if not wanted or c[0] in wanted]
    report = ["# Confronto openrouteservice ↔ navmaster-engine", "",
              "Estratto heidelberg.test.pbf, %d richieste per caso, seme %d, soglia geometria %d m. "
              "Differenze relative = (motore − ORS) / ORS. Le statistiche contano solo le richieste a cui "
              "rispondono entrambi; i rifiuti sono nelle tabelle degli esiti." % (args.n, args.seed, NEAR_M), "",
              "ORS: `%s` — motore: `%s`" % (args.ors, args.engine), ""]
    overview = ["| caso | profilo | entrambi | dist. entro 1%% | dist. entro 5%% | durata entro 5%% | geometria mediana | solo ORS err. | solo motore err. |",
                "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    details = []
    summary_json = {}
    t0 = time.time()
    for name, profile, npts, make_body in cases:
        rnd = random.Random("%d:%s" % (args.seed, name))
        print("caso %s (%s)" % (name, profile), flush=True)
        rows = run_case(name, profile, npts, make_body, args.n, rnd, args.ors, args.engine, args.out)
        both = [r for r in rows if r["esito"] == "entrambi"]
        dd = [abs(r["d_dist"]) for r in both if r["d_dist"] is not None]
        du = [abs(r["d_dur"]) for r in both if r["d_dur"] is not None]
        ov = [min(r["ov_eng_in_ors"], r["ov_ors_in_eng"]) for r in both if r["ov_eng_in_ors"] is not None and r["ov_ors_in_eng"] is not None]
        overview.append("| %s | %s | %d/%d | %d | %d | %d | %s | %d | %d |" % (
            name, profile, len(both), len(rows), sum(1 for x in dd if x <= 0.01), sum(1 for x in dd if x <= 0.05),
            sum(1 for x in du if x <= 0.05), fmt_frac(median(ov)),
            sum(1 for r in rows if r["esito"] == "solo-ors-errore"), sum(1 for r in rows if r["esito"] == "solo-motore-errore")))
        details += case_report(name, profile, rows, args.ors, args.engine)
        summary_json[name] = {"profile": profile, "rows": [row_public(r) | {"body": r["body"]} for r in rows]}

    report += ["## Quadro d'insieme", ""] + overview + ["", "## Dettaglio per caso", ""] + details
    report.append("Durata totale: %.0f s." % (time.time() - t0))
    text = "\n".join(report) + "\n"
    with open(os.path.join(args.out, "rapporto.md"), "w") as fh:
        fh.write(text)
    with open(os.path.join(args.out, "risultati.json"), "w") as fh:
        json.dump({"n": args.n, "seed": args.seed, "ors": args.ors, "engine": args.engine, "casi": summary_json}, fh)
    print()
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
