#!/bin/sh
# Costruisce il grafo se manca e avvia il server.
set -e
if [ ! -f /graph/profiles ]; then
  if [ -n "$IMPORT_PBF" ] && [ -f "$IMPORT_PBF" ]; then
    echo "nessun grafo in /graph: lo costruisco da $IMPORT_PBF"
    JAVA_OPTS="$JAVA_OPTS" /app/cli/bin/engine-cli import "$IMPORT_PBF" /graph --profiles=car,hgv
  else
    echo "nessun grafo in /graph e IMPORT_PBF non impostato" >&2
    exit 1
  fi
fi
ARGS="--graph=/graph --port=${PORT:-8082}"
[ -n "$NAVMASTER_API_KEY" ] && ARGS="$ARGS --api-key=$NAVMASTER_API_KEY"
exec env JAVA_OPTS="$JAVA_OPTS" /app/server/bin/engine-server $ARGS
