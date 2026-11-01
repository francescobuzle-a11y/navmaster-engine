# navmaster-engine: immagine del server HTTP compatibile openrouteservice.
#
#   docker build -t navmaster-engine .
#   docker run -v ./graphs/lombardia:/graph -p 8082:8082 navmaster-engine
#
# Il grafo va costruito prima (engine-cli import) e montato in /graph; in alternativa montare il PBF in
# /data e avviare con IMPORT_PBF=/data/regione.osm.pbf per costruirlo al primo avvio.
FROM eclipse-temurin:17-jdk AS build
WORKDIR /src
COPY . .
RUN ./gradlew --no-daemon -q :engine-server:installDist :engine-cli:installDist

FROM eclipse-temurin:17-jre
ENV JAVA_OPTS="-Xmx3g" PORT=8082 NAVMASTER_API_KEY=""
WORKDIR /app
COPY --from=build /src/engine-server/build/install/engine-server /app/server
COPY --from=build /src/engine-cli/build/install/engine-cli /app/cli
COPY docker-entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh
VOLUME ["/graph", "/data"]
EXPOSE 8082
HEALTHCHECK --start-period=120s --interval=30s --timeout=5s CMD curl -fs http://localhost:8082/ors/v2/health || exit 1
ENTRYPOINT ["/app/entrypoint.sh"]
