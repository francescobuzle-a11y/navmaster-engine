package net.osmand.plus.onlinerouting.engine;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import net.osmand.plus.OsmandApplication;
import net.osmand.plus.R;
import net.osmand.plus.onlinerouting.EngineParameter;
import net.osmand.plus.onlinerouting.VehicleType;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Set;

import it.navmaster.engine.RoutingGraph;
import it.navmaster.engine.api.ApiException;
import it.navmaster.engine.api.DirectionsLimits;
import it.navmaster.engine.api.DirectionsRequest;
import it.navmaster.engine.api.DirectionsService;
import it.navmaster.engine.api.Json;
import it.navmaster.engine.api.ResponseFormat;

/**
 * NavMaster: motore di routing OFFLINE. Si presenta a OsmAnd come un "motore online" di tipo
 * openrouteservice, ma invece di una richiesta HTTP calcola il percorso in locale con
 * navmaster-engine e restituisce lo stesso GeoJSON che restituirebbe openrouteservice,
 * così il parser di {@link OrsEngine} viene riusato senza modifiche.
 *
 * Il grafo va copiato nella cartella dati di OsmAnd, sottocartella {@value #GRAPH_DIR}
 * (file profiles, nodes, edges, geometry, names, turn_costs, shortcuts_*, levels_*, landmarks_*).
 */
public class NavMasterLocalEngine extends OrsEngine {

	public static final String GRAPH_DIR = "navmaster/graph";
	public static final String TYPE_NAME = "NAVMASTER";

	private static volatile DirectionsService service;
	private static volatile String loadedPath;

	public NavMasterLocalEngine(@Nullable Map<String, String> params) {
		super(params);
	}

	@NonNull
	@Override
	public OnlineRoutingEngine getType() {
		return EngineType.NAVMASTER_TYPE;
	}

	@NonNull
	@Override
	public String getTitle() {
		return "NavMaster (offline)";
	}

	@NonNull
	@Override
	public String getTypeName() {
		return TYPE_NAME;
	}

	@NonNull
	@Override
	public String getStandardUrl() {
		return "local://navmaster/";
	}

	@Override
	protected void collectAllowedParameters(@NonNull Set<EngineParameter> params) {
		params.add(EngineParameter.KEY);
		params.add(EngineParameter.VEHICLE_KEY);
		params.add(EngineParameter.CUSTOM_NAME);
		params.add(EngineParameter.NAME_INDEX);
	}

	@Override
	protected void collectAllowedVehicles(@NonNull List<VehicleType> vehicles) {
		vehicles.add(new VehicleType("driving-hgv", R.string.routing_engine_vehicle_type_hgv));
		vehicles.add(new VehicleType("driving-car", R.string.routing_engine_vehicle_type_car));
	}

	@Override
	public OnlineRoutingEngine newInstance(Map<String, String> params) {
		return new NavMasterLocalEngine(params);
	}

	/** Cartella del grafo nella cartella dati di OsmAnd. */
	@NonNull
	public static File getGraphDir(@NonNull OsmandApplication app) {
		return app.getAppPath(GRAPH_DIR);
	}

	/** Carica il grafo una volta sola (memory-mapped: la RAM usata resta bassa). */
	@NonNull
	public static synchronized DirectionsService getService(@NonNull OsmandApplication app) throws IOException {
		File dir = getGraphDir(app);
		if (service != null && dir.getAbsolutePath().equals(loadedPath)) {
			return service;
		}
		if (!new File(dir, "profiles").exists()) {
			throw new IOException("Grafo NavMaster non trovato: copia la cartella del grafo in " + dir.getAbsolutePath());
		}
		try {
			RoutingGraph graph = RoutingGraph.Companion.load(dir, true);
			DirectionsLimits limits = new DirectionsLimits(50, 3_000_000.0, 3_000_000.0, 400.0);
			service = new DirectionsService(graph, limits, "0.1.0");
			loadedPath = dir.getAbsolutePath();
			return service;
		} catch (Exception e) {
			throw new IOException("Impossibile caricare il grafo NavMaster da " + dir.getAbsolutePath() + ": " + e.getMessage(), e);
		}
	}

	/**
	 * Calcola il percorso in locale. Riceve il corpo JSON che {@link OrsEngine#getRequestBody}
	 * avrebbe inviato al server e restituisce il GeoJSON di openrouteservice (o il suo oggetto d'errore).
	 */
	@NonNull
	@SuppressWarnings("unchecked")
	public static String compute(@NonNull OsmandApplication app, @Nullable String profile, @NonNull String body) throws IOException {
		DirectionsService s = getService(app);
		String p = profile == null || profile.isEmpty() ? "driving-hgv" : profile;
		try {
			Object parsed = Json.INSTANCE.parse(body);
			DirectionsRequest request = DirectionsRequest.Companion.fromJson(p, (Map<String, Object>) parsed);
			return Json.INSTANCE.stringify(s.handle(request, ResponseFormat.GEOJSON));
		} catch (ApiException e) {
			return Json.INSTANCE.stringify(s.errorBody(e));
		} catch (RuntimeException e) {
			throw new IOException("Errore del motore NavMaster: " + e.getMessage(), e);
		}
	}
}
