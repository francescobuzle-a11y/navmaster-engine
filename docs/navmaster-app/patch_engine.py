#!/usr/bin/env python3
# NavMaster Truck - integrazione del motore di routing offline (navmaster-engine) in OsmAnd.
# Eseguito da apply.sh dopo patch.py:  python3 android/navmaster/patch_engine.py android
#
# Cosa fa:
#   1) aggiunge il jar del motore (navmaster/libs/navmaster-engine-core.jar) alle dipendenze dell'app;
#   2) copia NavMasterLocalEngine.java e lo registra fra i tipi di motore;
#   3) OnlineRoutingHelper: le richieste del motore NavMaster vengono calcolate in locale,
#      e per tutti i motori openrouteservice (locale compreso) vengono inviate le misure del camion
#      del profilo (altezza, larghezza, lunghezza, peso, merci pericolose, pedaggi/traghetti/autostrade);
#   4) OrsEngine: campo `options`, istruzioni in italiano, rotonde con numero d'uscita;
#   5) la scheda "Prova" del motore funziona anche in locale.
import os, shutil, sys

A = sys.argv[1]                      # radice del repository android
O = os.path.join(A, 'OsmAnd')
S = os.path.join(O, 'src', 'net', 'osmand', 'plus')
NM = os.path.join(A, 'navmaster')


def patch(path, old, new, count=1):
    s = open(path, encoding='utf-8').read()
    n = s.count(old)
    if n < count:
        sys.exit(f'PATCH FAILED: {path}\n  pattern not found: {old[:120]!r}')
    s = s.replace(old, new)
    open(path, 'w', encoding='utf-8').write(s)
    print(f'patched {os.path.relpath(path, A)} ({n}x)')


# 1) dipendenza sul jar del motore
jar = os.path.join(NM, 'libs', 'navmaster-engine-core.jar')
if not os.path.exists(jar):
    sys.exit(f'manca {jar}: copia il jar prodotto da `./gradlew :engine-core:androidJar` di navmaster-engine')
patch(os.path.join(O, 'build.gradle'), '\ndependencies {\n',
      "\ndependencies {\n\t// NavMaster: motore di routing offline\n\timplementation files('../navmaster/libs/navmaster-engine-core.jar')\n")

# 2) motore locale registrato fra i tipi
eng_dir = os.path.join(S, 'onlinerouting', 'engine')
shutil.copy(os.path.join(NM, 'NavMasterLocalEngine.java'), os.path.join(eng_dir, 'NavMasterLocalEngine.java'))
et = os.path.join(eng_dir, 'EngineType.java')
patch(et, '\tpublic static final OnlineRoutingEngine GPX_TYPE = new GpxEngine(null);\n',
      '\tpublic static final OnlineRoutingEngine GPX_TYPE = new GpxEngine(null);\n'
      '\tpublic static final OnlineRoutingEngine NAVMASTER_TYPE = new NavMasterLocalEngine(null);\n')
patch(et, '\t\t\t\t\tORS_TYPE,\n\t\t\t\t\tGPX_TYPE\n', '\t\t\t\t\tNAVMASTER_TYPE,\n\t\t\t\t\tORS_TYPE,\n\t\t\t\t\tGPX_TYPE\n')

# 3) OnlineRoutingHelper: calcolo locale + misure del camion per i motori openrouteservice
orh = os.path.join(S, 'onlinerouting', 'OnlineRoutingHelper.java')
patch(orh, '\t\t\tString body = engine.getRequestBody(path, startBearing);\n'
           '\t\t\tMap<String, String> headers = engine.getRequestHeaders();\n'
           '\t\t\tString content = makeRequest(url, method, body, headers);\n',
      '\t\t\tif (engine instanceof net.osmand.plus.onlinerouting.engine.OrsEngine) {\n'
      '\t\t\t\t((net.osmand.plus.onlinerouting.engine.OrsEngine) engine).setNavmasterOptions(navmasterTruckOptions(params.mode));\n'
      '\t\t\t}\n'
      '\t\t\tString body = engine.getRequestBody(path, startBearing);\n'
      '\t\t\tMap<String, String> headers = engine.getRequestHeaders();\n'
      '\t\t\tString content = engine instanceof net.osmand.plus.onlinerouting.engine.NavMasterLocalEngine\n'
      '\t\t\t\t\t? net.osmand.plus.onlinerouting.engine.NavMasterLocalEngine.compute(app, engine.getVehicleKeyForUrl(), body)\n'
      '\t\t\t\t\t: makeRequest(url, method, body, headers);\n')
patch(orh, '\t@NonNull\n\tpublic String makeRequest(@NonNull String url) throws IOException {\n',
      '''	// NavMaster: parametri camion del profilo (routing.xml: height/width/length/weight/hazmat_category, avoid_*) -> opzioni openrouteservice
	@NonNull
	private org.json.JSONObject navmasterTruckOptions(@Nullable net.osmand.plus.settings.backend.ApplicationMode mode) throws JSONException {
		net.osmand.plus.settings.backend.OsmandSettings s = app.getSettings();
		org.json.JSONObject restrictions = new org.json.JSONObject();
		for (String key : new String[] {"height", "width", "length", "weight"}) {
			double v = Algorithms.parseDoubleSilently(s.getCustomRoutingProperty(key, "0").getModeValue(mode), 0);
			if (v > 0) {
				restrictions.put(key, v); // metri / tonnellate
			}
		}
		if (!"0".equals(s.getCustomRoutingProperty("hazmat_category", "0").getModeValue(mode))) {
			restrictions.put("hazmat", true);
		}
		org.json.JSONObject options = new org.json.JSONObject();
		options.put("vehicle_type", "hgv");
		if (restrictions.length() > 0) {
			options.put("profile_params", new org.json.JSONObject().put("restrictions", restrictions));
		}
		org.json.JSONArray avoid = new org.json.JSONArray();
		if (s.getCustomRoutingBooleanProperty("avoid_toll", false).getModeValue(mode)) avoid.put("tollways");
		if (s.getCustomRoutingBooleanProperty("avoid_ferries", false).getModeValue(mode)) avoid.put("ferries");
		if (s.getCustomRoutingBooleanProperty("avoid_motorway", false).getModeValue(mode)) avoid.put("highways");
		if (avoid.length() > 0) {
			options.put("avoid_features", avoid);
		}
		return options;
	}

	@NonNull
	public String makeRequest(@NonNull String url) throws IOException {
''')

# 4) OrsEngine: opzioni, lingua, rotonde con numero d'uscita
ors = os.path.join(eng_dir, 'OrsEngine.java')
patch(ors, '''	public OrsEngine(@Nullable Map<String, String> params) {
		super(params);
	}
''', '''	public OrsEngine(@Nullable Map<String, String> params) {
		super(params);
	}

	// NavMaster: "options" openrouteservice (vehicle_type, profile_params.restrictions, avoid_features)
	@Nullable
	private JSONObject navmasterOptions;

	public void setNavmasterOptions(@Nullable JSONObject options) {
		this.navmasterOptions = options;
	}
''')
patch(ors, '\t\tJSONObject jsonBody = new JSONObject();\n',
      '\t\tJSONObject jsonBody = new JSONObject();\n'
      '\t\tjsonBody.put("language", "it");\n'
      '\t\tif (navmasterOptions != null && "driving-hgv".equals(getVehicleKeyForUrl())) {\n'
      '\t\t\tjsonBody.put("options", navmasterOptions);\n'
      '\t\t}\n')
patch(ors, '\t\t\t\tTurnType turnType = getTurnType(step.getInt("type"), leftSideNavigation);\n',
      '\t\t\t\tint orsType = step.getInt("type");\n'
      '\t\t\t\tTurnType turnType = (orsType == 7 && step.has("exit_number"))\n'
      '\t\t\t\t\t\t? TurnType.getExitTurn(step.getInt("exit_number"), 0, leftSideNavigation)\n'
      '\t\t\t\t\t\t: getTurnType(orsType, leftSideNavigation);\n')

# 5) scheda "Prova" del motore: calcolo locale invece della richiesta HTTP
frag = os.path.join(S, 'onlinerouting', 'ui', 'OnlineRoutingEngineFragment.java')
patch(frag, '\t\t\t\tString response = helper.makeRequest(exampleCard.getEditedText(), method, body, headers);\n',
      '\t\t\t\tString response = requestedEngine instanceof net.osmand.plus.onlinerouting.engine.NavMasterLocalEngine\n'
      '\t\t\t\t\t\t? net.osmand.plus.onlinerouting.engine.NavMasterLocalEngine.compute((net.osmand.plus.OsmandApplication) requireActivity().getApplication(), requestedEngine.getVehicleKeyForUrl(), body)\n'
      '\t\t\t\t\t\t: helper.makeRequest(exampleCard.getEditedText(), method, body, headers);\n')

print('NavMaster engine patches applied OK')
