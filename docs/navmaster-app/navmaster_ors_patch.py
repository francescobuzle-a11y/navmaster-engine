# 13) NavMaster x openrouteservice: online truck routing with real HGV restrictions
#     Paste this block into patch.py (inside apply.sh) before the final print().
#     It uses patch() and S already defined there.
#     Upstream OsmAnd OrsEngine only sends "coordinates": driving-hgv then ignores
#     height/weight/width/length/hazmat. Here we send the TRUCK profile values.
ors = os.path.join(S, 'onlinerouting', 'engine', 'OrsEngine.java')
orh = os.path.join(S, 'onlinerouting', 'OnlineRoutingHelper.java')

# 13a) OrsEngine: options field + language, and options in the body
patch(ors, '''	public OrsEngine(@Nullable Map<String, String> params) {
		super(params);
	}
''', '''	public OrsEngine(@Nullable Map<String, String> params) {
		super(params);
	}

	// NavMaster: ORS "options" (vehicle_type, profile_params.restrictions, avoid_features)
	@Nullable
	private JSONObject navmasterOptions;

	public void setNavmasterOptions(@Nullable JSONObject options) {
		this.navmasterOptions = options;
	}
''')
patch(ors, '		JSONObject jsonBody = new JSONObject();\n',
      '		JSONObject jsonBody = new JSONObject();\n'
      '		jsonBody.put("language", "it");\n'
      '		if (navmasterOptions != null && "driving-hgv".equals(getVehicleKeyForUrl())) {\n'
      '			jsonBody.put("options", navmasterOptions);\n'
      '		}\n')

# 13b) OrsEngine: roundabouts with exit number (ORS returns step.exit_number)
patch(ors, '				TurnType turnType = getTurnType(step.getInt("type"), leftSideNavigation);\n',
      '				int orsType = step.getInt("type");\n'
      '				TurnType turnType = (orsType == 7 && step.has("exit_number"))\n'
      '						? TurnType.getExitTurn(step.getInt("exit_number"), 0, leftSideNavigation)\n'
      '						: getTurnType(orsType, leftSideNavigation);\n')

# 13c) OnlineRoutingHelper: read the profile parameters before each request
patch(orh, '			String body = engine.getRequestBody(path, startBearing);\n',
      '			if (engine instanceof net.osmand.plus.onlinerouting.engine.OrsEngine) {\n'
      '				((net.osmand.plus.onlinerouting.engine.OrsEngine) engine).setNavmasterOptions(navmasterTruckOptions(params.mode));\n'
      '			}\n'
      '			String body = engine.getRequestBody(path, startBearing);\n')
patch(orh, '	public String makeRequest(@NonNull String url) throws IOException {\n',
      '''	// NavMaster: truck parameters of the profile (routing.xml: height/width/length/weight/hazmat_category, avoid_*) -> ORS options
	@NonNull
	private org.json.JSONObject navmasterTruckOptions(@Nullable net.osmand.plus.settings.backend.ApplicationMode mode) throws JSONException {
		net.osmand.plus.settings.backend.OsmandSettings s = app.getSettings();
		org.json.JSONObject restrictions = new org.json.JSONObject();
		for (String key : new String[] {"height", "width", "length", "weight"}) {
			double v = Algorithms.parseDoubleSilently(s.getCustomRoutingProperty(key, "0").getModeValue(mode), 0);
			if (v > 0) {
				restrictions.put(key, v); // ORS: metres / tonnes
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

	public String makeRequest(@NonNull String url) throws IOException {
''')
