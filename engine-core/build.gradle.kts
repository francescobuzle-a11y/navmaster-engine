// engine-core: graph storage, OSM import, encoded values, weightings, routing algorithms.
// No server or Android dependencies: this module must stay usable inside the NavMaster APK.
dependencies {
    // OSM PBF decoding (protobuf schema of the PBF format)
    implementation("org.openstreetmap.pbf:osmpbf:1.6.0")
    implementation("com.google.protobuf:protobuf-java:4.31.1")
    // Geometry (concave hull for isochrones, polygons for avoid areas / borders)
    implementation("org.locationtech.jts:jts-core:1.20.0")
    // Primitive collections for the hot path (no boxing)
    implementation("com.carrotsearch:hppc:0.10.0")

    testImplementation(kotlin("test"))
    testImplementation("org.junit.jupiter:junit-jupiter:5.13.1")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}
