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

// Jar unico (motore + hppc + JTS + lettore PBF) da includere nell'APK di NavMaster/OsmAnd.
// Esclude la stdlib Kotlin, che l'app ha già.
tasks.register<Jar>("androidJar") {
    group = "build"
    description = "Jar del motore con le dipendenze, per l'app Android"
    archiveBaseName.set("navmaster-engine-core")
    archiveVersion.set("")
    duplicatesStrategy = DuplicatesStrategy.EXCLUDE
    from(sourceSets.main.get().output)
    dependsOn(configurations.runtimeClasspath)
    from({
        configurations.runtimeClasspath.get()
            .filter { it.name.endsWith(".jar") && !it.name.startsWith("kotlin-stdlib") && !it.name.startsWith("annotations") }
            .map { zipTree(it) }
    })
    exclude("META-INF/*.SF", "META-INF/*.DSA", "META-INF/*.RSA", "META-INF/versions/**", "module-info.class",
        "META-INF/LICENSE*", "META-INF/NOTICE*", "META-INF/DEPENDENCIES", "META-INF/maven/**", "META-INF/proguard/**", "META-INF/*.kotlin_module", "**/*.proto")
}
