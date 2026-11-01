plugins {
    id("com.gradleup.shadow") version "8.3.6"
}

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
// Esclude la stdlib Kotlin, che l'app ha già. protobuf e osmpbf vengono rilocati in it.navmaster.shaded,
// perché OsmAnd include una propria (vecchia) versione di com.google.protobuf e R8 non accetta classi duplicate.
tasks.register<com.github.jengelman.gradle.plugins.shadow.tasks.ShadowJar>("androidJar") {
    group = "build"
    description = "Jar del motore con le dipendenze, per l'app Android"
    archiveBaseName.set("navmaster-engine-core")
    archiveClassifier.set("")
    archiveVersion.set("")
    from(sourceSets.main.get().output)
    configurations = listOf(project.configurations.runtimeClasspath.get())
    dependencies {
        exclude(dependency("org.jetbrains.kotlin:.*"))
        exclude(dependency("org.jetbrains:annotations"))
    }
    relocate("com.google.protobuf", "it.navmaster.shaded.protobuf")
    relocate("crosby.binary", "it.navmaster.shaded.osmpbf")
    exclude("META-INF/*.SF", "META-INF/*.DSA", "META-INF/*.RSA", "META-INF/versions/**", "module-info.class",
        "META-INF/LICENSE*", "META-INF/NOTICE*", "META-INF/DEPENDENCIES", "META-INF/maven/**", "META-INF/proguard/**", "META-INF/*.kotlin_module", "**/*.proto")
}
