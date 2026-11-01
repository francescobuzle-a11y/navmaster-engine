// engine-server: HTTP API compatibile con openrouteservice v2 (/ors/v2/directions/{profile} ...),
// così il client ORS già presente in NavMaster/OsmAnd funziona senza modifiche.
plugins {
    application
}

dependencies {
    implementation(project(":engine-core"))
    implementation("io.ktor:ktor-server-core:3.1.3")
    implementation("io.ktor:ktor-server-netty:3.1.3")
    implementation("io.ktor:ktor-server-cors:3.1.3")
    implementation("io.ktor:ktor-server-status-pages:3.1.3")
    implementation("ch.qos.logback:logback-classic:1.5.18")

    testImplementation(kotlin("test"))
    testImplementation("io.ktor:ktor-server-test-host:3.1.3")
    testImplementation("org.junit.jupiter:junit-jupiter:5.13.1")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

application {
    mainClass.set("it.navmaster.server.MainKt")
    applicationDefaultJvmArgs = listOf("-Xmx2g")
}

tasks.named<JavaExec>("run") {
    workingDir = rootProject.projectDir
}
