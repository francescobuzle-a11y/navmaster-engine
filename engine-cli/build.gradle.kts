// engine-cli: build graphs from OSM files and run routes from the command line.
plugins {
    application
}

dependencies {
    implementation(project(":engine-core"))
}

application {
    mainClass.set("it.navmaster.cli.MainKt")
    applicationDefaultJvmArgs = listOf("-Xmx4g")
}

// `./gradlew :engine-cli:run --args="..."` lavora dalla radice del progetto, così i percorsi relativi (data/, graphs/) funzionano
tasks.named<JavaExec>("run") {
    workingDir = rootProject.projectDir
}

dependencies {
    testImplementation(kotlin("test"))
    testImplementation("org.junit.jupiter:junit-jupiter:5.13.1")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}
