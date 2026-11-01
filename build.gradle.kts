plugins {
    kotlin("jvm") version "2.1.21" apply false
}

allprojects {
    group = "it.navmaster"
    version = "0.1.0-SNAPSHOT"
}

subprojects {
    apply(plugin = "org.jetbrains.kotlin.jvm")

    extensions.configure<org.jetbrains.kotlin.gradle.dsl.KotlinJvmProjectExtension> {
        // JVM 17: runs on the server and inside the Android app (desugaring not needed).
        jvmToolchain(17)
    }

    tasks.withType<Test>().configureEach {
        useJUnitPlatform()
        maxHeapSize = "2g"
    }
}
