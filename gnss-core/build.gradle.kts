import org.gradle.api.publish.maven.MavenPublication

plugins {
    alias(libs.plugins.jetbrains.kotlin.jvm)
    id("maven-publish")
    id("signing")
}

java {
    sourceCompatibility = JavaVersion.VERSION_21
    targetCompatibility = JavaVersion.VERSION_21
    withSourcesJar()
}

kotlin {
    jvmToolchain(21)
    compilerOptions {
        moduleName.set("app.simt.gnss.core")
    }
}

dependencies {
    testImplementation(libs.junit)
}

tasks.withType<Test>().configureEach {
    useJUnit()
}

// Maven Central requires a javadoc artifact and the Portal validates its presence. API docs are
// not generated yet, so this is an empty jar for now; pointing it at Dokka output is a follow-up.
val javadocJar by tasks.registering(Jar::class) {
    archiveClassifier.set("javadoc")
}

// Coordinates and POM come from the root build file; the upload is done by nmcp.
publishing {
    publications {
        create<MavenPublication>("release") {
            from(components["java"])
            artifact(javadocJar)
        }
    }
}

// Maven Central rejects unsigned artifacts, so the release workflow must supply a key. Kept
// conditional so a plain `./gradlew build` still works without one.
signing {
    val signingKey: String? = providers.gradleProperty("signingKey").orNull
    val signingPassword: String? = providers.gradleProperty("signingPassword").orNull
    if (signingKey != null && signingPassword != null) {
        useInMemoryPgpKeys(signingKey, signingPassword)
        sign(publishing.publications["release"])
    }
}
