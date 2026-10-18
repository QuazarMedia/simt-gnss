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

// Coordinates, POM and the GitHub Packages target come from the root build file.
publishing {
    publications {
        create<MavenPublication>("release") {
            from(components["java"])
        }
    }
}

// Signed only when a key is supplied; GitHub Packages does not require signatures, Maven
// Central does, so the two are one configuration away from each other.
signing {
    val signingKeyId: String? = providers.gradleProperty("signingKeyId").orNull
    val signingKey: String? = providers.gradleProperty("signingKey").orNull
    val signingPassword: String? = providers.gradleProperty("signingPassword").orNull
    if (signingKeyId != null && signingKey != null && signingPassword != null) {
        useInMemoryPgpKeys(signingKeyId, signingKey, signingPassword)
        sign(publishing.publications["release"])
    }
}
