import org.gradle.api.publish.maven.MavenPublication

plugins {
    // AGP 9 provides built-in Kotlin support; no `org.jetbrains.kotlin.android` plugin needed.
    alias(libs.plugins.android.library)
    id("maven-publish")
    id("signing")
}

android {
    namespace = "app.simt.gnss"
    compileSdk = libs.versions.compileSdk.get().toInt()

    defaultConfig {
        minSdk = libs.versions.minSdk.get().toInt()
        consumerProguardFiles("consumer-rules.pro")
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_21
        targetCompatibility = JavaVersion.VERSION_21
    }

    testOptions {
        unitTests.isIncludeAndroidResources = true
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
        }
    }
}

dependencies {
    api(project(":gnss-core"))

    testImplementation(libs.junit)
    testImplementation(libs.robolectric)
}

// Maven Central requires a javadoc artifact and the Portal validates its presence. API docs are
// not generated yet, so this is an empty jar for now; pointing it at Dokka output is a follow-up.
val javadocJar by tasks.registering(Jar::class) {
    archiveClassifier.set("javadoc")
}

// Coordinates and POM come from the root build file; the upload is done by nmcp. The `release`
// component only exists once AGP has created it, hence `afterEvaluate`. Signing is wired in the
// same block: signing earlier would look up a publication that does not exist yet.
afterEvaluate {
    publishing {
        publications {
            create<MavenPublication>("release") {
                from(components["release"])
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
}
