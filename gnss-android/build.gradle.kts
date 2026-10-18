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

// Coordinates, POM and the GitHub Packages target come from the root build file. The
// `release` component only exists once AGP has created it, hence `afterEvaluate`.
afterEvaluate {
    publishing {
        publications {
            create<MavenPublication>("release") {
                from(components["release"])
            }
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
