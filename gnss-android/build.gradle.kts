plugins {
    // AGP 9 provides built-in Kotlin support; no `org.jetbrains.kotlin.android` plugin needed.
    alias(libs.plugins.android.library)
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
}

dependencies {
    api(project(":gnss-core"))

    testImplementation(libs.junit)
    testImplementation(libs.robolectric)
}
