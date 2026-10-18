pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

plugins {
    // Provisions the JVM toolchain pinned by the modules (:gnss-core uses jvmToolchain(21)).
    id("org.gradle.toolchains.foojay-resolver-convention") version "1.0.0"
    // Uploads the existing maven-publish publications to the Maven Central Portal.
    id("com.gradleup.nmcp.settings") version "1.6.2"
}

nmcpSettings {
    centralPortal {
        // User token from https://central.sonatype.com/ (Account -> Generate User Token).
        // Supplied by the release workflow; never committed.
        username = providers.gradleProperty("centralPortalUsername").orNull
            ?: System.getenv("CENTRAL_PORTAL_USERNAME")
            ?: ""
        password = providers.gradleProperty("centralPortalPassword").orNull
            ?: System.getenv("CENTRAL_PORTAL_PASSWORD")
            ?: ""
        // Upload the deployment, then confirm the release by hand in the Portal UI.
        publishingType = "USER_MANAGED"
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "simt-gnss"

include(":gnss-core")
include(":gnss-android")
