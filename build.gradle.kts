// Top-level build file. Plugin versions are pinned in gradle/libs.versions.toml.
plugins {
    alias(libs.plugins.android.library) apply false
    alias(libs.plugins.jetbrains.kotlin.android) apply false
    alias(libs.plugins.jetbrains.kotlin.jvm) apply false
    alias(libs.plugins.spotless) apply false
}

subprojects {
    apply(plugin = "com.diffplug.spotless")

    extensions.configure<com.diffplug.gradle.spotless.SpotlessExtension>("spotless") {
        kotlin {
            target("**/*.kt")
            targetExclude("**/build/**", "**/bin/**")
            ktlint(libs.versions.ktlint.get())
                .setEditorConfigPath("$rootDir/.editorconfig")
            trimTrailingWhitespace()
            endWithNewline()
        }

        kotlinGradle {
            target("**/*.gradle.kts")
            targetExclude("**/build/**")
            ktlint(libs.versions.ktlint.get())
            trimTrailingWhitespace()
            endWithNewline()
        }
    }
}
