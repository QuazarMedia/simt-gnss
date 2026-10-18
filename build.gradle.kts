import org.gradle.api.publish.PublishingExtension
import org.gradle.api.publish.maven.MavenPublication

// Top-level build file. Plugin versions are pinned in gradle/libs.versions.toml.
plugins {
    alias(libs.plugins.android.library) apply false
    alias(libs.plugins.jetbrains.kotlin.android) apply false
    alias(libs.plugins.jetbrains.kotlin.jvm) apply false
    alias(libs.plugins.spotless) apply false
}

// The published version. Overridable from the command line (`-PVERSION_NAME=…`), which is
// how the release workflow derives it from the git tag.
val libraryVersion: String = providers.gradleProperty("VERSION_NAME").getOrElse("0.1.0")

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

    // Publishing is shared by the modules that opt in with `maven-publish` (see
    // gnss-core/build.gradle.kts and gnss-android/build.gradle.kts). Coordinates, POM
    // metadata and the target repository are declared once here so the two published
    // artifacts cannot drift apart.
    plugins.withId("maven-publish") {
        group = "app.simt"
        version = libraryVersion

        extensions.configure<PublishingExtension> {
            publications.withType<MavenPublication>().configureEach {
                pom {
                    name.set(project.name)
                    description.set("SIMT GNSS engine library — ${project.name}")
                    url.set("https://github.com/QuazarMedia/simt-gnss")

                    licenses {
                        license {
                            name.set("The Apache License, Version 2.0")
                            url.set("https://www.apache.org/licenses/LICENSE-2.0.txt")
                        }
                    }

                    developers {
                        developer {
                            id.set("saidelimam")
                            name.set("Said Elimam")
                            email.set("contact@simt.app")
                        }
                    }

                    scm {
                        connection.set("scm:git:git://github.com/QuazarMedia/simt-gnss.git")
                        developerConnection.set("scm:git:ssh://github.com:QuazarMedia/simt-gnss.git")
                        url.set("https://github.com/QuazarMedia/simt-gnss/tree/main")
                    }
                }
            }

            repositories {
                maven {
                    name = "GitHubPackages"
                    url = uri("https://maven.pkg.github.com/QuazarMedia/simt-gnss")
                    credentials {
                        username = project.findProperty("GITHUB_USER") as String? ?: System.getenv("GITHUB_USER")
                        password = project.findProperty("GITHUB_TOKEN") as String? ?: System.getenv("GITHUB_TOKEN")
                    }
                }
            }
        }
    }
}
