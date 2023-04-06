import org.jetbrains.kotlin.gradle.plugin.KotlinPlatformJvmPlugin
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile
apply(plugin = "java-common")
apply<KotlinPlatformJvmPlugin>()
dependencies {
    val implementation: Configuration by configurations
    implementation(Deps.Kotlin.stdlib)
}
repositories {
    mavenCentral()
}
tasks {
    withType<KotlinCompile>() {
        kotlinOptions {
            freeCompilerArgs = listOf(
                "-Xjsr305=strict",
            )
            jvmTarget = Versions.java
        }
    }
}