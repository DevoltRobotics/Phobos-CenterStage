package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder

fun main() {
    val meepMeep = MeepMeep(600)

    val robot = DefaultBotBuilder(meepMeep)
            .setColorScheme(ColorSchemeBlueDark())
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 17.5)
            .followTrajectorySequence { drive: DriveShim ->
                drive.trajectorySequenceBuilder(Pose2d(10.0, 59.0, Math.toRadians(270.0))).apply {
                    path(
                            backdropPixelScoreY = 34.8,
                            spikeMarkPixelScorePose = Pose2d(33.0, 28.0, Math.toRadians(180.0)),
                            firstTrussCrossPath = {
                                lineToConstantHeading(Vector2d(35.0, 48.0))
                                lineToConstantHeading(Vector2d(8.0, 34.0))
                                lineToConstantHeading(Vector2d(-22.0, 33.0))
                            }
                    )
                }.build()
            }

    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            // Add both of our declared bot entities
            .addEntity(robot)
            .start()
}



private fun TrajectorySequenceBuilder.path(
        cycles: Int = 1,
        backdropPixelScoreY: Double = -34.8,
        spikeMarkPixelScorePose: Pose2d,
        firstTrussCrossPath: TrajectorySequenceBuilder.() -> Unit
) {
    splineToSplineHeading(
            Pose2d(51.2, backdropPixelScoreY, Math.toRadians(180.0)),
            Math.toRadians(0.0)
    )
    waitSeconds(1.5)

    repeat(cycles) {
        // cycle
        if (it == 0) { // spike mark pixel
            lineToSplineHeading(spikeMarkPixelScorePose)

            waitSeconds(1.9) // FIRST STACK GRAB

            firstTrussCrossPath()

            splineToConstantHeading( // proceed to grab
                    Vector2d(-60.0, 13.5),
                    Math.toRadians(180.0)
            )

            lineToLinearHeading(Pose2d(-50.0, 11.5, Math.toRadians(115.0)))

            lineToLinearHeading(Pose2d(-57.8, 11.5, Math.toRadians(180.0)))

            lineToLinearHeading(Pose2d(-59.0, 11.5, Math.toRadians(180.0)))
        } else {
            // TODO: Intake grab
            lineToConstantHeading(Vector2d(-32.5, 26.0)) // align to truss
            splineToConstantHeading(Vector2d(-52.8, 9.5), Math.toRadians(180.0)) // proceed to grab
        }

        waitSeconds(1.4)

        lineToConstantHeading(Vector2d(-22.3, 6.1)) // Align to cross stage door

        splineToConstantHeading(Vector2d(50.0, 35.2), Math.toRadians(270.0)) // Align to backdrop

        waitSeconds(2.9)
    }
}
