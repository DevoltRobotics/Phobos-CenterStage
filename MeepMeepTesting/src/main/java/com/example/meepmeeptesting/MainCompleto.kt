package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim

fun main() {
    val meepMeep = MeepMeep(600)

    val robot = DefaultBotBuilder(meepMeep)
            .setColorScheme(ColorSchemeBlueDark())
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 17.5)
            .followTrajectorySequence { drive: DriveShim ->
                drive.trajectorySequenceBuilder(Pose2d(11.0, -59.0, Math.toRadians(90.0)))
                    //.setReversed(true)
                    .splineToSplineHeading(Pose2d(47.2, -34.8, Math.toRadians(180.0)), Math.toRadians(0.0))

                    // cycle
                    .lineToConstantHeading(Vector2d(-55.5, -36.2))

                    .lineToConstantHeading(Vector2d(46.0, -34.8)) // grab
                    .lineToConstantHeading(Vector2d(47.2, -34.8))
                    .build()
            }

    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            // Add both of our declared bot entities
            .addEntity(robot)
            .start()
}