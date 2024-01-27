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
                drive.trajectorySequenceBuilder(Pose2d(10.0, -59.0, Math.toRadians(90.0))).apply {
                    path( // B
                            backdropPixelScoreY = -34.8,
                            spikeMarkPixelScorePose = Pose2d(23.0, -44.0, Math.toRadians(90.0)),
                            firstTrussCrossPath = {
                                setReversed(true)
                                splineToConstantHeading(Vector2d(10.0, -35.0), Math.toRadians(180.0))

                                splineToConstantHeading(Vector2d(-33.0, -35.0), Math.toRadians(180.0))
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
        cycles: Int = 2,
        backdropPixelScoreY: Double = -34.8,
        spikeMarkPixelScorePose: Pose2d,
        firstTrussCrossPath: TrajectorySequenceBuilder.() -> Unit
) {
    UNSTABLE_addTemporalMarkerOffset(0.0) {
    }

    UNSTABLE_addTemporalMarkerOffset(1.0) {
    }

    splineToSplineHeading(Pose2d(50.2, backdropPixelScoreY, Math.toRadians(180.0)), Math.toRadians(0.0))

    UNSTABLE_addTemporalMarkerOffset(0.0) {
    }

    UNSTABLE_addTemporalMarkerOffset(2.0) {
    }

    repeat(cycles) {
        waitSeconds(2.0)

        // cycle
        if(it == 0) {
            UNSTABLE_addTemporalMarkerOffset(0.0) {
            }

            lineToSplineHeading(spikeMarkPixelScorePose)

            UNSTABLE_addTemporalMarkerOffset(0.0) {
            }

            waitSeconds(1.5)

            UNSTABLE_addTemporalMarkerOffset(0.0) {
            }

            firstTrussCrossPath()
            splineToSplineHeading(Pose2d(-56.0, -11.5, Math.toRadians(180.0)), Math.toRadians(180.0))
        } else {
            lineToConstantHeading(Vector2d(-32.5, -34.0))
            splineToConstantHeading(Vector2d(-56.0, -11.5), Math.toRadians(180.0))
        }

        UNSTABLE_addTemporalMarkerOffset(0.0) {
        }
        UNSTABLE_addTemporalMarkerOffset(0.2) {
        }
        waitSeconds(2.0)

        UNSTABLE_addTemporalMarkerOffset(0.0) {
        }

        UNSTABLE_addTemporalMarkerOffset(3.0) {
        }
        UNSTABLE_addTemporalMarkerOffset(3.5) {
        }
        lineToConstantHeading(Vector2d(-7.3, -7.8))

        UNSTABLE_addTemporalMarkerOffset(3.0) {
        }

        UNSTABLE_addTemporalMarkerOffset(4.0) {
        }
        splineToConstantHeading(Vector2d(50.2, -34.8), Math.toRadians(270.0))

        UNSTABLE_addTemporalMarkerOffset(0.0) {
        }
        UNSTABLE_addTemporalMarkerOffset(1.8) {
        }
        waitSeconds(2.0)
    }
}
