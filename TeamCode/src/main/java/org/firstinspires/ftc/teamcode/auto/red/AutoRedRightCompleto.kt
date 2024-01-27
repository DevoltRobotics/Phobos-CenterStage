package org.firstinspires.ftc.teamcode.auto.red

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.github.serivesmejia.deltacommander.endRightAway
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.command.box.BoxArmDownCmd
import org.firstinspires.ftc.teamcode.command.box.BoxArmPositionCmd
import org.firstinspires.ftc.teamcode.command.box.BoxDoorsCloseCmd
import org.firstinspires.ftc.teamcode.command.box.BoxLeftDoorCloseCmd
import org.firstinspires.ftc.teamcode.command.box.BoxLeftDoorOpenCmd
import org.firstinspires.ftc.teamcode.command.intake.IntakeDoorCloseCmd
import org.firstinspires.ftc.teamcode.command.intake.IntakeDoorOpenCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmWristPositionCmd
import org.firstinspires.ftc.teamcode.command.lift.LiftGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeAbsorbCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeReleaseCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeStopCmd
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.vision.Pattern
import org.firstinspires.ftc.teamcode.vision.Pattern.*

@Autonomous(name = "R - Derecha Completo", group ="###RFINAL")
class AutoRedRightCompleto : PhobosAuto(Alliance.RED) {

    override val startPose = Pose2d(10.0, -59.0, Math.toRadians(90.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {
        when(pattern) {
            A -> path( // A
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(32.0, -24.0, Math.toRadians(180.0)),
                trussCrossRealignPose = Pose2d(32.0, -34.0, Math.toRadians(180.0))
            )
            C -> path( // C
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(32.0, -24.0, Math.toRadians(180.0)),
                trussCrossRealignPose = Pose2d(32.0, -34.0, Math.toRadians(180.0))
            )
            else -> path( // B
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(32.0, -24.0, Math.toRadians(180.0)),
                trussCrossRealignPose = Pose2d(32.0, -34.0, Math.toRadians(180.0))
            )
        }
    }.build()

    private fun TrajectorySequenceBuilder.path(
            cycles: Int = 2,
            backdropPixelScoreY: Double = -34.8,
            spikeMarkPixelScorePose: Pose2d,
            trussCrossRealignPose: Pose2d,
    ) {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + LiftGoToPositionCmd(200)
            + IntakeArmWristPositionCmd(0.42).endRightAway()
            + BoxDoorsCloseCmd().endRightAway()
        }

        UNSTABLE_addTemporalMarkerOffset(1.0) {
            + BoxArmPositionCmd(0.6)
        }

        splineToSplineHeading(Pose2d(50.2, backdropPixelScoreY, Math.toRadians(180.0)), Math.toRadians(0.0))

        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + BoxLeftDoorOpenCmd()
        }

        UNSTABLE_addTemporalMarkerOffset(2.0) {
            + BoxArmDownCmd()
            + BoxLeftDoorCloseCmd()
            + LiftGoToPositionCmd(0)
        }

        repeat(cycles) {
            waitSeconds(2.0)

            // cycle
            if(it == 1) {
                UNSTABLE_addTemporalMarkerOffset(0.0) {
                    + IntakeArmGoToPositionCmd(-230)
                }

                lineToSplineHeading(spikeMarkPixelScorePose)

                UNSTABLE_addTemporalMarkerOffset(0.0) {
                    + IntakeArmWristPositionCmd(0.6).endRightAway()
                    + IntakeReleaseCmd()
                }

                waitSeconds(1.5)

                UNSTABLE_addTemporalMarkerOffset(0.0) {
                    + IntakeArmGoToPositionCmd(0)
                    + IntakeArmWristPositionCmd(0.42).endRightAway()
                    + IntakeStopCmd()
                }

                lineToSplineHeading(trussCrossRealignPose)
            }

            splineToConstantHeading(Vector2d(-32.5, -32.0), Math.toRadians(180.0))

            UNSTABLE_addTemporalMarkerOffset(0.0) {
                + IntakeAbsorbCmd()
                + IntakeArmWristPositionCmd(0.54).endRightAway()
            }
            UNSTABLE_addTemporalMarkerOffset(0.2) {
                + IntakeArmGoToPositionCmd(-240)
            }
            splineToConstantHeading(Vector2d(-56.0, -11.5), Math.toRadians(180.0))

            waitSeconds(2.0)

            UNSTABLE_addTemporalMarkerOffset(0.0) {
                + IntakeArmWristPositionCmd(0.42).endRightAway()
                + IntakeArmGoToPositionCmd(0)
            }

            UNSTABLE_addTemporalMarkerOffset(3.0) {
                + IntakeDoorOpenCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(3.5) {
                + IntakeDoorCloseCmd()
            }
            lineToConstantHeading(Vector2d(-7.3, -7.8))

            UNSTABLE_addTemporalMarkerOffset(3.0) {
                + LiftGoToPositionCmd(200)
            }

            UNSTABLE_addTemporalMarkerOffset(4.0) {
                + BoxArmPositionCmd(0.6)
            }
            splineToConstantHeading(Vector2d(50.2, -34.8), Math.toRadians(270.0))

            UNSTABLE_addTemporalMarkerOffset(0.0) {
                + BoxLeftDoorOpenCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(1.8) {
                + BoxArmDownCmd()
                + BoxLeftDoorCloseCmd()
                + LiftGoToPositionCmd(0)
            }
            waitSeconds(2.0)
        }
    }

}