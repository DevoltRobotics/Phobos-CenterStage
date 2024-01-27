package org.firstinspires.ftc.teamcode.auto.red

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.github.serivesmejia.deltacommander.endRightAway
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.command.box.BoxArmDownCmd
import org.firstinspires.ftc.teamcode.command.box.BoxArmMiddleCmd
import org.firstinspires.ftc.teamcode.command.box.BoxArmPositionCmd
import org.firstinspires.ftc.teamcode.command.box.BoxDoorsCloseCmd
import org.firstinspires.ftc.teamcode.command.box.BoxDoorsOpenCmd
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
            A -> path( // B
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(23.0, -44.0, Math.toRadians(90.0)),
                firstTrussCrossPath = {
                    setReversed(true)
                    splineToConstantHeading(Vector2d(10.0, -33.0), Math.toRadians(180.0))

                    splineToConstantHeading(Vector2d(-33.0, -33.0), Math.toRadians(180.0))
                }
            )
            C -> path( // B
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(23.0, -44.0, Math.toRadians(90.0)),
                firstTrussCrossPath = {
                    setReversed(true)
                    splineToConstantHeading(Vector2d(10.0, -33.0), Math.toRadians(180.0))
                    splineToConstantHeading(Vector2d(-33.0, -33.0), Math.toRadians(180.0))
                }
            )
            else -> path( // B
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(23.0, -44.0, Math.toRadians(90.0)),
                firstTrussCrossPath = {
                    setReversed(true)
                    splineToConstantHeading(Vector2d(10.0, -33.0), Math.toRadians(180.0))

                    splineToConstantHeading(Vector2d(-33.0, -33.0), Math.toRadians(180.0))
                }
            )
        }
    }.build()

    private fun TrajectorySequenceBuilder.path(
        cycles: Int = 1,
        backdropPixelScoreY: Double = -34.8,
        spikeMarkPixelScorePose: Pose2d,
        firstTrussCrossPath: TrajectorySequenceBuilder.() -> Unit
    ) {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + IntakeArmWristPositionCmd(0.42).endRightAway()
            + BoxLeftDoorCloseCmd()
        }

        UNSTABLE_addTemporalMarkerOffset(1.5) {
            + LiftGoToPositionCmd(200)
        }

        UNSTABLE_addTemporalMarkerOffset(2.5) {
            + BoxArmMiddleCmd()
        }

        splineToSplineHeading(
            Pose2d(51.2, backdropPixelScoreY, Math.toRadians(180.0)),
            Math.toRadians(0.0)
        )

        UNSTABLE_addTemporalMarkerOffset(0.5) {
            + BoxDoorsOpenCmd()
        }
        UNSTABLE_addTemporalMarkerOffset(1.2) {
            + BoxDoorsCloseCmd()
        }

        UNSTABLE_addTemporalMarkerOffset(2.0) {
            + LiftGoToPositionCmd(0)
            + BoxArmDownCmd()
        }
        waitSeconds(1.5)

        repeat(cycles) {
            // cycle
            if (it == 0) {
                UNSTABLE_addTemporalMarkerOffset(1.0) {
                    + IntakeArmGoToPositionCmd(-230)
                }

                lineToSplineHeading(spikeMarkPixelScorePose)

                UNSTABLE_addTemporalMarkerOffset(0.0) {
                    + IntakeArmWristPositionCmd(0.6).endRightAway()
                    + IntakeReleaseCmd()
                }

                UNSTABLE_addTemporalMarkerOffset(1.0) {
                    + IntakeStopCmd()
                    + IntakeArmGoToPositionCmd(0)
                }

                waitSeconds(1.5)

                firstTrussCrossPath()
                splineToSplineHeading(
                    Pose2d(-56.0, -11.5, Math.toRadians(180.0)),
                    Math.toRadians(180.0)
                )
            } else {
                lineToConstantHeading(Vector2d(-32.5, -26.0))
                splineToConstantHeading(Vector2d(-56.0, -11.5), Math.toRadians(180.0))
            }

            waitSeconds(2.0)

            lineToConstantHeading(Vector2d(-22.3, -6.1))
            splineToConstantHeading(Vector2d(50.2, -34.8), Math.toRadians(270.0))
            waitSeconds(2.0)
        }
    }

}