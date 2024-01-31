package org.firstinspires.ftc.teamcode.auto.red

import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.github.serivesmejia.deltacommander.dsl.deltaSequence
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
import org.firstinspires.ftc.teamcode.command.box.BoxRightDoorCloseCmd
import org.firstinspires.ftc.teamcode.command.intake.IntakeDoorCloseCmd
import org.firstinspires.ftc.teamcode.command.intake.IntakeDoorOpenCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmWristPositionCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmWristTiltInCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmWristTiltOutCmd
import org.firstinspires.ftc.teamcode.command.lift.LiftGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeAbsorbCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeReleaseCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeStopCmd
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.vision.Pattern
import org.firstinspires.ftc.teamcode.vision.Pattern.*
import java.util.Vector

@Autonomous(name = "R - Derecha Completo", group ="###RFINAL")
class AutoRedRightCompleto : PhobosAuto(Alliance.RED) {

    override val startPose = Pose2d(10.0, -59.0, Math.toRadians(90.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {
        when(pattern) {
            A -> path( // B
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(33.0, -28.0, Math.toRadians(180.0)),
                firstTrussCrossPath = {
                    lineToConstantHeading(Vector2d(35.0, -48.0))
                    lineToConstantHeading(Vector2d(8.0, -34.0))
                    lineToConstantHeading(Vector2d(-22.0, -33.0))
                }
            )
            C -> path( // B
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(33.0, -28.0, Math.toRadians(180.0)),
                firstTrussCrossPath = {
                    lineToConstantHeading(Vector2d(35.0, -48.0))
                    lineToConstantHeading(Vector2d(8.0, -35.0))
                    lineToConstantHeading(Vector2d(-22.0, -33.0))
                }
            )
            else -> path( // B
                backdropPixelScoreY = -34.8,
                spikeMarkPixelScorePose = Pose2d(33.0, -28.0, Math.toRadians(180.0)),
                firstTrussCrossPath = {
                    lineToConstantHeading(Vector2d(35.0, -48.0))
                    lineToConstantHeading(Vector2d(8.0, -35.0))
                    lineToConstantHeading(Vector2d(-22.0, -33.0))
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
            + IntakeArmWristTiltInCmd().endRightAway()
            + IntakeDoorCloseCmd()
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
            if (it == 0) { // spike mark pixel
                UNSTABLE_addTemporalMarkerOffset(1.0) {
                    + IntakeArmGoToPositionCmd(-230)
                }

                lineToSplineHeading(spikeMarkPixelScorePose)

                UNSTABLE_addTemporalMarkerOffset(0.2) {
                    + IntakeArmWristTiltOutCmd().endRightAway()
                    + IntakeReleaseCmd()
                }

                UNSTABLE_addTemporalMarkerOffset(2.3) {
                    + IntakeStopCmd()
                    + IntakeArmGoToPositionCmd(0, IntakeArmWristTiltInCmd())
                }

                waitSeconds(1.9) // FIRST STACK GRAB

                firstTrussCrossPath()

                UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    + IntakeArmGoToPositionCmd(-325, IntakeArmWristPositionCmd(0.56))
                }

                splineToConstantHeading( // proceed to grab
                    Vector2d(-60.0, -13.5),
                    Math.toRadians(180.0)
                )

                UNSTABLE_addTemporalMarkerOffset(0.8) {
                    + IntakeArmGoToPositionCmd(-270, IntakeArmWristPositionCmd(0.56))
                }
                lineToLinearHeading(Pose2d(-50.0, -11.5, Math.toRadians(115.0)), SampleMecanumDrive.getVelocityConstraint(
                        DriveConstants.MAX_VEL * 0.3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.6))

                UNSTABLE_addTemporalMarkerOffset(0.0) {
                    + IntakeAbsorbCmd()
                    + IntakeArmGoToPositionCmd(-340, IntakeArmWristPositionCmd(0.56))
                }

                lineToLinearHeading(Pose2d(-57.8, -11.5, Math.toRadians(180.0)), SampleMecanumDrive.getVelocityConstraint(
                        DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.6))

                lineToLinearHeading(Pose2d(-59.0, -11.5, Math.toRadians(180.0)), SampleMecanumDrive.getVelocityConstraint(
                    DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.6))
            } else {
                // TODO: Intake grab
                lineToConstantHeading(Vector2d(-32.5, -26.0)) // align to truss
                splineToConstantHeading(Vector2d(-52.8, -9.5), Math.toRadians(180.0)) // proceed to grab
            }

            UNSTABLE_addTemporalMarkerOffset(1.4) { // Prepare for passthrough
                + IntakeReleaseCmd()
                + IntakeArmGoToPositionCmd(0, IntakeArmWristTiltInCmd())
            }

            waitSeconds(1.4)

            UNSTABLE_addTemporalMarkerOffset(1.2) { // Pixel passthrough
                + IntakeDoorOpenCmd()
                + IntakeStopCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(1.6) {
                + IntakeDoorCloseCmd()
            }

            lineToConstantHeading(Vector2d(-22.3, -6.1)) // Align to cross stage door

            UNSTABLE_addTemporalMarkerOffset(0.1) {
                + LiftGoToPositionCmd(220)
            }
            UNSTABLE_addTemporalMarkerOffset(1.3) {
                + BoxArmMiddleCmd()
            }
            splineToConstantHeading(Vector2d(50.0, -35.2), Math.toRadians(270.0)) // Align to backdrop

            UNSTABLE_addTemporalMarkerOffset(1.0) {
                + BoxDoorsOpenCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(2.5) {
                + BoxDoorsCloseCmd()
                + BoxArmDownCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(2.8) {
                + LiftGoToPositionCmd(0)
            }
            waitSeconds(2.9)
        }
    }

}