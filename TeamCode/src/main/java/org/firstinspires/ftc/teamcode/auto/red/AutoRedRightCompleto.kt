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
import org.firstinspires.ftc.teamcode.vision.RedCompletoTeamElementDetectionPipeline
import java.util.Vector

@Autonomous(name = "R - Derecha Completo", group ="###RFINAL")
class AutoRedRightCompleto : PhobosAuto(Alliance.RED, pipeline = RedCompletoTeamElementDetectionPipeline()) {

    override val startPose = Pose2d(10.0, -59.0, Math.toRadians(90.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {
        setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 1.2, DriveConstants.MAX_ANG_VEL * 1.1, DriveConstants.TRACK_WIDTH))
        setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 1.2))

        when(pattern) {
            A -> path( // A
                backdropPixelScoreY = -26.8,
                spikeMarkPixelScorePose = Pose2d(14.5, -34.0, Math.toRadians(180.0)),
                preSpikeMarkPath = {
                    lineToConstantHeading(Vector2d(16.0, -33.0))
                },
                firstTrussCrossPath = {
                    lineToConstantHeading(Vector2d(35.0, -48.0))
                    lineToConstantHeading(Vector2d(8.0, -33.0))
                    lineToConstantHeading(Vector2d(-26.0, -31.0))
                }
            )
            C -> path( // C
                backdropPixelScoreY = -41.8,
                spikeMarkPixelScorePose = Pose2d(32.5, -29.0, Math.toRadians(180.0)),
                firstTrussCrossPath = {
                    lineToConstantHeading(Vector2d(35.0, -48.0))
                    lineToConstantHeading(Vector2d(8.0, -33.0))
                    lineToConstantHeading(Vector2d(-26.0, -31.0))
                }
            )
            else -> path( // B
                    backdropPixelScoreY = -32.8,
                    spikeMarkPixelScorePose = Pose2d(28.0, -22.0, Math.toRadians(180.0)),
                    firstTrussCrossPath = {
                        lineToConstantHeading(Vector2d(35.0, -48.0))
                        lineToConstantHeading(Vector2d(8.0, -33.0))
                        lineToConstantHeading(Vector2d(-26.0, -31.0))
                    }
            )
        }
    }.build()

    private fun TrajectorySequenceBuilder.path(
        cycles: Int = 1,
        backdropPixelScoreY: Double = -34.8,
        spikeMarkPixelScorePose: Pose2d,
        preSpikeMarkPath: (TrajectorySequenceBuilder.() -> Unit)? = null,
        firstTrussCrossPath: TrajectorySequenceBuilder.() -> Unit,
    ) {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + IntakeArmWristTiltInCmd().endRightAway()
            + IntakeDoorCloseCmd()
        }

        UNSTABLE_addTemporalMarkerOffset(1.5) {
            + LiftGoToPositionCmd(150)
        }

        UNSTABLE_addTemporalMarkerOffset(2.5) {
            + BoxArmMiddleCmd()
        }

        splineToSplineHeading(
            Pose2d(51.9, backdropPixelScoreY, Math.toRadians(180.0)),
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
                    + IntakeArmGoToPositionCmd(-190)
                }

                lineToSplineHeading(spikeMarkPixelScorePose)

                UNSTABLE_addTemporalMarkerOffset(0.2) {
                    + IntakeArmWristPositionCmd(0.59).endRightAway()
                    + IntakeReleaseCmd()
                }

                preSpikeMarkPath?.invoke(this@path)

                UNSTABLE_addTemporalMarkerOffset(2.3) {
                    + IntakeStopCmd()
                    + IntakeArmGoToPositionCmd(0, IntakeArmWristTiltInCmd())
                }

                waitSeconds(1.6) // FIRST STACK GRAB

                firstTrussCrossPath()

                UNSTABLE_addTemporalMarkerOffset(0.1) {
                    + IntakeArmGoToPositionCmd(-180, IntakeArmWristPositionCmd(0.42))
                }

                splineToConstantHeading( // proceed to grab
                    Vector2d(-50.0, -9.5),
                    Math.toRadians(180.0)
                )

                waitSeconds(0.2)

                UNSTABLE_addTemporalMarkerOffset(-0.2) {
                    + IntakeAbsorbCmd()
                    + IntakeArmGoToPositionCmd(-310, IntakeArmWristPositionCmd(0.42))
                }
                lineToLinearHeading(Pose2d(-57.5, -9.5, Math.toRadians(180.0)), SampleMecanumDrive.getVelocityConstraint(
                        DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.6))

                UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    + IntakeArmGoToPositionCmd(-330, IntakeArmWristPositionCmd(0.45))
                }
                lineToLinearHeading(Pose2d(-58.0, -6.5, Math.toRadians(155.0)), SampleMecanumDrive.getVelocityConstraint(
                    DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.6))

                UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    + IntakeArmGoToPositionCmd(-350, IntakeArmWristPositionCmd(0.45))
                }
                lineToLinearHeading(Pose2d(-58.2, -5.5, Math.toRadians(180.0)), SampleMecanumDrive.getVelocityConstraint(
                    DriveConstants.MAX_VEL * 0.4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.6))
            } else {
                // TODO: Intake grab
                lineToConstantHeading(Vector2d(-32.5, -26.0)) // align to truss
                splineToConstantHeading(Vector2d(-52.8, -9.5), Math.toRadians(180.0)) // proceed to grab
            }

            UNSTABLE_addTemporalMarkerOffset(1.0) {
                + IntakeReleaseCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(1.4) { // Prepare for passthrough
                + IntakeArmGoToPositionCmd(0, IntakeArmWristTiltInCmd())
            }

            waitSeconds(1.4)

            UNSTABLE_addTemporalMarkerOffset(1.2) { // Pixel passthrough
                + IntakeDoorOpenCmd()
                + IntakeStopCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(1.9) {
                + IntakeDoorCloseCmd()
            }

            lineToSplineHeading(Pose2d(-22.3, -6.1, Math.toRadians(180.0))) // Align to cross stage door

            UNSTABLE_addTemporalMarkerOffset(0.1) {
                + LiftGoToPositionCmd(310)
            }
            UNSTABLE_addTemporalMarkerOffset(1.5) {
                + BoxArmMiddleCmd()
            }
            splineToConstantHeading(Vector2d(51.3, -35.2), Math.toRadians(270.0)) // Align to backdrop

            UNSTABLE_addTemporalMarkerOffset(0.5) {
                + BoxDoorsOpenCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(1.5) {
                + BoxDoorsCloseCmd()
                + BoxArmDownCmd()
            }
            UNSTABLE_addTemporalMarkerOffset(2.0) {
                + LiftGoToPositionCmd(0)
            }
            waitSeconds(2.9)
        }
    }

}