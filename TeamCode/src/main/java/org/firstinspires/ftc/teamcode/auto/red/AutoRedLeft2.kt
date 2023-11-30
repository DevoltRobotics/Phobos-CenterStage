package org.firstinspires.ftc.teamcode.auto.red

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.github.serivesmejia.deltacommander.command.DeltaRunCmd
import com.github.serivesmejia.deltacommander.dsl.deltaSequence
import com.github.serivesmejia.deltacommander.endRightAway
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.command.box.BoxArmDownCmd
import org.firstinspires.ftc.teamcode.command.box.BoxArmPositionCmd
import org.firstinspires.ftc.teamcode.command.box.BoxDoorsCloseCmd
import org.firstinspires.ftc.teamcode.command.box.BoxDoorsOpenCmd
import org.firstinspires.ftc.teamcode.command.box.BoxLeftDoorCloseCmd
import org.firstinspires.ftc.teamcode.command.box.BoxLeftDoorOpenCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmDriveCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmWristPositionCmd
import org.firstinspires.ftc.teamcode.command.lift.LiftDriveCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeReleaseCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeStopCmd
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.vision.Pattern

@Autonomous(name = "R - Izquierda 2 PX", group ="###AFINAL")
class AutoRedLeft2 : PhobosAuto(Alliance.RED) {

    override val startPose = Pose2d(-36.0, -59.0, Math.toRadians(90.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {
        patternA()
    }.build()

    fun TrajectorySequenceBuilder.patternA() {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            +IntakeArmWristPositionCmd(0.43).endRightAway()
        }

        UNSTABLE_addTemporalMarkerOffset(2.0) {
            +IntakeArmGoToPositionCmd(-250)
        }

        lineToSplineHeading(Pose2d(-36.5, -16.0, Math.toRadians(235.0)))

        UNSTABLE_addTemporalMarkerOffset(0.0) {
            +IntakeArmWristPositionCmd(0.59).endRightAway()
            +IntakeReleaseCmd()
        }

        UNSTABLE_addTemporalMarkerOffset(1.0) {
            +IntakeStopCmd()
            +IntakeArmGoToPositionCmd(0)
        }
        waitSeconds(0.5)
        lineToLinearHeading(Pose2d(-36.5, -10.0, Math.toRadians(200.0)))
        waitSeconds(2.0)

        lineToLinearHeading(Pose2d(20.0, -7.0, Math.toRadians(180.0)))

        UNSTABLE_addTemporalMarkerOffset(2.0) {
            + deltaSequence {
                - LiftDriveCmd { 1.0 }.async()

                - waitForSeconds(0.21)

                - LiftDriveCmd { 0.0 }.async()

                - waitForSeconds(1.0)

                - BoxArmPositionCmd(0.42).async()

                - waitForSeconds(1.0)

                - BoxLeftDoorOpenCmd().async()

                - waitForSeconds(2.0)

                - BoxLeftDoorCloseCmd().async()
                - BoxArmDownCmd().async()

                - LiftDriveCmd { -0.5 }.async()
                - waitForSeconds(5.0)
                - LiftDriveCmd { 0.0 }.async()
            }
        }

        lineToSplineHeading(Pose2d(56.5, -25.0, Math.toRadians(180.0)))

        waitSeconds(6.0)

        lineTo(Vector2d(54.0, -25.0))
    }

    fun TrajectorySequenceBuilder.patternB() {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            +IntakeArmWristPositionCmd(0.43).endRightAway()
        }
        UNSTABLE_addTemporalMarkerOffset(2.0) {
            +IntakeArmGoToPositionCmd(-250)
        }

        lineToSplineHeading(Pose2d(-35.0, -9.0, Math.toRadians(270.0)))

        UNSTABLE_addTemporalMarkerOffset(0.0) {
            +IntakeArmWristPositionCmd(0.59).endRightAway()
            +IntakeReleaseCmd()
        }

        UNSTABLE_addTemporalMarkerOffset(1.0) {
            +IntakeStopCmd()
            +IntakeArmGoToPositionCmd(0)
        }
        waitSeconds(0.5)
        lineTo(Vector2d(-35.0, -8.0))
        waitSeconds(2.0)

        lineTo(Vector2d(20.0, -7.0))

        UNSTABLE_addTemporalMarkerOffset(2.0) {
            + deltaSequence {
                - LiftDriveCmd { 1.0 }.async()

                - waitForSeconds(0.21)

                - LiftDriveCmd { 0.0 }.async()

                - waitForSeconds(1.0)

                - BoxArmPositionCmd(0.42).async()

                - waitForSeconds(1.0)

                - BoxLeftDoorOpenCmd().async()

                - waitForSeconds(2.0)

                - BoxLeftDoorCloseCmd().async()
                - BoxArmDownCmd().async()

                - LiftDriveCmd { -0.5 }.async()
                - waitForSeconds(5.0)
                - LiftDriveCmd { 0.0 }.async()
            }
        }

        lineToSplineHeading(Pose2d(56.5, -31.0, Math.toRadians(180.0)))

        waitSeconds(6.0)

        lineTo(Vector2d(54.0, -26.0))
    }

}