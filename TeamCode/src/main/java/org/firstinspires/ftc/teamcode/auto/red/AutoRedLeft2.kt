package org.firstinspires.ftc.teamcode.auto.red

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
import org.firstinspires.ftc.teamcode.command.box.BoxLeftDoorCloseCmd
import org.firstinspires.ftc.teamcode.command.box.BoxLeftDoorOpenCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmWristPositionCmd
import org.firstinspires.ftc.teamcode.command.lift.LiftDriveCmd
import org.firstinspires.ftc.teamcode.command.lift.LiftGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeReleaseCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeStopCmd
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.vision.Pattern

@Autonomous(name = "R - Izquierda 2 PX", group ="###RFINAL")
class AutoRedLeft2 : PhobosAuto(Alliance.RED) {

    override val startPose = Pose2d(-36.0, -59.0, Math.toRadians(90.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {
        // waitSeconds(8.0) // Hypebirds momento

        when(pattern) {
            Pattern.A -> path( // A
                spikeMarkAlignPose = Pose2d(-36.5, -16.0, Math.toRadians(235.0)),
                pixelSpikeLeavePose = Pose2d(-36.5, -8.0, Math.toRadians(215.0)),
                backdropScorePose = Pose2d(57.0, -24.0, Math.toRadians(180.0)),
                parkVector = Vector2d(54.0, -23.0)
            )

            Pattern.B -> path( // B
                spikeMarkAlignPose = Pose2d(-35.0, -9.0, Math.toRadians(270.0)),
                pixelSpikeLeavePose = Pose2d(-35.0, -6.0, Math.toRadians(270.0)),
                backdropScorePose = Pose2d(57.0, -31.0, Math.toRadians(180.0)),
                parkVector = Vector2d(54.0, -23.0)
            )

            Pattern.C -> path( // C
                spikeMarkAlignPose = Pose2d(-34.5, -33.0, Math.toRadians(0.0)),
                armDownTimeOffset = 0.0,
                spikeMarkArmPos = -200,
                waitBeforeFirstDrive = 0.8,

                pixelSpikeLeavePose = Pose2d(-53.0, -8.0, Math.toRadians(0.0)),
                backdropScorePose = Pose2d(57.0 , -41.8, Math.toRadians(180.0)),
                parkVector = Vector2d(53.0, -23.0)
            )
        }
    }.build()

    fun TrajectorySequenceBuilder.path(
            spikeMarkAlignPose: Pose2d,
            pixelSpikeLeavePose: Pose2d,
            spikeMarkArmPos: Int = -230,
            backdropScorePose: Pose2d,
            parkVector: Vector2d,

            waitBeforeFirstDrive: Double? = null,
            armDownTimeOffset: Double = 2.0
    ) {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + IntakeArmWristPositionCmd(0.42).endRightAway()
        }

        UNSTABLE_addTemporalMarkerOffset(armDownTimeOffset) {
            + IntakeArmGoToPositionCmd(spikeMarkArmPos)
        }

        if(waitBeforeFirstDrive != null && waitBeforeFirstDrive > 0.0) {
            waitSeconds(waitBeforeFirstDrive)
        }

        lineToSplineHeading(spikeMarkAlignPose)

        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + IntakeArmWristPositionCmd(0.56).endRightAway()
            + IntakeReleaseCmd()
        }

        UNSTABLE_addTemporalMarkerOffset(1.0) {
            + IntakeStopCmd()
            + IntakeArmGoToPositionCmd(0)
        }

        waitSeconds(0.8)
        lineToLinearHeading(pixelSpikeLeavePose)
        waitSeconds(2.0)

        lineToLinearHeading(Pose2d(20.0, -7.0, Math.toRadians(180.0)))

        UNSTABLE_addTemporalMarkerOffset(2.0) {
            + deltaSequence {
                - LiftGoToPositionCmd(140).async()
                - waitForSeconds(0.15)

                - waitForSeconds(1.0)

                - BoxArmMiddleCmd().async()

                - waitForSeconds(1.0)

                - BoxLeftDoorOpenCmd().async()

                - waitForSeconds(1.5)

                - BoxLeftDoorCloseCmd().async()
                - BoxArmDownCmd().async()

                - waitForSeconds(1.0)

                - LiftDriveCmd { -0.5 }.async()
                - waitForSeconds(5.0)
                - LiftDriveCmd { 0.0 }.async()
            }
        }

        lineToSplineHeading(backdropScorePose)

        waitSeconds(6.0)

        lineTo(parkVector)
    }

}