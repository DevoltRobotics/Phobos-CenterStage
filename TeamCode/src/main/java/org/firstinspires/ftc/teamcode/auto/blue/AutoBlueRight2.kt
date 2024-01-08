package org.firstinspires.ftc.teamcode.auto.blue

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.github.serivesmejia.deltacommander.dsl.deltaSequence
import com.github.serivesmejia.deltacommander.endRightAway
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.command.box.BoxArmDownCmd
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

@Autonomous(name = "A - Derecha 2 PX", group ="###AFINAL")
class AutoBlueRight2 : PhobosAuto(Alliance.BLUE) {

    override val startPose = Pose2d(-36.0, 59.0, Math.toRadians(270.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {
        when(pattern) {
            Pattern.A -> path( // C
                spikeMarkAlignPose = Pose2d(-33.6, 33.0, Math.toRadians(0.0)),
                armDownTimeOffset = 0.0,
                waitBeforeFirstDrive = 0.8,

                pixelSpikeLeavePose = Pose2d(-45.0, 5.0, Math.toRadians(0.0)),
                backdropScorePose = Pose2d(55.0, 39.0, Math.toRadians(180.0)),
                parkVector = Vector2d(53.0, 25.0)
            )

            Pattern.B -> path( // B
                spikeMarkAlignPose = Pose2d(-35.0, 9.0, Math.toRadians(90.0)),
                pixelSpikeLeavePose = Pose2d(-35.0, 4.0, Math.toRadians(91.0)),
                backdropScorePose = Pose2d(54.7, 32.0, Math.toRadians(180.0)),
                parkVector = Vector2d(54.0, 26.0)
            )

            Pattern.C -> path( // A
                spikeMarkAlignPose = Pose2d(-34.0, 19.0, Math.toRadians(135.0)),
                pixelSpikeLeavePose = Pose2d(-35.0, 7.0, Math.toRadians(140.0)),
                backdropScorePose = Pose2d(56.5, 23.0, Math.toRadians(180.0)),
                parkVector = Vector2d(54.0, 23.0),

                armDownTicks = -225
            )
        }
    }.build()

    fun TrajectorySequenceBuilder.path(
            spikeMarkAlignPose: Pose2d,
            pixelSpikeLeavePose: Pose2d,
            backdropScorePose: Pose2d,
            parkVector: Vector2d,

            waitBeforeFirstDrive: Double? = null,
            armDownTimeOffset: Double = 2.0,
            armDownTicks: Int = -230
    ) {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + IntakeArmWristPositionCmd(0.42).endRightAway()
        }

        UNSTABLE_addTemporalMarkerOffset(armDownTimeOffset) {
            + IntakeArmGoToPositionCmd(armDownTicks)
        }

        if(waitBeforeFirstDrive != null && waitBeforeFirstDrive > 0.0) {
            waitSeconds(waitBeforeFirstDrive)
        }

        lineToSplineHeading(spikeMarkAlignPose)

        UNSTABLE_addTemporalMarkerOffset(-0.3) {
            + IntakeArmWristPositionCmd(0.59).endRightAway()
            + IntakeReleaseCmd()
        }

        UNSTABLE_addTemporalMarkerOffset(1.0) {
            + IntakeStopCmd()
            + IntakeArmGoToPositionCmd(0)
        }
        waitSeconds(0.8)
        lineToLinearHeading(pixelSpikeLeavePose)
        waitSeconds(2.0)

        lineToLinearHeading(Pose2d(20.0, 7.0, Math.toRadians(180.0)))

        UNSTABLE_addTemporalMarkerOffset(2.0) {
            + deltaSequence {
                - LiftGoToPositionCmd(280).async()
                - waitForSeconds(0.15)

                - waitForSeconds(1.0)

                - BoxArmPositionCmd(0.4).async()

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

        lineToSplineHeading(backdropScorePose)

        waitSeconds(6.0)

        lineTo(parkVector)
    }

}