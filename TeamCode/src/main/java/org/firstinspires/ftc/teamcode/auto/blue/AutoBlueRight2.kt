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
                spikeMarkAlignPose = Pose2d(-33.5, 32.0, Math.toRadians(0.0)),
                armDownTimeOffset = 0.0,
                waitBeforeFirstDrive = 0.8,

                pixelSpikeLeavePose = Pose2d(-45.0, 5.0, Math.toRadians(0.0)),
                backdropScorePose = Pose2d(54.5, 39.0, Math.toRadians(180.0)),
                parkVector = Vector2d(53.0, 25.0)
            )

            Pattern.B -> path( // B
                spikeMarkAlignPose = Pose2d(-35.0, 9.0, Math.toRadians(90.0)),
                pixelSpikeLeavePose = Pose2d(-35.0, 6.0, Math.toRadians(91.0)),
                backdropScorePose = Pose2d(54.5, 30.0, Math.toRadians(180.0)),
                parkVector = Vector2d(54.0, 26.0)
            )

            Pattern.C -> path( // A
                spikeMarkAlignPose = Pose2d(-36.5, 16.0, Math.toRadians(50.0)),
                pixelSpikeLeavePose = Pose2d(-36.5, 10.0, Math.toRadians(20.0)),
                backdropScorePose = Pose2d(56.5, 24.0, Math.toRadians(180.0)),
                parkVector = Vector2d(54.0, 25.0)
            )
        }
    }.build()

    fun TrajectorySequenceBuilder.path(
            spikeMarkAlignPose: Pose2d,
            pixelSpikeLeavePose: Pose2d,
            backdropScorePose: Pose2d,
            parkVector: Vector2d,

            waitBeforeFirstDrive: Double? = null,
            armDownTimeOffset: Double = 2.0
    ) {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + IntakeArmWristPositionCmd(0.42).endRightAway()
        }

        UNSTABLE_addTemporalMarkerOffset(armDownTimeOffset) {
            + IntakeArmGoToPositionCmd(-230)
        }

        if(waitBeforeFirstDrive != null && waitBeforeFirstDrive > 0.0) {
            waitSeconds(waitBeforeFirstDrive)
        }

        lineToSplineHeading(spikeMarkAlignPose)

        UNSTABLE_addTemporalMarkerOffset(0.0) {
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
                - LiftDriveCmd { 1.0 }.async()
                - waitForSeconds(0.3)
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

        lineToSplineHeading(backdropScorePose)

        waitSeconds(6.0)

        lineTo(parkVector)
    }

}