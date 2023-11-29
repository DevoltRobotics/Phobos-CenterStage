package org.firstinspires.ftc.teamcode.auto.red

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.github.serivesmejia.deltacommander.dsl.deltaSequence
import com.github.serivesmejia.deltacommander.endRightAway
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmDriveCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmWristPositionCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeReleaseCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeStopCmd
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.Pattern

@Autonomous(name = "R - Izquierda 2 PX", group ="###AFINAL")
class AutoRedLeft2 : PhobosAuto(Alliance.RED) {

    override val startPose = Pose2d(-36.0, -59.0, Math.toRadians(90.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder (startPose).apply {
        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + deltaSequence {
                - IntakeArmWristPositionCmd(0.42).endRightAway()

                - IntakeArmDriveCmd({ -1.0 }, { 0.0 }).async()
                - waitForSeconds(4.0)
                - IntakeArmDriveCmd({ 0.0 }, { 0.0 }).async()
            }
        }

        lineToSplineHeading(Pose2d(-35.0, -7.0, Math.toRadians(270.0)))

        UNSTABLE_addTemporalMarkerOffset(0.0) {
            + IntakeArmWristPositionCmd(0.55).endRightAway()
            + IntakeReleaseCmd()
        }
        UNSTABLE_addTemporalMarkerOffset(2.0) {
            + IntakeStopCmd()

            + deltaSequence {
                - IntakeArmDriveCmd({ 1.0 }, { 0.0 }).async()
                - waitForSeconds(4.3)
                - IntakeArmDriveCmd({ 0.0 }, { 0.0 }).async()
            }
        }
        waitSeconds(3.0)

        lineTo(Vector2d(20.0, -7.0))

        lineToSplineHeading(Pose2d(55.0, -29.0, Math.toRadians(180.0)))
    }.build()

}