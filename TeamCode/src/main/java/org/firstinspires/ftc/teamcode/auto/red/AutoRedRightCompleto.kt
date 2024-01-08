package org.firstinspires.ftc.teamcode.auto.red

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.Pattern

@Autonomous(name = "R - Derecha Completo", group ="###RFINAL")
class AutoRedRightCompleto : PhobosAuto(Alliance.RED) {

    override val startPose = Pose2d(11.0, -59.0, Math.toRadians(270.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {

    }.build()

}