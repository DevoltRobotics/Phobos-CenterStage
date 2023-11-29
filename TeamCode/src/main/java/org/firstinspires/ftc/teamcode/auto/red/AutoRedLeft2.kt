package org.firstinspires.ftc.teamcode.auto.red

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.Pattern

@Autonomous(name = "R - Izquierda 2 PX", group ="###AFINAL")
class AutoRedLeft2 : PhobosAuto(Alliance.RED) {

    override val startPose = Pose2d()

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {

    }.build()

}