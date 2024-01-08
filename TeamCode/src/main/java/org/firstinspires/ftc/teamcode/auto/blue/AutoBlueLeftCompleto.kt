package org.firstinspires.ftc.teamcode.auto.blue

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.vision.Pattern

@Autonomous(name = "A - Izquierda Completo", group ="###AFINAL")
class AutoBlueLeftCompleto : PhobosAuto(Alliance.BLUE){

    override val startPose = Pose2d(11.0, 59.0, Math.toRadians(90.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {

    }.build()

}