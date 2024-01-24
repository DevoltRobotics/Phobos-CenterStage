package org.firstinspires.ftc.teamcode.auto.red

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.auto.PhobosAuto
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.vision.Pattern

@Autonomous(name = "R - Derecha Completo", group ="###RFINAL")
class AutoRedRightCompleto : PhobosAuto(Alliance.RED) {

    override val startPose = Pose2d(10.0, -59.0, Math.toRadians(90.0))

    override fun sequence(pattern: Pattern) = drive.trajectorySequenceBuilder(startPose).apply {
        splineToSplineHeading(Pose2d(50.2, -34.8, Math.toRadians(180.0)), Math.toRadians(0.0))

        repeat(2) {
            waitSeconds(2.0)

            // cycle
            lineToConstantHeading(Vector2d(-32.5, -32.0))
            splineToConstantHeading(Vector2d(-56.0, -11.5), Math.toRadians(180.0))

            waitSeconds(2.0)

            lineToConstantHeading(Vector2d(-7.3, -7.8))
            splineToConstantHeading(Vector2d(50.2, -34.8), Math.toRadians(270.0))
        }
    }.build()

    fun TrajectorySequenceBuilder.path() {
        splineToSplineHeading(Pose2d(47.2, -34.8, Math.toRadians(180.0)), Math.toRadians(0.0))

        waitSeconds(3.0)

        // cycle
        splineToConstantHeading(Vector2d(30.5, -36.2), Math.toRadians(180.0))

        waitSeconds(2.0)

        splineToConstantHeading(Vector2d(11.0, -58.0), Math.toRadians(180.0))

        splineToSplineHeading(Pose2d(-34.0, -58.8, Math.toRadians(180.0)), Math.toRadians(180.0)) // grab
        splineToConstantHeading(Vector2d(-56.0, -11.5), Math.toRadians(180.0))

        waitSeconds(2.0)

        lineToConstantHeading(Vector2d(-7.3, -11.5))

        splineToConstantHeading(Vector2d(47.2, -34.8), Math.toRadians(270.0))
    }

}