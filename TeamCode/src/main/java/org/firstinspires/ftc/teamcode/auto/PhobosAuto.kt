package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.PhobosOpMode
import org.firstinspires.ftc.teamcode.command.mecanum.TrajectorySequenceCmd
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.Pattern

abstract class PhobosAuto(val alliance: Alliance) : PhobosOpMode() {

    val drive get() = hardware.drive
    open val startPose = Pose2d()

    override fun setup() {
    }

    override fun begin() {
        drive.poseEstimate = startPose

        // + TrajectorySequenceCmd(sequence()) // TODO: vision
    }

    abstract fun sequence(pattern: Pattern) : TrajectorySequence
}