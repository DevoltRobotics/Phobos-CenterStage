package org.firstinspires.ftc.teamcode.command.mecanum

import com.github.serivesmejia.deltacommander.DeltaCommand
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsystem.MecanumSubsystem

class TrajectorySequenceCmd(val sequence: TrajectorySequence) : DeltaCommand() {
    val sub = require<MecanumSubsystem>()

    override fun init() {
        sub.drive.followTrajectorySequenceAsync(sequence)
    }

    override fun run() {
        if(!sub.drive.isBusy) {
            requestEnd()
        }
    }
}