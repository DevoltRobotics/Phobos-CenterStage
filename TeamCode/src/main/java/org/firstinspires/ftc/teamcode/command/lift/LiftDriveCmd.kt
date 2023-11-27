package org.firstinspires.ftc.teamcode.command.lift

import com.github.serivesmejia.deltacommander.DeltaCommand
import org.firstinspires.ftc.teamcode.subsystem.BoxArmSubsystem
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem

class LiftDriveCmd(val power: () -> Double) : DeltaCommand() {

    val sub = require<LiftSubsystem>()

    override fun run() {
        sub.power = power()
    }

}