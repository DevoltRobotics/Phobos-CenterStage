package org.firstinspires.ftc.teamcode.command.box

import com.github.serivesmejia.deltacommander.DeltaCommand
import org.firstinspires.ftc.teamcode.subsystem.BoxArmSubsystem

class BoxArmDriveCmd(val power: () -> Double) : DeltaCommand() {

    val sub = require<BoxArmSubsystem>()

    override fun run() {
        sub.position += power() * 0.04
    }

}