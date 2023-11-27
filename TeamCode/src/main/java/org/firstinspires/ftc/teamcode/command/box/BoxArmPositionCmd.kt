package org.firstinspires.ftc.teamcode.command.box

import com.github.serivesmejia.deltacommander.DeltaCommand
import org.firstinspires.ftc.teamcode.subsystem.BoxArmSubsystem

open class BoxArmPositionCmd(val position: Double) : DeltaCommand() {
    val sub = require<BoxArmSubsystem>()

    override fun run() {
        sub.position = position
    }
}

class BoxArmDownCmd : BoxArmPositionCmd(1.0)