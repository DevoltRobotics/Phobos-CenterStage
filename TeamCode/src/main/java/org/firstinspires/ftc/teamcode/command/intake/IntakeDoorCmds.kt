package org.firstinspires.ftc.teamcode.command.intake

import com.github.serivesmejia.deltacommander.DeltaCommand
import org.firstinspires.ftc.teamcode.subsystem.IntakeDoorSubsystem

class IntakeDoorOpenCmd : DeltaCommand() {
    val sub = require<IntakeDoorSubsystem>()

    override fun run() {
        sub.doorServo.position = 0.5
    }
}

class IntakeDoorCloseCmd : DeltaCommand() {
    val sub = require<IntakeDoorSubsystem>()

    override fun run() {
        sub.doorServo.position = 1.0
    }
}