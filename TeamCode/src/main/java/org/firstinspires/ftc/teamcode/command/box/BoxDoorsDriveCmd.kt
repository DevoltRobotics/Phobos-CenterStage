package org.firstinspires.ftc.teamcode.command.box

import com.github.serivesmejia.deltacommander.DeltaCommand
import org.firstinspires.ftc.teamcode.subsystem.BoxSubsystem

class BoxDoorsDriveCmd(val left: () -> Boolean, val right: () -> Boolean) : DeltaCommand() {

    val sub = require<BoxSubsystem>()

    override fun run() {
        if(left()) {
            sub.leftDoorServo.position = BoxSubsystem.leftOpenPos
        } else {
            sub.leftDoorServo.position = BoxSubsystem.leftClosePos
        }

        if(right()) {
            sub.rightDoorServo.position = BoxSubsystem.rightOpenPos
        } else {
            sub.rightDoorServo.position = BoxSubsystem.rightClosePos
        }
    }

}