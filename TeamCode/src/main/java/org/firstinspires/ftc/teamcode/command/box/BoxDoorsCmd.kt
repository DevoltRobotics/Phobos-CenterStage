package org.firstinspires.ftc.teamcode.command.box

import com.github.serivesmejia.deltacommander.DeltaCommand
import org.firstinspires.ftc.teamcode.subsystem.BoxSubsystem

open class BoxLeftDoorPositionCmd(val position: Double) : DeltaCommand() {

    val sub = require<BoxSubsystem>()

    override fun run() {
        sub.leftDoorServo.position = position
    }
}

open class BoxRightDoorPositionCmd(val position: Double) : DeltaCommand() {

    val sub = require<BoxSubsystem>()

    override fun run() {
        sub.rightDoorServo.position = position
    }
}


open class BoxDoorsPositionCmd(val left: Double, val right: Double) : DeltaCommand() {

    val sub = require<BoxSubsystem>()

    override fun run() {
        sub.leftDoorServo.position = left
        sub.rightDoorServo.position = right
    }
}

class BoxRightDoorOpenCmd : BoxRightDoorPositionCmd(BoxSubsystem.rightOpenPos)
class BoxLeftDoorOpenCmd : BoxLeftDoorPositionCmd(BoxSubsystem.leftOpenPos)

class BoxRightDoorCloseCmd : BoxRightDoorPositionCmd(BoxSubsystem.rightClosePos)
class BoxLeftDoorCloseCmd : BoxLeftDoorPositionCmd(BoxSubsystem.leftClosePos)

class BoxDoorsOpenCmd : BoxDoorsPositionCmd(BoxSubsystem.leftOpenPos, BoxSubsystem.rightOpenPos)
class BoxDoorsCloseCmd : BoxDoorsPositionCmd(BoxSubsystem.leftClosePos, BoxSubsystem.rightClosePos)