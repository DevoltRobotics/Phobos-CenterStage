package org.firstinspires.ftc.teamcode.subsystem

import com.github.serivesmejia.deltacommander.DeltaSubsystem
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.command.box.BoxArmDownCmd
import org.firstinspires.ftc.teamcode.command.box.BoxDoorsCloseCmd

class BoxSubsystem(val leftDoorServo: Servo, val rightDoorServo: Servo) : DeltaSubsystem() {

    init {
        defaultCommand = BoxDoorsCloseCmd()

        maxRunningCommands = 2
    }

    override fun loop() {
    }

    companion object {
        const val leftOpenPos = 0.7
        const val leftClosePos = 0.0

        const val rightOpenPos = 0.0
        const val rightClosePos = 0.7
    }

}

class BoxArmSubsystem(val leftArmServo: Servo, val rightArmServo: Servo) : DeltaSubsystem() {

    var position = 0.0

    init {
        defaultCommand = BoxArmDownCmd()
    }

    override fun loop() {
        position = Range.clip(position, 0.0, 1.0);

        leftArmServo.position = (1.0 - (position - 0.1))
        rightArmServo.position = position
    }

}