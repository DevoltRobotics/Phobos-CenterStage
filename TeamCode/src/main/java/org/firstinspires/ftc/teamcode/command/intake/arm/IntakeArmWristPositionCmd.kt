package org.firstinspires.ftc.teamcode.command.intake.arm

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.github.serivesmejia.deltacommander.DeltaCommand
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

open class IntakeArmWristPositionCmd(val position: Double) : DeltaCommand() {

    val sub = require<IntakeArmSubsystem>()

    override fun run() {
        sub.wristServo.position = position
    }

    override fun end(interrupted: Boolean) {
    }

}