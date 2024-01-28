package org.firstinspires.ftc.teamcode.command.intake.arm

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.github.serivesmejia.deltacommander.DeltaCommand
import com.github.serivesmejia.deltacommander.subsystem
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

open class IntakeArmWristPositionCmd(val position: Double) : DeltaCommand() {

    val sub = subsystem<IntakeArmSubsystem>()

    override fun run() {
        sub.wristServo.position = position
    }

    override fun end(interrupted: Boolean) {
    }

}

class IntakeArmWristTiltInCmd : IntakeArmWristPositionCmd(0.42)
class IntakeArmWristTiltOutCmd : IntakeArmWristPositionCmd(0.6)