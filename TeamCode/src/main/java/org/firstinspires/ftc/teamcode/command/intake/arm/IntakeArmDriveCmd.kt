package org.firstinspires.ftc.teamcode.command.intake.arm

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.github.serivesmejia.deltacommander.DeltaCommand
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

open class IntakeArmDriveCmd(val power: () -> Double, val wristPower: () -> Double) : DeltaCommand() {

    val sub = require<IntakeArmSubsystem>()

    val deltaTimer = ElapsedTime()

    override fun run() {
        sub.controller.targetPosition += (power() * deltaTimer.seconds() * IntakeArmSubsystem.ticksPerSecond)

        sub.controller.targetPosition = Range.clip(sub.controller.targetPosition, -400.0, 0.0)

        sub.updateController()

        deltaTimer.reset()
    }

    override fun end(interrupted: Boolean) {
        sub.armMotor.power = 0.0
    }

}

class IntakeArmStopCmd : IntakeArmDriveCmd({ 0.0 }, { 0.0 }) {
    override fun run() {
        sub.downWristPosition = 0.5

        super.run()
    }
}