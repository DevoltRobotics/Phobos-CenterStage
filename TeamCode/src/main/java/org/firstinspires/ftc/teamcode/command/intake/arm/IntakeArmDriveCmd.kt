package org.firstinspires.ftc.teamcode.command.intake.arm

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.github.serivesmejia.deltacommander.DeltaCommand
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

open class IntakeArmDriveCmd(val power: () -> Double, val wristPower: () -> Double) : DeltaCommand() {

    val sub = require<IntakeArmSubsystem>()

    val deltaTimer = ElapsedTime()

    val positionUpdateTimer = ElapsedTime()

    override fun run() {
        sub.controller.targetPosition += (power() * deltaTimer.seconds() * IntakeArmSubsystem.drivingTicksPerSecond)

        sub.downWristPosition = 0.5

        sub.controller.targetPosition = Range.clip(sub.controller.targetPosition, -500.0, 50.0)

        sub.updateController()

        if(positionUpdateTimer.seconds() >= 5.0) {
            sub.controller.targetPosition = sub.armMotor.currentPosition.toDouble()
            positionUpdateTimer.reset()
        }

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