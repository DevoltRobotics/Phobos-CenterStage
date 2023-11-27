package org.firstinspires.ftc.teamcode.command.mecanum

import com.github.serivesmejia.deltacommander.DeltaCommand
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

open class IntakeRunCmd(val power: Double) : DeltaCommand() {
    val sub = require<IntakeSubsystem>()

    override fun run() {
        sub.intakeMotor.power = power
    }

    override fun end(interrupted: Boolean) {
        sub.intakeMotor.power = 0.0
    }
}

class IntakeAbsorbCmd : IntakeRunCmd(1.0)
class IntakeReleaseCmd : IntakeRunCmd(-1.0)
class IntakeStopCmd() : IntakeRunCmd(0.0)