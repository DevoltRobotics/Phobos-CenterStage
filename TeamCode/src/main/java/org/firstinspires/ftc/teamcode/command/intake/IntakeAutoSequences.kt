package org.firstinspires.ftc.teamcode.command.intake

import com.github.serivesmejia.deltacommander.dsl.deltaSequence
import com.github.serivesmejia.deltacommander.subsystem
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmGoToPositionCmd

fun intakeDepositIntoBoxSequence() = deltaSequence {
    - IntakeArmGoToPositionCmd(0).waitUntil { sub.armMotor.currentPosition >= -50 }

    - IntakeDoorOpenCmd().async()
    - waitForSeconds(2.0)
    - IntakeDoorCloseCmd().async()
}