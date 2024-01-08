package org.firstinspires.ftc.teamcode.command.lift

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.github.serivesmejia.deltacommander.DeltaCommand
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem

open class LiftGoToPositionCmd(val position: Int) : DeltaCommand() {

    val sub = require<LiftSubsystem>()

    override fun init() {
    }

    override fun run() {
        sub.controller.targetPosition = position.toDouble()

        sub.updateController()
    }

    override fun end(interrupted: Boolean) {
        sub.power = 0.0
    }

}