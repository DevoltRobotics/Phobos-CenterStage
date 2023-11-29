package org.firstinspires.ftc.teamcode.command.intake.arm

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.github.serivesmejia.deltacommander.DeltaCommand
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

open class IntakeArmGoToPositionCmd(val position: Int) : DeltaCommand() {

    val sub = require<IntakeArmSubsystem>()

    val timer = ElapsedTime()

    private var start = 0

    override fun init() {
        timer.reset()
        start = sub.armMotor.currentPosition
    }

    override fun run() {
        val t = timer.seconds()

        sub.controller.targetPosition = Range.clip(lerp(start.toDouble(), position.toDouble(), t), start.toDouble(), position.toDouble())

        sub.updateController()
    }

    override fun end(interrupted: Boolean) {
        sub.armMotor.power = 0.0
    }

    fun lerp(start: Double, end: Double, t: Double) = start + t * (end - start)
}