package org.firstinspires.ftc.teamcode.command.intake.arm

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.github.serivesmejia.deltacommander.DeltaCommand
import com.qualcomm.robotcore.util.ElapsedTime

open class IntakeArmGoToPositionCmd(val position: Int) : DeltaCommand() {

    val sub = require<IntakeArmSubsystem>()

    val profile = generateProfile()
    val timer = ElapsedTime()

    override fun init() {
        timer.reset()
    }

    override fun run() {
        val t = timer.seconds()

        val state = profile[t]

        sub.controller.targetPosition = state.x
        sub.controller.targetVelocity = state.v
        sub.controller.targetAcceleration = state.a

        sub.updateController()
    }

    override fun end(interrupted: Boolean) {
        sub.armMotor.power = 0.0
    }

    private fun generateProfile() = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(sub.armMotor.currentPosition.toDouble(), 0.0, 0.0),
            MotionState(position.toDouble(), 0.0, 0.0),
            IntakeArmSubsystem.ticksPerSecond.toDouble(),
            IntakeArmSubsystem.ticksPerPerSecond.toDouble()
    )
}