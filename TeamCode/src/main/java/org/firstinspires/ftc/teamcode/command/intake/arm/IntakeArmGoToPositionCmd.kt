package org.firstinspires.ftc.teamcode.command.intake.arm

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.github.serivesmejia.deltacommander.DeltaCommand
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

open class IntakeArmGoToPositionCmd(val position: Int) : DeltaCommand() {

    val sub = require<IntakeArmSubsystem>()

    private var t = 0.0

    private var start: Double? = null

    override fun init() {
    }

    override fun run() {
        if(start == null) {
            start = sub.armMotor.currentPosition.toDouble();
        }

        t += 1.0
        t = Range.clip(t, 0.0, 60.0)

        sub.controller.targetPosition = lerp(start!!, position.toDouble(), t / 60)

        sub.updateController()
    }

    override fun end(interrupted: Boolean) {
        sub.armMotor.power = 0.0
    }

    fun lerp(start: Double, end: Double, t: Double) = start + t * (end - start)
}