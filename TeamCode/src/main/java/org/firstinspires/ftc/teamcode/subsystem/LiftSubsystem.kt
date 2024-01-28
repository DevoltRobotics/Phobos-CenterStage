package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.github.serivesmejia.deltacommander.DeltaSubsystem
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

class LiftSubsystem(val leftMotor: DcMotor, val rightMotor: DcMotor) : DeltaSubsystem() {

    var controller = createController()
        private set

    var power = 0.0

    init {
        rightMotor.direction = DcMotorSimple.Direction.REVERSE;
    }

    override fun loop() {
        leftMotor.power = power + 0.16
        rightMotor.power = power + 0.16
    }

    fun reset() {
        rightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun updateController() {
        power = controller.update(leftMotor.currentPosition.toDouble())
    }

    private fun createController() = PIDFController(Lift.pidfCoefficients)
}

@Config
object Lift {
    @JvmField var pidfCoefficients = PIDCoefficients(0.005, 0.0, 0.0)
    @JvmField var middlePos = 1000
}