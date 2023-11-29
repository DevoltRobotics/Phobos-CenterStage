package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.github.serivesmejia.deltacommander.DeltaSubsystem
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Config
class LiftSubsystem(val liftMotor: DcMotor) : DeltaSubsystem() {

    var controller = createController()
        private set

    var power = 0.0

    init {
        liftMotor.direction = DcMotorSimple.Direction.REVERSE;
    }

    override fun loop() {
        liftMotor.power = power + 0.2
    }

    fun reset() {
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun updateController() {
        liftMotor.power = controller.update(liftMotor.currentPosition.toDouble())
    }

    private fun createController() = PIDFController(pidfCoefficients)

    companion object {
        @JvmStatic var pidfCoefficients = PIDCoefficients(0.03, 0.0001, 0.0001)

        @JvmStatic var middlePos = 1000
    }
}