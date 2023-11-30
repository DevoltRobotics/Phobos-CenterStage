package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.github.serivesmejia.deltacommander.DeltaSubsystem
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmStopCmd

@Config
class IntakeArmSubsystem(val armMotor: DcMotorEx, val wristServo: Servo) : DeltaSubsystem() {

    var controller = createController()
        private set

    var downWristPosition = 0.53

    private var previousCoeffs = pidfCoefficients.copy()

    init {
    }

    override fun init() {
        controller.targetPosition = armMotor.currentPosition.toDouble()
    }

    override fun loop() {
    }

    fun updateController() {
        armMotor.power = controller.update(armMotor.currentPosition.toDouble())
    }

    fun reset() {
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun createController() = PIDFController(pidfCoefficients, kV, kA, kStatic)

    companion object {
        @JvmStatic var pidfCoefficients = PIDCoefficients(0.01, 0.0001, 0.0001)

        @JvmStatic var kV = 0.0
        @JvmStatic var kA = 0.0
        @JvmStatic var kStatic = 0.0

        @JvmStatic var drivingTicksPerSecond = 250

        @JvmStatic var ticksPerSecond = 100
        @JvmStatic var ticksPerPerSecond = 60
    }
}