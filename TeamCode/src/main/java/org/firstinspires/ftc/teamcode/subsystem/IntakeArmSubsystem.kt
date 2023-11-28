package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.github.serivesmejia.deltacommander.DeltaSubsystem
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmStopCmd

class IntakeArmSubsystem(val armMotor: DcMotorEx, val wristServo: Servo) : DeltaSubsystem() {

    var controller = createController()
        private set

    var wristBusy = false
    var downWristPosition = 0.5

    private var previousCoeffs = pidfCoefficients.copy()

    init {
        defaultCommand = IntakeArmStopCmd()
    }

    override fun init() {
        controller.targetPosition = armMotor.currentPosition.toDouble()
    }

    override fun loop() {
        if(!wristBusy) {
            if(armMotor.currentPosition >= -200) {
                wristServo.position = 0.45
            } else {
                wristServo.position = downWristPosition
            }
        }

        if(pidfCoefficients != previousCoeffs) {
            controller = createController()
        }

        previousCoeffs = pidfCoefficients.copy()
    }

    fun updateController() {
        armMotor.power = controller.update(armMotor.currentPosition.toDouble(), armMotor.velocity)
    }

    fun reset() {
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun createController() = PIDFController(pidfCoefficients, kV, kA, kStatic)

    @Config
    companion object {
        @JvmStatic var pidfCoefficients = PIDCoefficients(0.01, 0.0001, 0.0001)

        @JvmStatic var kV = 0.00027
        @JvmStatic var kA = 0.00001
        @JvmStatic var kStatic = 0.05

        @JvmStatic var ticksPerSecond = 300
        @JvmStatic var ticksPerPerSecond = 100
    }
}
