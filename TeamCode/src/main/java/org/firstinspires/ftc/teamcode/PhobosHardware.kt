package org.firstinspires.ftc.teamcode

import com.github.serivesmejia.deltasimple.SimpleHardware
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.hardware.rev.RevTouchSensor

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive

class PhobosHardware : SimpleHardware() {

    val drive by lazy { SampleMecanumDrive(hardwareMap) }

    val intake by hardware<CRServo>("in")
    val intakeArm by hardware<DcMotorEx>("ia")

    val intakeDoor by hardware<Servo>("id")
    val intakeWrist by hardware<Servo>("iw")

    val lift by hardware<DcMotor>("sl")

    val depositBarsLeft by hardware<Servo>("al")
    val depositBarsRight by hardware<Servo>("ar")

    val depositLeft by hardware<Servo>("dl")
    val depositRight by hardware<Servo>("dr")

    override fun init() {
        intake.gobilda()
        intakeDoor.gobilda()
        intakeWrist.gobilda()

        depositBarsLeft.gobilda()
        depositBarsRight.gobilda()

        depositLeft.gobilda()
        depositRight.gobilda()
    }

    // squeezing out extra degrees from gobilda servos
    fun PwmControl.gobilda() {
        pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    fun Servo.gobilda() = (this as PwmControl).gobilda()
    fun CRServo.gobilda() = (this as PwmControl).gobilda()

}