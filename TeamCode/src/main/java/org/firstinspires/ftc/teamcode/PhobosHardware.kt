package org.firstinspires.ftc.teamcode

import com.github.serivesmejia.deltasimple.SimpleHardware
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
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

    val liftLeft by hardware<DcMotor>("sl")
    val liftRight by hardware<DcMotor>("sr")

    val hang by hardware<DcMotor>("hg")

    val depositBarsLeft by hardware<Servo>("al")
    val depositBarsRight by hardware<Servo>("ar")

    val depositLeft by hardware<Servo>("dl")
    val depositRight by hardware<Servo>("dr")

    val planeLauncher by hardware<Servo>("pl")

    val blinkin by hardware<RevBlinkinLedDriver>("blinkin")

    override fun init() {
        intake.gobilda()
        intakeDoor.gobilda()
        intakeWrist.gobilda()

        depositBarsLeft.gobilda()
        depositBarsRight.gobilda()

        depositLeft.gobilda()
        depositRight.gobilda()

        planeLauncher.gobilda()
        planeLauncher.position = 0.0

        hang.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

}