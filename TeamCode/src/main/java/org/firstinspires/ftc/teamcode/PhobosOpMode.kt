package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.subsystem.IntakeArmSubsystem
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.github.serivesmejia.deltaevent.opmode.DeltaOpMode
import org.firstinspires.ftc.teamcode.subsystem.*

abstract class PhobosOpMode : DeltaOpMode() {

    override val hardware = PhobosHardware()

    lateinit var mecanumSub: MecanumSubsystem

    lateinit var intakeSub: IntakeSubsystem
    lateinit var intakeArmSub: IntakeArmSubsystem
    lateinit var intakeDoorSub: IntakeDoorSubsystem

    lateinit var liftSub: LiftSubsystem

    lateinit var boxSub: BoxSubsystem
    lateinit var boxArmSub: BoxArmSubsystem

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        mecanumSub = MecanumSubsystem(hardware.drive)

        intakeSub = IntakeSubsystem(hardware.intake)
        intakeArmSub = IntakeArmSubsystem(hardware.intakeArm, hardware.intakeWrist)
        intakeDoorSub = IntakeDoorSubsystem(hardware.intakeDoor)

        liftSub = LiftSubsystem(hardware.lift)

        boxSub = BoxSubsystem(hardware.depositLeft, hardware.depositRight)
        boxArmSub = BoxArmSubsystem(hardware.depositBarsLeft, hardware.depositBarsRight)

        setup()
    }

    override fun runUpdate() {
        lastKnownPose = hardware.drive.poseEstimate
    }

    abstract fun setup()

}