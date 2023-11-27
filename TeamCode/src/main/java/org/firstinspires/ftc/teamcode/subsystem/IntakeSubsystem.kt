package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.github.serivesmejia.deltacommander.DeltaSubsystem
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

class IntakeSubsystem(val intakeMotor: DcMotorSimple) : DeltaSubsystem() {
    override fun loop() {
    }
}

class IntakeDoorSubsystem(val doorServo: Servo) : DeltaSubsystem() {
    override fun loop() {
    }
}