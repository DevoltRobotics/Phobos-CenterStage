package org.firstinspires.ftc.teamcode.subsystem

import com.github.serivesmejia.deltacommander.DeltaSubsystem
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive

class MecanumSubsystem(val drive: SampleMecanumDrive) : DeltaSubsystem() {

    override fun loop() {
        drive.update()
    }

}