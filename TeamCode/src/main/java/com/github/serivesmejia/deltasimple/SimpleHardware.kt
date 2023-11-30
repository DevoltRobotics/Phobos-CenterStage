package com.github.serivesmejia.deltasimple

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo

abstract class SimpleHardware {

    lateinit var hardwareMap: HardwareMap

    fun initHardware(hardwareMap: HardwareMap) {
        this.hardwareMap = hardwareMap
        init()
    }
    
    protected open fun init() { }

    inline fun <reified T> hardware(name: String): Lazy<T> = lazy {
        require(::hardwareMap.isInitialized) { "The HardwareMap is not defined, call initHardware(hardwareMap)" }
        hardwareMap.get(T::class.java, name)!!
    }


    // squeezing out extra degrees from gobilda servos
    fun PwmControl.gobilda() {
        pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    fun Servo.gobilda() = (this as PwmControl).gobilda()
    fun CRServo.gobilda() = (this as PwmControl).gobilda()

}