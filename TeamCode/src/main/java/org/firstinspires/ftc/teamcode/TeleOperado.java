package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleOperado extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor izquierda = hardwareMap.dcMotor.get("izquierda");
        DcMotor derecha = hardwareMap.dcMotor.get("derecha");

        waitForStart();

        while(opModeIsActive()) {
            izquierda.setPower(-gamepad1.left_stick_y);
            derecha.setPower(-gamepad1.right_stick_y);
        }
    }

}
