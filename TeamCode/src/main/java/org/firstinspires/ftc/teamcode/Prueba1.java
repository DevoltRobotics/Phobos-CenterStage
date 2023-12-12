package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Prueba1 extends LinearOpMode {

    DcMotor izquierda; // Motores del chasis
    DcMotor derecha;

    @Override
    public void runOpMode() {
        izquierda = hardwareMap.get(DcMotor.class, "izquierda"); // Obtener los motores del chasis
        derecha = hardwareMap.get(DcMotor.class, "derecha");

        izquierda.setDirection(DcMotorSimple.Direction.REVERSE); // Invertir motor de un lado para contrarrestar la orientacion

        izquierda.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Asegurarnos de que el robot este quieto
        derecha.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        izquierda.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        derecha.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            izquierda.setPower(-gamepad1.left_stick_y);
            derecha.setPower(-gamepad1.right_stick_y);
        }
    }

}
