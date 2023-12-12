package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Giroscopio")
public class AutonomoGyro extends LinearOpMode {

    DcMotor izquierda; // Motores del chasis
    DcMotor derecha;

    IMU revImu; // Sensor IMU

    double P = 0.0065; // Constante P

    double toleranciaGrados = 2; // Valor en el que el robot va a detenerse
    double powerMinimo = 0.15; // Energia minima en las llantas para que el robot se pueda mover

    @Override
    public void runOpMode()  {
        revImu = hardwareMap.get(IMU.class, "imu"); // Obtener el sensor del hardware

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot( // Indicar la posicion y orientacion del hub
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        revImu.initialize(new IMU.Parameters(orientationOnRobot)); // Establecer los parametros de posicion y orientacion

        izquierda = hardwareMap.get(DcMotor.class, "izquierda"); // Obtener los motores del chasis
        derecha = hardwareMap.get(DcMotor.class, "derecha");

        izquierda.setDirection(DcMotorSimple.Direction.REVERSE); // Invertir motor de un lado para contrarrestar la orientacion

        izquierda.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Asegurarnos de que el robot este quieto
        derecha.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        izquierda.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Modo "brake" para que los motores generen resistencia cuando no se mueven
        derecha.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart(); // Esperar a presionar PLAY en la Driver Station

        girar(-180); // Girar 180 grados a la izquierda
    }

    /**
     * Codigo escrito por Devolt Phobos 12887 para la comunidad FIRST Mexico
     */
    public void girar(double grados) { // Codigo de la funcion girar
        revImu.resetYaw(); // Reiniciar el angulo del robot a 0

        double error;

        do {
            double angulo = revImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // Obtener el angulo del robot
            error = grados - angulo; // Calcular la diferencia entre el angulo deseado y el angulo actual

            double power = error * P; // Calcular la energia de los motores con la constante P

            if(Math.abs(power) < powerMinimo) { // Evitar que el robot se detenga antes de lo debido
                                                // estableciendo un minimo de energia

                // Se utiliza el signo (signum) adecuado para mantener la direccion correcta (positivo o negativo)
                power = Math.signum(power) * powerMinimo;
            }

            telemetry.addData("Angulo", angulo); // Escribir datos a la driver station
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);

            telemetry.update(); // Enviar los datos

            if(grados > 0) { // Dependiendo de la direccion de los grados, (positivo o negativo)
                // vamos a hacer que el robot gire a la izquierda o la derecha

                izquierda.setPower(power);
                derecha.setPower(-power); // Girar a la derecha (grados positivos)
            } else if(grados < 0) {
                izquierda.setPower(-power); // Girar a la izquierda (grados negativos)
                derecha.setPower(power);
            }
        } while(Math.abs(error) > toleranciaGrados && opModeIsActive()); // Crear un bucle que no se detiene hasta
                                                                         // que el robot llegue al objetivo o se detenga
                                                                         // el programa
    }

}