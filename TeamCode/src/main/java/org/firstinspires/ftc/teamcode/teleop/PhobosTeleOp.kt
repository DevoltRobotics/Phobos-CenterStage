package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.github.serivesmejia.deltacommander.command.DeltaInstantCmd
import com.github.serivesmejia.deltacommander.command.DeltaRunCmd
import com.github.serivesmejia.deltacommander.dsl.deltaSequence
import com.github.serivesmejia.deltacommander.endRightAway
import com.github.serivesmejia.deltaevent.gamepad.button.Button
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.PhobosOpMode
import org.firstinspires.ftc.teamcode.command.box.BoxArmDownCmd
import org.firstinspires.ftc.teamcode.command.box.BoxArmDriveCmd
import org.firstinspires.ftc.teamcode.command.box.BoxArmPositionCmd
import org.firstinspires.ftc.teamcode.command.box.BoxDoorsDriveCmd
import org.firstinspires.ftc.teamcode.command.intake.IntakeDoorCloseCmd
import org.firstinspires.ftc.teamcode.command.intake.IntakeDoorOpenCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmDriveCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.intake.arm.IntakeArmWristPositionCmd
import org.firstinspires.ftc.teamcode.command.lift.LiftDriveCmd
import org.firstinspires.ftc.teamcode.command.lift.LiftGoToPositionCmd
import org.firstinspires.ftc.teamcode.command.mecanum.FieldCentricMecanumCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeAbsorbCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeReleaseCmd
import org.firstinspires.ftc.teamcode.command.mecanum.IntakeStopCmd
import org.firstinspires.ftc.teamcode.lastKnownAlliance
import org.firstinspires.ftc.teamcode.lastKnownPose
import kotlin.math.abs

@TeleOp(name = "Carlos", group = "##PHOBOS")
class PhobosTeleOp : PhobosOpMode() {

    override fun setup() {
        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);

        intakeArmSub.reset()
        liftSub.reset()

        hardware.drive.poseEstimate = lastKnownPose.plus(Pose2d(0.0, 0.0, lastKnownAlliance.angleOffset))

        /* START A */

        // MECANUM

        superGamepad1.scheduleOnPress(Button.LEFT_STICK_BUTTON, DeltaInstantCmd {
            hardware.drive.poseEstimate = Pose2d(0.0, 0.0, 0.0)
        })

        + FieldCentricMecanumCmd(gamepad1)

        // INTAKE

        superGamepad1.scheduleOn(Button.A,
                IntakeAbsorbCmd(),
                IntakeStopCmd()
        )

        superGamepad1.scheduleOn(Button.RIGHT_BUMPER,
                IntakeAbsorbCmd(),
                IntakeStopCmd()
        )

        superGamepad1.scheduleOn(Button.B,
                IntakeReleaseCmd(),
                IntakeStopCmd()
        )

        superGamepad1.scheduleOn(Button.LEFT_BUMPER,
                IntakeReleaseCmd(),
                IntakeStopCmd()
        )

        superGamepad2.scheduleOn(Button.LEFT_BUMPER,
            DeltaInstantCmd { hardware.planeLauncher.position = 0.0 },
            DeltaInstantCmd { hardware.planeLauncher.position = 0.2 }
        )

        /* START B */

        // INTAKE ARM

        intakeArmSub.defaultCommand = IntakeArmDriveCmd({ -gamepad2.left_stick_y.toDouble() }, { 0.0 })

        + DeltaRunCmd {
            if(abs(-gamepad2.left_stick_y) >= 0.1) {
                intakeArmSub.free()
            }
        }

        superGamepad2.scheduleOn(Button.DPAD_LEFT,
                IntakeArmGoToPositionCmd(0)
        )

        // INTAKE

        superGamepad2.scheduleOn(Button.A,
                IntakeDoorOpenCmd(),
                IntakeDoorCloseCmd()
        )

        superGamepad2.toggleScheduleOn(Button.B,
                IntakeArmWristPositionCmd(0.42).endRightAway(),
                IntakeArmWristPositionCmd(0.54).endRightAway()
        )

        // LIFT

        liftSub.defaultCommand = LiftDriveCmd { -gamepad2.right_stick_y.toDouble() }

        + DeltaRunCmd {
            if(abs(-gamepad2.right_stick_y) >= 0.1) {
                liftSub.free()
            }
        }

        superGamepad2.scheduleOnPress(Button.DPAD_RIGHT,
                BoxArmPositionCmd(0.42)
        )

        superGamepad2.scheduleOnPress(Button.DPAD_DOWN,
                BoxArmDownCmd()
        )

        superGamepad2.scheduleOnPress(Button.DPAD_UP,
            LiftGoToPositionCmd(1200)
        )

        // DEPOSIT BOX

        boxArmSub.defaultCommand = BoxArmDriveCmd { (gamepad2.right_trigger - gamepad2.left_trigger).toDouble() }

        + DeltaRunCmd {
            if(gamepad2.right_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
                boxArmSub.free()
            }
        }

        boxSub.defaultCommand = BoxDoorsDriveCmd({gamepad1.x}, {gamepad1.y})

        // HANG

        superGamepad2.scheduleOn(Button.X,
            DeltaInstantCmd { hardware.hang.power = 1.0 },
            DeltaInstantCmd { hardware.hang.power = 0.0 }
        )

        superGamepad2.scheduleOn(Button.Y,
            deltaSequence {
                - cuelgaSequence().async()
                - DeltaInstantCmd { hardware.hang.power = -1.0 }
            },
            DeltaInstantCmd { hardware.hang.power = 0.0 }
        )

        // telemetry
        + DeltaRunCmd {
            telemetry.addData("pose", hardware.drive.poseEstimate)

            telemetry.addData("lift target", liftSub.controller.targetPosition)
            telemetry.addData("lift", liftSub.leftMotor.currentPosition)

            telemetry.addData("arm", hardware.intakeArm.power)
            telemetry.addData("arm target", intakeArmSub.controller.targetPosition)
            telemetry.addData("arm pos", hardware.intakeArm.currentPosition)

            telemetry.addData("fbl pos", hardware.depositBarsLeft.position)
            telemetry.addData("fbr pos", hardware.depositBarsRight.position)

            telemetry.addData("fl", hardware.drive.leftFront.power)
            telemetry.addData("fl pos", hardware.drive.leftFront.currentPosition)
            telemetry.addData("fr", hardware.drive.rightFront.power)
            telemetry.addData("fr pos", hardware.drive.rightFront.currentPosition)
            telemetry.addData("bl", hardware.drive.leftRear.power)
            telemetry.addData("bl pos", hardware.drive.leftRear.currentPosition)
            telemetry.addData("br", hardware.drive.rightRear.power)
            telemetry.addData("br pos", hardware.drive.rightRear.currentPosition)

            telemetry.update()
        }
    }

    fun cuelgaSequence() = deltaSequence {
        - IntakeArmWristPositionCmd(0.7).async()
        - BoxArmPositionCmd(0.1)
    }

}