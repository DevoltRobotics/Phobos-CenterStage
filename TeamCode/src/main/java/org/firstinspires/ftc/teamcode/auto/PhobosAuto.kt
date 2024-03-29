package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.github.serivesmejia.deltacommander.command.DeltaRunCmd
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.PhobosOpMode
import org.firstinspires.ftc.teamcode.lastKnownAlliance
import org.firstinspires.ftc.teamcode.lastKnownPose
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.vision.BlueTeamElementDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.Pattern
import org.firstinspires.ftc.teamcode.vision.RedTeamElementDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.TeamElementDetectionPipeline
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

abstract class PhobosAuto(val alliance: Alliance, var pipeline: TeamElementDetectionPipeline? = null) : PhobosOpMode() {

    val drive get() = hardware.drive

    open val startPose = Pose2d()

    lateinit var camera: OpenCvWebcam
        private set

    override fun setup() {
        intakeArmSub.reset()

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"))

        if(pipeline == null) {
            pipeline = when (alliance) {
                Alliance.RED -> RedTeamElementDetectionPipeline()
                Alliance.BLUE -> BlueTeamElementDetectionPipeline()
            }
        }

        camera.setPipeline(pipeline!!)
        camera.setMillisecondsPermissionTimeout(4000)

        camera.openCameraDeviceAsync(object: AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)

                FtcDashboard.getInstance().startCameraStream(camera, 0.0)
            }

            override fun onError(errorCode: Int) {
            }
        })
    }

    override fun initializeUpdate() {
        telemetry.addData("Pattern", pipeline!!.analysis)
        telemetry.update()
    }

    override fun begin() {
        camera.stopStreaming()

        drive.poseEstimate = startPose

        drive.followTrajectorySequenceAsync(sequence(pipeline!!.analysis))

        + DeltaRunCmd {
            lastKnownAlliance = alliance
            lastKnownPose = drive.poseEstimate

            telemetry.addData("arm", hardware.intakeArm.power)
            telemetry.addData("arm target", intakeArmSub.controller.targetPosition)
            telemetry.addData("arm pos", hardware.intakeArm.currentPosition)

            telemetry.update()
        }
    }

    abstract fun sequence(pattern: Pattern) : TrajectorySequence
}