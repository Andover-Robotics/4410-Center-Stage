package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.auto.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.*;
import org.firstinspires.ftc.teamcode.auto.util.PoseStorage;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

import java.util.Map;
import java.util.Objects;

@Config
@Autonomous(name = "TestAutonomous")
public class TestAutonomous extends LinearOpMode {
    Bot bot;

    int secondsElapsed = 0; // Track how many seconds have passed

    @Override
    public void runOpMode() throws InterruptedException {
        // Instantiate
        telemetry.setAutoClear(true);
        bot = Bot.getInstance(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        // Define camera values
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        ColorDetectionPipeline colorDetection = new ColorDetectionPipeline(telemetry);
        // Start camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error code:", errorCode);
            }
        });
        camera.setPipeline(colorDetection);

        // Threads
        // Run slides periodic
        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
            }
        });
        // Track time passed while op mode is running
        Thread trackTime = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                secondsElapsed++;
                telemetry.addData("Seconds",secondsElapsed);
                telemetry.update();
                sleep(1000);
            }
        });

        // Bot initialized state
        bot.state = Bot.BotState.STORAGE;
        bot.storage();
        bot.claw.fullOpen();
        // Pickup
        Thread pickup = new Thread(() -> {
            sleep(500);
            bot.slides.runToBottom();
            bot.claw.fullOpen();
            sleep(400);
            bot.fourbar.pickup();
            sleep(400);
            bot.claw.pickupClose();
            sleep(300);
            bot.storage();
            sleep(200);
            stop();
        });
        pickup.start();
        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException ignored) { }
        waitForStart();

        // Auto start
        if (opModeIsActive() && !isStopRequested()) {
            trackTime.start();
            periodic.start();
            Pose2d startPose = new Pose2d(12,60,Math.toRadians(90));
            // To spike mark
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(28, 25, Math.toRadians(180)))
                .build());
            // Place purple pixel
            bot.intake(true);
            sleep(500);
            // To backboard
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(51, 35, Math.toRadians(180)))
                    .addDisplacementMarker(10, () -> {
                        bot.fourbar.topOuttake(true);
                        bot.fourbar.setArm(0.21);
                    })
                    .build());
            // Place yellow pixel
            bot.claw.setPosition(0.66);
            sleep(400);
            bot.fourbar.setWrist(0.65);
            sleep(100);
            // To pixel stack
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)), Math.toRadians(180))
                    .addDisplacementMarker(2, () -> {
                        bot.storage();
                        bot.claw.close();
                    })
                    .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180)))
                    .build());
            // Intake from stack
            bot.intake.setIntakeHeight(0.5);
            bot.intake(false);
            sleep(1000);
            bot.intake.stopIntake();
            bot.intake.setIntakeHeight(bot.intake.intakeStorage);
            // To backboard
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(38, 11, Math.toRadians(180)))
                    .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(90))
                    .addDisplacementMarker(5, () -> {
                        bot.slides.runTo(-500.0);
                        bot.outtakeOut(2);
                        bot.fourbar.setArm(0.21);
                    })
                    .build());
            // Score pixels on backboard
            bot.claw.halfOpen();
            sleep(500);
            bot.slides.runTo(-700.0);
            bot.outtakeOut(1);
            sleep(300);
            bot.claw.setPosition(0.66);
            sleep(200);
            // To park
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(52, 11, Math.toRadians(180)),Math.toRadians(15))
                    .addDisplacementMarker(2, () -> {
                        bot.storage();
                        bot.claw.close();
                    })
                    .build());
            // Stop op mode
            sleep(500);
            PoseStorage.currentPose = drive.getPoseEstimate();
            requestOpModeStop();
        }

        //periodic.interrupt();
        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) {
            telemetry.addLine("Exception as followsL: " + e);
        }
    }
}