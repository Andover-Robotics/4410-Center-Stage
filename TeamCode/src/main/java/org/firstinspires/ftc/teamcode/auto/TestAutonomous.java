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
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline2;
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
    public static double stackDelay = 0.0; // Delay before going to stack
    public static int park = 0; // Parking position (0-none, 1-left, 2-right)
    public static int side = 1; // Side (1-close, 2-far)
    public static int spikeMark = 0; // Which spike mark randomization object is detected on (1-left, 2-center, 3-right)
    private int secondsElapsed = 0; // Track how many seconds have passed

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
        ColorDetectionPipeline2 colorDetection = new ColorDetectionPipeline2(telemetry);
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
        try { camera.stopStreaming(); camera.closeCameraDevice();}
        catch ( OpenCvCameraException ignored ) { }

        // Threads
        // Slides periodic
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

        boolean dropped = false;
        while (!isStarted()) {
            gp1.readButtons();

            // Drop/pickup pixel
            if (gp1.wasJustPressed(GamepadKeys.Button.START)){
                if (!dropped){ // Drop
                    bot.claw.fullOpen();
                    dropped = true;
                } else { // Re-grip
                    bot.slides.runToBottom();
                    bot.claw.fullOpen();
                    sleep(100);
                    bot.fourbar.pickup();
                    sleep(400);
                    bot.claw.pickupClose();
                    sleep(300);
                    bot.storage();
                    dropped = false;
                }
            }

            // Change side
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                side = side == 1 ? 2 : 1;
            }
            telemetry.addData("side", side);

            // Change stack wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (stackDelay < 10.0) stackDelay+=1.0;
                else stackDelay = 0.0;
            }
            telemetry.addData("stack delay", stackDelay);

            // Initiate color detection
            spikeMark = colorDetection.getSpikeMark();
            String spikeMarkString = "";
            switch (spikeMark) {
                case 1: spikeMarkString = "left"; break;
                case 2: spikeMarkString = "center"; break;
                case 3: spikeMarkString = "right"; break;
                default: spikeMarkString = "none"; break;
            }
            telemetry.addData("detected spike", spikeMarkString);

            // Controls
            telemetry.addLine("START-drop/pickup, A-side, DPAD_LEFT-stack delay");

            // Update
            telemetry.update();
            sleep(20);
        }

        waitForStart();

        // Auto start
        if (opModeIsActive() && !isStopRequested()) {
            // Run threads
            trackTime.start();
            periodic.start();

            // Define starting pose
            // TODO: TUNE THIS STARTING POSE
            Pose2d startPose = new Pose2d();
            startPose = new Pose2d(12,60,Math.toRadians(90));

            // To spike mark
            // Define spike mark pose
            TrajectorySequence spikeTrajectory = null;
            if (side == 1) { // Close side
                switch (spikeMark) {
                    case 1: spikeTrajectory = drive.trajectorySequenceBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(35, 35, Math.toRadians(180)))
                            .build(); break;
                    case 2: spikeTrajectory = drive.trajectorySequenceBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(28, 25, Math.toRadians(180)))
                            .build(); break;
                    case 3: spikeTrajectory = drive.trajectorySequenceBuilder(startPose).back(2)
                            .splineToConstantHeading(new Vector2d(25, 40), Math.toRadians(300))
                            .splineToSplineHeading(new Pose2d(12, 33, Math.toRadians(180)), Math.toRadians(180))
                            .build(); break;
                }
            } else { // Far side
                switch (spikeMark) {
                    case 1: spikeTrajectory = drive.trajectorySequenceBuilder(startPose)
                            .lineToSplineHeading(new Pose2d(-46, 15, Math.toRadians(90)))
                            .build(); break;
                    case 2:
                    case 3:
                }
            }
            drive.followTrajectorySequence(spikeTrajectory);
            // Place purple pixel - 0.5 seconds
            bot.intake(true);
            sleep(500);

            // To backboard
            // Define backboard y value
            int backboardX = 51, backboardY = 0;
            switch (spikeMark) {
                case 1: backboardY = 35; break;
                case 2: backboardY = 41; break;
                case 3: backboardY = 29; break;
            }
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(backboardX, backboardY, Math.toRadians(180)))
                    .addDisplacementMarker(10, () -> {
                        bot.fourbar.topOuttake(true);
                        bot.fourbar.setArm(0.21);
                    })
                    .build());
            // Place yellow pixel - 0.5 seconds
            bot.claw.setPosition(0.66);
            sleep(400);
            bot.fourbar.setWrist(0.65);
            sleep(100);

            // Pixel stack trajectory starts here
            // Check if there is stack delay, if there is, run to corner
            if (stackDelay != 0.0) {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(55, 11, Math.toRadians(180)),Math.toRadians(20))
                    .waitSeconds(stackDelay)
                    .build());
            }
            // To pixel stack
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)), Math.toRadians(180))
                    .addDisplacementMarker(2, () -> {
                        bot.storage();
                        bot.claw.close();
                    })
                    .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180)))
                    .build());
            // Intake from stack - 1 second
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
            // Score pixels on backboard - 0.8 seconds
            bot.claw.halfOpen();
            sleep(500);
            bot.slides.runTo(-700.0);
            bot.outtakeOut(1);
            sleep(300);
            bot.claw.setPosition(0.66);
            sleep(200);

            // Parking trajectory
            if (park != 0) {
                // Define parking y value
                int parkY = 0;
                if (park == 1) parkY = 59; else if (park == 2) parkY = 11;
                // To park
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(52, parkY, Math.toRadians(180)),Math.toRadians(15))
                    .addDisplacementMarker(2, () -> {
                        bot.storage();
                        bot.claw.close();
                    })
                    .build());
            }

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