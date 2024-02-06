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
    private int secondsElapsed = 0; // Track how many seconds have passed

    // CONFIGURATION VARIABLES
    enum Side {CLOSE, FAR} public static Side side = Side.CLOSE; // Side relative to backboard, defaults close
    enum Alliance {BLUE, RED} public static Alliance alliance = Alliance.BLUE; // Alliance, defaults blue
    public static int park = 0; // Parking position (0-none, 1-left, 2-right)
    public static int spikeMark = 0; // Which spike mark randomization object is detected on (1-left, 2-center, 3-right)
    public static int stackIterations = 0; // How many times go to stack (0-none, 1-2+2, 2-2+4, 3-2+5)
    public static int slidesHeight = 0; // Height of slides when scoring yellow pixel on backboard
    // BOOLEANS
    public static boolean toBackboard = true; // Go to backboard or stop after scoring purple pixel on spike mark
    public static boolean toStack = false; // Go to pixel stack for extra points or stop after scoring yellow pixel
    // DELAYS
    public static double spikeDelay = 0.0; // Delay before going to spike mark
    public static double stackDelay = 0.0; // Delay before going to stack, will park on side and wait
    public static double backboardDelay = 0.0; // Delay before going to backboard, after scoring spike mark

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

        Thread stopAuto = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                if (drive.getExternalHeadingVelocity() > (0.2 * DriveConstants.MAX_ANG_VEL)) {
                    telemetry.addLine("SHUTTING DOWN!!!");
                    requestOpModeStop();
                }
            }
        });


        // Configuration variables
        boolean dropped = false;
        String spikeMarkString = "", stackString = "", parkString = "";
        /*
        LIST OF CONFIGURATION CONTROLS: last updated 1/29/24 - zachery

        Y - change alliance
        A - change side
        X - change park
        B - change pixel stack configurations

        dpad up - backboard delay
        dpad down - spike mark delay
        dpad left - stack delay
        dpad right -

        joystick left button - toggle to backboard
        joystick right button - toggle to stack

        right bumper - increment slides height
        left bumper - decrement slides height

        start - drop/pickup
         */
        // Initialized configurations start
        while (!isStarted()) {
            gp1.readButtons();

            // Drop/pickup pixel
            if (gp1.wasJustPressed(GamepadKeys.Button.START)){
                if (!dropped){ // Drop
                    bot.claw.fullOpen();
                    dropped = true;
                } else { // Pickup
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

            // Change alliance
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                alliance = alliance == Alliance.BLUE ? Alliance.RED : Alliance.BLUE;
            }
            // Change side
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                side = side == Side.CLOSE ? Side.FAR : Side.CLOSE;
            }
            telemetry.addData("side", side + " alliance: " + alliance);

            // Change pixel stack configurations
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                if (side == Side.CLOSE) {
                    switch (stackIterations) {
                        case 0: stackIterations = 1; toStack = true; stackString = "2+2"; break;
                        case 1: stackIterations = 2; toStack = true; stackString = "2+4"; break;
                        case 2: stackIterations = 0; toStack = false; stackString = "2+0"; break;
                    }
                } else {
                    switch (stackIterations) {
                        case 0: stackIterations = 1; toStack = true; stackString = "2+3"; break;
                        case 1: stackIterations = 2; toStack = true; stackString = "2+5"; break;
                        case 2: stackIterations = 0; toStack = false; stackString = "2+0"; break;
                    }
                }
            }
            telemetry.addData("pixel stack", stackString + " (x" + stackIterations + ")");

            // Change park
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                switch (park) {
                    case 0: park = 1; parkString = "Left"; break;
                    case 1: park = 2; parkString = "Right"; break;
                    case 2: park = 0; parkString = "None"; break;
                }
            }
            telemetry.addData("Parking (X)", parkString);

            // DELAYS
            // Backboard
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                if (backboardDelay < 10.0) backboardDelay+=1.0;
                else backboardDelay = 0.0;
            }
            // Spike mark
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                if (spikeDelay < 10.0) spikeDelay+=1.0;
                else spikeDelay = 0.0;
            }
            // Stack
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (stackDelay < 10.0) stackDelay+=1.0;
                else stackDelay = 0.0;
            }
            telemetry.addData("delays(s): backboard", backboardDelay + " spike: " + spikeDelay + " stack: " + stackDelay);

            // BOOLEANS
            // Toggle to backboard
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) toBackboard = !toBackboard;
            // Toggle to stack
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) toStack = !toStack;
            telemetry.addData("to backboard", toBackboard + " to stack: " + toStack);

            // SLIDE HEIGHT
            // Increment
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                slidesHeight += slidesHeight < 1000 ? 200 : 0;
            }
            // Decrement
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                slidesHeight -= slidesHeight > 0 ? 200 : 0;
            }
            telemetry.addData("slides height", slidesHeight);

            // Initiate color detection
            if (alliance == Alliance.RED) colorDetection.setAlliance(1);
            else colorDetection.setAlliance(2);
            spikeMark = colorDetection.getSpikeMark();
            switch (spikeMark) {
                case 1: spikeMarkString = "LEFT"; break;
                case 2: spikeMarkString = "CENTER"; break;
                case 3: spikeMarkString = "RIGHT"; break;
                default: spikeMarkString = "NONE"; break;
            }
            telemetry.addData("detected spike", spikeMarkString);

            // Update
            telemetry.update();
            sleep(20);
        }

        waitForStart();

        // Auto start
        if (opModeIsActive() && !isStopRequested()) {
            // Run threads
            trackTime.start();
            stopAuto.start();
            periodic.start();

            // Define starting pose
            // TODO: TUNE EACH STARTING POSE
            if (alliance == Alliance.BLUE && side == Side.CLOSE) { // Blue close
                drive.setPoseEstimate(new Pose2d(12,60,Math.toRadians(90)));
            } else if (alliance == Alliance.RED && side == Side.CLOSE) { // Red close
                drive.setPoseEstimate(new Pose2d(12,-60,Math.toRadians(-90)));
            } else if (alliance == Alliance.BLUE && side == Side.FAR) { // Blue far
                drive.setPoseEstimate(new Pose2d(-35,60,Math.toRadians(90)));
            } else if (alliance == Alliance.RED && side == Side.FAR) { // Red far
                drive.setPoseEstimate(new Pose2d(-35,-60,Math.toRadians(-90)));
            }

            // To spike mark
            // Define spike mark pose
            TrajectorySequence spikeTrajectory = null;
            if (alliance == Alliance.RED) {
                if (side == Side.CLOSE) { // Close side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .back(2)
                                .splineToConstantHeading(new Vector2d(25, -40), Math.toRadians(45))
                                .splineToSplineHeading(new Pose2d(12, -33, Math.toRadians(180)), Math.toRadians(180))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28, -25, Math.toRadians(180)))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(35, -35, Math.toRadians(180)))
                                .build(); break;
                    }
                } else { // Far side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-44, -45, Math.toRadians(130)))
                                .splineToSplineHeading(new Pose2d(-35, -35, Math.toRadians(180)), Math.toRadians(1))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineTo(new Vector2d(-38,-14))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-42, -46, Math.toRadians(130)))
                                .splineToSplineHeading(new Pose2d(-36, -32, Math.toRadians(180)), Math.toRadians(1))
                                .build(); break;
                    }
                }
            } else {
                if (side == Side.CLOSE) { // Close side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(35, 35, Math.toRadians(180)))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(28, 25, Math.toRadians(180)))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate()).back(2)
                                .splineToConstantHeading(new Vector2d(25, 40), Math.toRadians(225))
                                .splineToSplineHeading(new Pose2d(12, 33, Math.toRadians(180)), Math.toRadians(180))
                                .build(); break;
                    }
                } else { // Far side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-42, 46, Math.toRadians(130)))
                                .splineToSplineHeading(new Pose2d(-36, 32, Math.toRadians(180)), Math.toRadians(1))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineTo(new Vector2d(-38,14))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-44, 45, Math.toRadians(130)))
                                .splineToSplineHeading(new Pose2d(-35, 35, Math.toRadians(180)), Math.toRadians(1))
                                .build(); break;
                    }
                }
            }
            sleep((long) spikeDelay * 100);
            drive.followTrajectorySequence(spikeTrajectory);
            // Place purple pixel
            bot.intake(true);
            sleep(500);
            bot.intake(false);
            sleep((long) backboardDelay * 100);

            // To backboard
            if (toBackboard) {
                // Extra movements for pixel stack on far side
                if (side == Side.FAR) {
                    TrajectorySequence stackTrajectory = null;
                    int stackX = -59;
                    if (alliance == Alliance.RED) {
                        switch (spikeMark) {
                            case 1: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-37, -24, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(stackX, -11), Math.toRadians(225))
                                    .build(); break;
                            case 2: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(stackX, -11, Math.toRadians(180)))
                                    .build(); break;
                            case 3: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-36, -16, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(stackX, -11), Math.toRadians(180))
                                    .build(); break;
                        }
                    } else {
                        switch (spikeMark) {
                            case 1: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-36, 16, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(stackX, 11), Math.toRadians(180))
                                    .build(); break;
                            case 2: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(stackX, 11, Math.toRadians(180)))
                                    .build(); break;
                            case 3: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-37, 24, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(stackX, 11), Math.toRadians(225))
                                    .build(); break;
                        }
                    }
                    drive.followTrajectorySequence(stackTrajectory);
                    // Intake from stack
                    bot.intake.setIntakeHeight(0.18);
                    bot.intake(false);
                    sleep(1000);
                    bot.intake.setIntakeHeight(bot.intake.intakeStorage);
                    // Across the field
                    int farY = alliance == Alliance.RED ? -11 : 11;
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(25, farY, Math.toRadians(180)))
                            .build());
                }

                // Define backboard y value
                int backboardX = 51, backboardY = 0;
                if (alliance == Alliance.RED) {
                    switch (spikeMark) {
                        case 1: backboardY = -30; break;
                        case 2: backboardY = -36; break;
                        case 3: backboardY = -41; break;
                    }
                } else {
                    switch (spikeMark) {
                        case 1: backboardY = 41; break;
                        case 2: backboardY = 37; break;
                        case 3: backboardY = 29; break;
                    }
                }
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(backboardX, backboardY, Math.toRadians(180)))
                        .addDisplacementMarker(10, () -> {
                            bot.fourbar.topOuttake(true);
                            bot.fourbar.setArm(0.21);
                            if (slidesHeight != 0) bot.slides.runTo(-slidesHeight);
                            else bot.slides.runToBottom();
                        })
                        .build());
                // Place yellow pixel
                if (side == Side.CLOSE) {
                    bot.claw.setPosition(0.66);
                    sleep(400);
                    bot.fourbar.setWrist(0.65);
                    sleep(100);
                } else { // Place both white and yellow
                    bot.intake.stopIntake();
                    bot.claw.halfOpen();
                    sleep(400);
                    bot.fourbar.setWrist(0.22);
                    bot.slides.runTo(-800);
                    bot.outtakeOut(1);
                    sleep(300);
                    bot.fourbar.setWrist(0.13);
                    sleep(100);
                    bot.claw.setPosition(0.66);
                    sleep(200);
                }

                // Pixel stack trajectory starts here
                double [] stackHeights = side == Side.FAR ? new double [] {0.2, 0.24} : new double [] {0.22, 0.27};
                if (toStack) {
                    int stackY1 = alliance == Alliance.RED ? -11 : 11;
                    int stackY2 = alliance == Alliance.RED ? -30 : 30;
                    // Check if there is stack delay, if there is, run to corner
                    if (stackDelay != 0.0) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(55, stackY1, Math.toRadians(180)),Math.toRadians(20))
                                .waitSeconds(stackDelay)
                                .build());
                    }
                    for (int i = 1; i < stackIterations; i++) {
                        // To pixel stack
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(30, stackY1, Math.toRadians(180)), Math.toRadians(180))
                                .addDisplacementMarker(2, () -> {
                                    bot.storage();
                                    bot.claw.close();
                                })
                                .lineToLinearHeading(new Pose2d(-60, stackY1, Math.toRadians(180)))
                                .build());
                        // Intake from stack
                        bot.intake.setIntakeHeight(stackHeights[i-1]);
                        bot.intake(false);
                        sleep(1000);
                        bot.intake.setIntakeHeight(bot.intake.intakeStorage);
                        // To backboard
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(38, stackY1, Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(50, stackY2, Math.toRadians(180)), Math.toRadians(0))
                                .addDisplacementMarker(5, () -> {
                                    bot.slides.runTo(-1000);
                                    bot.outtakeOut(2);
                                    bot.fourbar.setArm(0.21);
                                })
                                .build());
                        // Score pixels on backboard
                        bot.intake.stopIntake();
                        bot.claw.halfOpen();
                        sleep(400);
                        bot.fourbar.setWrist(0.22);
                        bot.slides.runTo(-800);
                        bot.outtakeOut(1);
                        sleep(300);
                        bot.fourbar.setWrist(0.13);
                        sleep(100);
                        bot.claw.setPosition(0.66);
                        sleep(200);
                    }
                }

                // Parking trajectory
                if (park != 0) {
                    int parkY = alliance == Alliance.RED ? (park == 1 ? -11 : -59) : (park == 1 ? 59 : 11);
                    // To park
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(52, parkY, Math.toRadians(180)),Math.toRadians(15))
                            .addDisplacementMarker(2, () -> {
                                bot.storage();
                                bot.claw.close();
                            })
                            .build());
                }
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