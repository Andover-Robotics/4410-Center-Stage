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
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public static int slidesHeight = 0; // Height of slides when scoring yellow pixel on backboard (0-1000)
    // BOOLEANS
    public static boolean toBackboard = true; // Go to backboard or stop after scoring purple pixel on spike mark
    public static boolean toStack = false; // Go to pixel stack for extra points or stop after scoring yellow pixel
    // DELAYS
    public static double spikeDelay = 0.0; // Delay before going to spike mark
    public static double stackDelay = 0.0; // Delay before going to stack, will park on side and wait
    public static double backboardDelay = 0.0; // Delay before going to backboard, after scoring spike mark
    DigitalChannel breakBeam;


    @Override
    public void runOpMode() throws InterruptedException {
        // Instantiate
        telemetry.setAutoClear(true);
        bot = Bot.getInstance(this);
        breakBeam = hardwareMap.get(DigitalChannel.class, "BreakBeam");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        // Define camera values
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        ColorDetectionPipeline2 colorDetection = new ColorDetectionPipeline2(telemetry);
        // Start camera
        camera.setPipeline(colorDetection);
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
            sleep(250);
            bot.slides.runToBottom();
            bot.claw.fullOpen();
            sleep(100);
            bot.fourbar.pickup();
            sleep(400);
            bot.claw.pickupClose();
            sleep(300);
            bot.storage();
        });
        pickup.start();

        // Tap pixel
        Thread pixelTap = new Thread(() -> {
            sleep(100);
            bot.slides.runToBottom();
            bot.claw.fullOpen();
            bot.intake.setIntakeHeight(0.1);
            sleep(100);
            bot.fourbar.pickup();
            sleep(200);
            bot.claw.pickupClose();
            sleep(250);
            bot.storage();
            sleep(150);
            bot.claw.fullOpen();
        });

        // Pick up in auto
        Thread autoPickup = new Thread(() -> {
            bot.intake.stopIntake();
            bot.slides.runToBottom();
            bot.claw.fullOpen();
            sleep(100);
            bot.fourbar.pickup();
            sleep(250);
            bot.claw.pickupClose();
            sleep(300);
            bot.storage();
        });


        Thread trackHeading = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                if (drive.getExternalHeadingVelocity() > (DriveConstants.MAX_ANG_VEL)) {
                    telemetry.addLine("SHUTTING DOWN!!!");
                    requestOpModeStop();
                }
            }
        });
        trackHeading.start();


        // Configuration variables
        boolean dropped = false;
        String spikeMarkString = "", stackString = "2+0 (x0)", parkString = "None";
        /*
        LIST OF CONFIGURATION CONTROLS: last updated 2/9/24 - zachery

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
        back - manual spike mark (FOR TUNING)
         */
        // Initialized configurations start
        while (!isStarted()) {
            gp1.readButtons();
            gp2.readButtons();

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
            telemetry.addData("side (a)", side + " alliance (y): " + alliance);

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
            telemetry.addData("pixel stack (b)", stackString + " (x" + stackIterations + ")");

            // Change park
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                switch (park) {
                    case 0: park = 1; parkString = "Left"; break;
                    case 1: park = 2; parkString = "Right"; break;
                    case 2: park = 0; parkString = "None"; break;
                }
            }
            telemetry.addData("parking (X)", parkString);

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
            telemetry.addData("delays: backboard (up)", backboardDelay + " spike (down): " + spikeDelay + " stack (left): " + stackDelay);

            // BOOLEANS
            // Toggle to backboard
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) toBackboard = !toBackboard;
            // Toggle to stack
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) toStack = !toStack;
            telemetry.addData("to backboard (Lstick)", toBackboard + " to stack (Rstick): " + toStack);

            // BUMPER FUNCTIONS
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                slidesHeight += slidesHeight < 1000 ? 200 : 0;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                slidesHeight -= slidesHeight > 0 ? 200 : 0;
            }
            telemetry.addData("slides height (bumpers)", slidesHeight);

            // Initiate color detection
            if (alliance == Alliance.RED)  {
                colorDetection.setAlliance(1);
            }
            else {
                colorDetection.setAlliance(2);
            }
            // spikeMark = colorDetection.getSpikeMark();
            // Manual change spike mark
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                spikeMark = 2;
                spikeMarkString = "CENTER";
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                spikeMark = 1;
                spikeMarkString = "LEFT";
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                spikeMark = 3;
                spikeMarkString = "RIGHT";
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
            trackHeading.start();
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
            if (alliance == Alliance.BLUE) {
                if (side == Side.CLOSE) { // BLUE Close side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(23, 45, Math.toRadians(90)))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(23, 33, Math.toRadians(60)))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13,58,Math.toRadians(90)))
                               // .splineToConstantHeading(new Vector2d(20, 40), Math.toRadians(225))
                                .splineToSplineHeading(new Pose2d(13, 33, Math.toRadians(0)), Math.toRadians(225))
                                .build(); break;
                    }
                } else { // BLUE Far side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-42, 46, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(-35, 34, Math.toRadians(180)), Math.toRadians(0))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-50, 22, Math.toRadians(180)))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-56, 32, Math.toRadians(180)))
                                .build(); break;
                    }
                }
            } else {
                if (side == Side.CLOSE) { // RED Close side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13,-58,Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(20, -40), Math.toRadians(225))
                                .splineToSplineHeading(new Pose2d(12, -33, Math.toRadians(0)), Math.toRadians(225))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(23, -32, Math.toRadians(60))) // To spike mark
                                .waitSeconds(1) // Place purple pixel
                                .lineToLinearHeading(new Pose2d(51, -35, Math.toRadians(180)))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate()).back(2)
                                .lineToLinearHeading(new Pose2d(17, -40, Math.toRadians(120)))
                                .build(); break;
                    }
                } else { // RED Far side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-47, -13, Math.toRadians(270)))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-50, -22, Math.toRadians(180)))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-42, -46, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(-35, -34, Math.toRadians(180)), Math.toRadians(0))
                                .build(); break;
                    }
                }
            }
            sleep((long) spikeDelay * 1000);
            bot.outtakeGround(); // Go to outtake ground before trajectory
            drive.followTrajectorySequence(spikeTrajectory);
            // Place purple pixel
            if(side == Side.CLOSE) {
                bot.claw.halfOpen();
            } else {
                bot.claw.fullOpen();
            }
            sleep(150);
            bot.storage();
            bot.claw.close();
            sleep((long) backboardDelay * 1000);

            // To backboard
            if (toBackboard) {
                int stackX = -56;
                // Extra movements on far side
                if (side == Side.FAR) {
                    // To pixel stack
                    TrajectorySequence stackTrajectory = null;
                    if (alliance == Alliance.BLUE) { // BLUE
                        switch (spikeMark) {
                            case 1: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-48, 16, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(stackX, 11), Math.toRadians(180),
                                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build(); break;
                            case 2: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(stackX, 11, Math.toRadians(180)),
                                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build(); break;
                            case 3: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(stackX+2, 11, Math.toRadians(180)),
                                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build(); break;
                        }
                    } else { // RED
                        switch (spikeMark) {
                            case 1: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(stackX, -11, Math.toRadians(180)))
                                    .build(); break;
                            case 2: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(stackX, -11, Math.toRadians(180)))
                                    .build(); break;
                            case 3: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-48, -16, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(stackX, -11), Math.toRadians(180))
                                    .build(); break;
                        }
                    }
                    bot.intake(false, 0.28);
                    drive.followTrajectorySequence(stackTrajectory);
                    // Intake from stack
                    sleep(800);
                    pixelTap.start();
                    bot.intake.setIntakeHeight(bot.intake.intakeStorage);
                    // Across the field
                    int farY = alliance == Alliance.RED ? -11 : 11;
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(25, farY, Math.toRadians(180)))
                            .build());
                    autoPickup.start();
                }

                // Define backboard y value
                int backboardX = 49, backboardY = 0;
                if (alliance == Alliance.RED) {
                    switch (spikeMark) {
                        case 1: backboardY = -30; break;
                        case 2: backboardY = -36; break;
                        case 3: backboardY = -41; break;
                    }
                } else {
                    switch (spikeMark) {
                        case 1: backboardY = 41; break;
                        case 2: backboardY = 36; break;
                        case 3: backboardY = 28; break;
                    }
                }
                Pose2d backboardPose = new Pose2d(backboardX+1, backboardY, Math.toRadians(180));
                if ((alliance == Alliance.BLUE && side == Side.CLOSE && spikeMark == 1) || (alliance == Alliance.RED) && side == Side.CLOSE && spikeMark == 3) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(backboardPose, Math.toRadians(0))
                            .build());
                } else {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(backboardPose)
                            .build());
                }
                // Place yellow pixel
                if (side == Side.CLOSE) {
                    bot.outtakeOut(1);
                    if (slidesHeight != 0) bot.slides.runTo(-slidesHeight);
                    else bot.slides.runToBottom();
                    sleep(750);
                    bot.claw.open();
                    sleep(100);
                    bot.storage();
                } else { // Place both white and yellow
                    bot.intake.stopIntake();
                    bot.outtakeOut(2);
                    if (slidesHeight != 0) bot.slides.runTo(-slidesHeight);
                    else bot.slides.runToBottom();
                    sleep(750);
                    bot.claw.open();
                    sleep(500);
                    bot.fourbar.setArm(0.65);
                    bot.fourbar.setWrist(0.72);
                    bot.fourbar.topOuttake(false);
                    bot.slides.runTo(-800);
                    sleep(500);
                    bot.claw.open();
                    sleep(500);
                    bot.storage();
                }

                // Pixel stack trajectory starts here
                double [] stackHeights = new double [] { // from top pixel (1) to bottom pixel (5)
                        0.27,
                        0.29,
                        0.32,
                        0.34,
                        0.38
                };
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
                    for (int i = 0; i < stackIterations; i++) {
                        Thread extendIntake = new Thread(() -> {
                            sleep(1200);
                            if (side == Side.CLOSE) {
                                bot.intake(false, stackHeights[0]);
                            } else if (side == Side.FAR) {
                                bot.intake(false, stackHeights[1]);
                            }
                        });
                        Thread extendIntake2 = new Thread(() -> {
                            sleep(1200);
                            if (side == Side.CLOSE) {
                                bot.intake(false, stackHeights[2]);
                            } else if (side == Side.FAR) {
                                bot.intake(false, stackHeights[3]);
                            }
                        });
                        if (i == 0) {
                            extendIntake.start();
                        } else {
                            extendIntake2.start();
                        }

                        // To pixel stack
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(30, stackY1, Math.toRadians(180)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(stackX+10, stackY1, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(stackX, stackY1, Math.toRadians(180)))
                                .build());
                        // Intake from stack
                        if (side == Side.CLOSE) {
                            if (stackIterations == 1) {
                                sleep(150);
                                bot.intake(false, stackHeights[1]);
                                sleep(800);
                                if (breakBeam.getState()) {
                                    bot.intake(true, stackHeights[2]);
                                    sleep(150);
                                    bot.intake(false, stackHeights[2]);
                                    sleep(1600);
                                }
                            } else if (stackIterations == 2) {
                                if (i == 0) {
                                    sleep(150);
                                    bot.intake(false, stackHeights[1]);
                                    sleep(800);
                                    if (breakBeam.getState()) {
                                        bot.intake(true, stackHeights[2]);
                                        sleep(300);
                                        bot.intake(false, stackHeights[2]);
                                        sleep(1600);
                                    }
                                } else {
                                    bot.intake(false, stackHeights[3]);
                                    sleep(800);
                                    if (breakBeam.getState()) {
                                        bot.intake(true, stackHeights[4]);
                                        sleep(300);
                                        bot.intake(false, stackHeights[4]);
                                        sleep(1200);
                                    }
                                }
                            }
                        } else if (side == Side.FAR) {
                            if (stackIterations == 1) {
                                bot.intake(false, stackHeights[1]);
                                sleep(800);
                                bot.intake(false, stackHeights[2]);
                                sleep(800);
                                if (breakBeam.getState()) {
                                    bot.intake(true, stackHeights[2]);
                                    sleep(300);
                                    bot.intake(false, stackHeights[2]);
                                    sleep(1200);
                                }
                            } else if (stackIterations == 2) {
                                if (i == 0) {
                                    bot.intake(false, stackHeights[1]);
                                    sleep(800);
                                    bot.intake(false, stackHeights[2]);
                                    sleep(800);
                                    if (breakBeam.getState()) {
                                        bot.intake(true, stackHeights[2]);
                                        sleep(300);
                                        bot.intake(false, stackHeights[2]);
                                        sleep(1200);
                                    }
                                } else {
                                    bot.intake(false, stackHeights[3]);
                                    sleep(800);
                                    bot.intake(false, stackHeights[4]);
                                    sleep(800);
                                    if (breakBeam.getState()) {
                                        bot.intake(true, stackHeights[4]);
                                        sleep(300);
                                        bot.intake(false, stackHeights[4]);
                                        sleep(1200);
                                    }
                                }
                            }
                        }


                        bot.intake(true, bot.intake.intakeUp);
                        pixelTap.start();

                        // Across field
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(38, stackY1, Math.toRadians(180)))
                                .build());
                        autoPickup.start();
                        // To backboard
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(backboardX, stackY2, Math.toRadians(180)), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(13, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());

                        // Score pixels on backboard
                        bot.intake.stopIntake();
                        bot.outtakeOut(2);
                        if (slidesHeight != 0) bot.slides.runTo(-slidesHeight);
                        else bot.slides.runToBottom();
                        sleep(500);
                        bot.claw.open();
                        sleep(500);
                        bot.fourbar.setArm(bot.fourbar.armTopOuttake);
                        bot.fourbar.setWrist(bot.fourbar.wristTopOuttake);
                        bot.fourbar.topOuttake(false);
                        bot.slides.runTo(-800);
                        sleep(500);
                        bot.claw.open();
                        sleep(500);
                        bot.storage();
                    }
                }

                // Parking trajectory
                if (park != 0) {
                    int parkY = alliance == Alliance.RED ? (park == 1 ? -10 : -58) : (park == 1 ? 58 : 10);
                    // To park
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(50, parkY, Math.toRadians(180)),Math.toRadians(15))
                            .build());
                }
            }

            // Stop op mode
            bot.storage();
            sleep(1000);
            PoseStorage.currentPose = drive.getPoseEstimate();
            requestOpModeStop();
        }

        //periodic.interrupt();
        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) {
            telemetry.addLine("Exception as follows: " + e);
        }
    }
}