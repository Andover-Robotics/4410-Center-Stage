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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    DistanceSensor frontDistanceSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        // Instantiate
        telemetry.setAutoClear(true);
        bot = Bot.getInstance(this);
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistance");
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

        // Pick up in auto
        Thread autoPickup2 = new Thread(() -> {
            bot.intake.stopIntake();
            bot.slides.runToBottom();
            bot.claw.fullOpen();
            sleep(100);
            bot.fourbar.pickup();
            sleep(250);
            bot.claw.pickupClose();
            sleep(300);
            bot.storage();
            sleep(250);
            bot.intake.stopIntake();
            bot.autoOuttakeOut(2);
            bot.slides.runTo(-700);
        });

        Thread autoPickup1 = new Thread(() -> {
            bot.intake.stopIntake();
            bot.slides.runToBottom();
            bot.claw.fullOpen();
            sleep(100);
            bot.fourbar.pickup();
            sleep(250);
            bot.claw.pickupClose();
            sleep(300);
            bot.storage();
            sleep(250);
            bot.intake.stopIntake();
            bot.autoOuttakeOut(2);
            bot.slides.runTo(-200);
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
            spikeMark = colorDetection.getSpikeMark();
            switch (spikeMark) {
                case 1: spikeMarkString = "LEFT"; break;
                case 2: spikeMarkString = "CENTER"; break;
                case 3: spikeMarkString = "RIGHT"; break;
                default: spikeMarkString = "NONE"; break;
            }
//            // Manual change spike mark
//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                spikeMark = 2;
//                spikeMarkString = "CENTER";
//            }
//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//                spikeMark = 1;
//                spikeMarkString = "LEFT";
//            }
//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
//                spikeMark = 3;
//                spikeMarkString = "RIGHT";
//            }
            telemetry.addData("detected spike", spikeMarkString);

            // Update
            telemetry.update();
            sleep(20);
        }
        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) { }

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

            // Pixel stack trajectory starts here
            double [] stackHeights = new double [] { // from top pixel (1) to bottom pixel (5)
                    0.24,
                    0.26,
                    0.285,
                    0.31,
                    0.34
            };

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
                                .lineToLinearHeading(new Pose2d(23, 32, Math.toRadians(60)))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(13,58,Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(12, 31, Math.toRadians(0)), Math.toRadians(225))
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
                                .lineToSplineHeading(new Pose2d(13,-58,Math.toRadians(-90)))
                                .splineToSplineHeading(new Pose2d(12, -31, Math.toRadians(0)), Math.toRadians(-225))
                                .build(); break;
                        case 2: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(23, -32, Math.toRadians(-60)))
                                .build(); break;
                        case 3: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(23, -45, Math.toRadians(-90)))
                                .build(); break;
                    }
                } else { // RED Far side
                    switch (spikeMark) {
                        case 1: spikeTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-56, -32, Math.toRadians(180)))
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
            sleep(100);
            bot.storage();
            bot.claw.close();
            sleep((long) backboardDelay * 1000);

            Thread outtake = new Thread(() -> {
                sleep(1000);
                bot.autoOuttakeOut(1);
                if (slidesHeight != 0) bot.slides.runTo(-slidesHeight);
                else bot.slides.runToBottom();
            });
            if (side == Side.CLOSE) {
                outtake.start();
            }

            // To backboard
            if (toBackboard) {
                int stackX = -59;
                // Extra movements on far side
                if (side == Side.FAR) {
                    // To pixel stack
                    TrajectorySequence stackTrajectory = null;
                    if (alliance == Alliance.BLUE) { // BLUE
                        switch (spikeMark) {
                            case 1: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-48, 16, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(stackX, 11), Math.toRadians(180),
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(16))
                                    .build(); break;
                            case 2: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(stackX, 11, Math.toRadians(180)),
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(16))
                                    .build(); break;
                            case 3: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(-57, 24, Math.toRadians(180)),
//                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(16))
                                    .lineToLinearHeading(new Pose2d(stackX, 11, Math.toRadians(180)),
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(16))
                                    .build(); break;
                        }
                    } else { // RED
                        switch (spikeMark) {
                            case 1: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(stackX, -11, Math.toRadians(180)),
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(16))
                                    .build(); break;
                            case 2: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(stackX, -11, Math.toRadians(180)),
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(16))
                                    .build(); break;
                            case 3: stackTrajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-48, -16, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(stackX, -11), Math.toRadians(180),
                                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(16))
                                    .build(); break;
                        }
                    }

                    Thread farIntake = new Thread(() -> {
                        sleep(1300);
                        bot.intake(false, stackHeights[0]);
                    });
                    farIntake.start();
                    drive.followTrajectorySequence(stackTrajectory);
                    // Intake from stack
                    int counter = 0, breakBeamCounter = 0;
                    do {
                        sleep(5);
                        counter+=5;
                        if (!breakBeam.getState()) {
                            breakBeamCounter++;
                        }
                    } while(counter < 1500 && breakBeamCounter < 2);
                    bot.autoFixPixels();
                    bot.intake(true, 0.1);
                    // Across the field
                    int farY = alliance == Alliance.RED ? -11 : 11;
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(25, farY, Math.toRadians(180)))
                            .build());
                    autoPickup1.start();
                }

                // Define backboard y value
                int backboardX = 48, backboardY = 0;
                if (alliance == Alliance.RED) {
                    switch (spikeMark) {
                        case 1: backboardY = -28; break;
                        case 2: backboardY = -34; break;
                        case 3: backboardY = -39; break;
                    }
                } else {
                    switch (spikeMark) {
                        case 1: backboardY = 39; break;
                        case 2: backboardY = 34; break;
                        case 3: backboardY = 28; break;
                    }
                }
                Pose2d backboardPose = new Pose2d(backboardX, backboardY, Math.toRadians(180));
                if (alliance == Alliance.BLUE && side == Side.CLOSE && spikeMark == 1) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(28, 50, Math.toRadians(180)))
                            .lineToLinearHeading(new Pose2d(backboardX+1, backboardY, Math.toRadians(180)))
                            .build());
                } else if (alliance == Alliance.RED && side == Side.CLOSE && spikeMark == 3) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(28, -50, Math.toRadians(180)))
                            .lineToLinearHeading(new Pose2d(backboardX+1, backboardY, Math.toRadians(180)))
                            .build());
                } else {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(backboardPose)
                            .build());
                }
                // Place yellow pixel
                if (side == Side.CLOSE) {
                    sleep(100);
                    bot.claw.open();
                    sleep(100);
                    bot.storage();
                } else { // Place both white and yellow
                    bot.intake.stopIntake();
                    bot.claw.open();
                    sleep(400);
                    bot.fourbar.setArm(0.65);
                    bot.fourbar.setWrist(0.72);
                    sleep(350);
                    bot.autoOuttakeOut(1);
                    bot.slides.runTo(-800);
                    sleep(300);
                    bot.claw.open();
                    sleep(300);
                    bot.storage();
                }


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
                            sleep(1600);
                            if (side == Side.CLOSE) {
                                bot.intake(false, stackHeights[0]);
                            } else if (side == Side.FAR) {
                                bot.intake(false, stackHeights[1]);
                            }
                        });
                        Thread extendIntake2 = new Thread(() -> {
                            sleep(1600);
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
//                                .lineToLinearHeading(new Pose2d(15, stackY1, Math.toRadians(180)))
//                                //.lineToLinearHeading(new Pose2d(stackX+10, stackY1, Math.toRadians(180))) dont use
//                                .lineToLinearHeading(new Pose2d(stackX, stackY1, Math.toRadians(180)),
//                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .lineToLinearHeading(new Pose2d(30, stackY1, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d( stackX+8, stackY1, Math.toRadians(180)))
                                .build());

                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(frontDistanceSensor.getDistance(DistanceUnit.INCH) - 5.25,
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(16))
                                .build());
                        // Intake from stack
                        if (stackIterations == 1 || (stackIterations == 2 && i == 0)) {
                            sleep(150);
                            if (side == Side.CLOSE) {
                                bot.intake(false, stackHeights[0]);
                            } else {
                                bot.intake(false, stackHeights[1]);
                            }
                            sleep(10);
                            if (side == Side.CLOSE) {
                                bot.intake(false, stackHeights[1]);
                            } else {
                                bot.intake(false, stackHeights[2]);
                            }
                            int counter = 0, breakBeamCounter = 0;
                            do {
                                sleep(5);
                                counter+=5;
                                if (!breakBeam.getState()) {
                                    breakBeamCounter++;
                                }
                            } while(counter < 1500 && breakBeamCounter < 6);

                        } else {
                            sleep(150);
                            if (side == Side.CLOSE) {
                                bot.intake(false, stackHeights[2]);
                            } else {
                                bot.intake(false, stackHeights[3]);
                            }
                            sleep(10);
                            if (side == Side.CLOSE) {
                                bot.intake(false, stackHeights[3]);
                            } else {
                                bot.intake(false, stackHeights[4]);
                            }
                            int counter = 0, breakBeamCounter = 0;
                            do {
                                sleep(5);
                                counter+=5;
                                if (!breakBeam.getState()) {
                                    breakBeamCounter++;
                                }
                            } while(counter < 1500 && breakBeamCounter < 6);
                        }


                        bot.intake(true, bot.intake.intakeUp);
                        bot.autoFixPixels();

                        // Across field
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(38, stackY1, Math.toRadians(180)))
                                .build());
                        autoPickup2.start();
                        // To backboard
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(backboardX, stackY2, Math.toRadians(180)), Math.toRadians(0))
                                .build());


                        // Score pixels on backboard
                        bot.claw.open();
                        sleep(500);
                        bot.autoOuttakeOut(1);
                        sleep(300);
                        bot.claw.open();
                        sleep(300);
                        bot.storage();
                    }
                }

                // Parking trajectory
                if (park != 0) {
                    int parkY = alliance == Alliance.RED ? (park == 1 ? -9 : -59) : (park == 1 ? 59 : 10);
                    // To park
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(50, parkY, Math.toRadians(180)),Math.toRadians(25))
                            .build());
                }
            }

            // Stop op mode
            bot.storage();
            sleep(1000);
            PoseStorage.currentPose = drive.getPoseEstimate();
            requestOpModeStop();
        }
        trackTime.interrupt();
        trackHeading.interrupt();
        periodic.interrupt();

//        periodic.interrupt();
//        try {
//            camera.stopStreaming();
//            camera.closeCameraDevice();
//        } catch (OpenCvCameraException e) {
//            telemetry.addLine("Exception as follows: " + e);
//        }
    }
}