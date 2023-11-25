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
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.auto.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.*;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import java.util.Objects;

@Config
@Autonomous(name = "MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    Bot bot;

    // Side - how close to backboard: LEFT - furthest, RIGHT - closest
    public enum Side {
        CLOSE, FAR, NULL;
    }
    Side side = Side.CLOSE;

    // Alliance
    public enum Alliance {
        RED, BLUE, NULL;
    }
    Alliance alliance = Alliance.BLUE;

    // Autonomous config values
    boolean park = false; // Parking position: false - don't move, true - park in corner
    boolean toBackboard = true; // Go to backboard: true, false`

    // TODO: TUNE THESE APRIL TAG VALUES TO FIT WITH APRIL TAGS
    static final double FEET_PER_METER = 3.28084;
    double fx = 1078.03779, fy = 1084.50988, cx = 580.850545, cy = 245.959325;
    double tagsize = 0.032; // UNITS ARE METERS //ONLY FOR TESTIN
    int ID_ONE = 1, ID_TWO = 2, ID_THREE = 3; // Tag ID 1,2,3 from the 36h11 family
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(true);
        bot = Bot.getInstance(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        // Define camera values
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
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

        // Define starting pose for robot
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // Threads
        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
            }
        });

        // Initialize bot
        bot.stopMotors();
        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        // Pickup top
        Thread pickup = new Thread(() -> {
            sleep(250);
            bot.slides.runToBottom();
            bot.claw.open();
            sleep(100);
            bot.fourbar.topPixel();
            sleep(800);
            bot.claw.close();
            sleep(300);
            bot.storage();
            sleep(200);
        });
        pickup.start();

        int spikeMark = 0;

        // Initialized, adjust values before start
        /*
        LIST OF CONFIG CONTROLS (so far) - Zachery:
        Driver 1 (gp1):
        Y - change alliance
        A - change side
        X - toggle run to backboard
        B - toggle park
         */
        while (!isStarted()) {
            gp1.readButtons();

            // Change alliance
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                if (alliance == Alliance.RED) alliance = Alliance.BLUE;
                else alliance = Alliance.RED;
            }
            telemetry.addData("Alliance", alliance);

            // Change side
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (side == Side.CLOSE) {
                    side = Side.FAR;
                    park = false;
                } else {
                    side = Side.CLOSE;
                    park = true;
                }
            }
            telemetry.addData("Side", side);

            // Toggle run to backboard
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                if (toBackboard) {
                    toBackboard = false;
                    park = false;
                } else {
                    toBackboard = true;
                }
            }
            telemetry.addData("To Backboard", toBackboard);

            // Toggle park
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                park = !park;
            }
            telemetry.addData("Parking", park);

            // Initiate color detection
            if (alliance == Alliance.RED) {
                colorDetection.setAlliance(1);
            } else { // Automatically checks for blue if no alliance is set
                colorDetection.setAlliance(2);
            }
            telemetry.addData("Current Camera FPS", camera.getFps());

            spikeMark = colorDetection.getSpikeMark();
            String spikeMarkString = "";
            switch (spikeMark) {
                case 1: spikeMarkString = "Left"; break;
                case 2: spikeMarkString = "Middle"; break;
                case 3: spikeMarkString = "Right"; break;
                default: spikeMarkString = "None"; break;
            }
            telemetry.addData("Detected Spike", spikeMarkString);

            // Update
            telemetry.update();
            sleep(20);
        }

        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) { }

        waitForStart();

        // TODO: TUNE/ADJUST TRAJECTORY PATHS (i definitely know what i'm doing)
        // Auto start
        if (opModeIsActive() && !isStopRequested()) {
            periodic.start();
            startPose = drive.getPoseEstimate();

            // Spike mark trajectory
            switch(spikeMark) {
                case 1: // LEFT
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(startPose)
                                .back(28)
                                .turn(Math.toRadians(90))
                                .back(6)
                                .forward(4)
                                .build());
                    break;
                case 2: // MIDDLE
                    drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(startPose)
                            .back(30)
                            .forward(4)
                            .build());
                    break;
                case 3: // RIGHT
                    drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(startPose)
                            .back(28)
                            .turn(Math.toRadians(-90))
                            .back(6)
                            .forward(4)
                            .build());
                    break;
            }

            // Outtake purple/top pixel
            bot.outtakeGround();
            sleep(1000);
            bot.claw.open();
            sleep(600);
            bot.storage();
            bot.claw.close();
            sleep(200);

            // Backstage actions
            startPose = drive.getPoseEstimate();
            if (toBackboard) {
                int turnRadians = 90;
                // Run to backboard
                if (alliance == Alliance.BLUE) { // BLUE ALLIANCE
                    if (side == Side.CLOSE) { // closer to backboard
                        switch (spikeMark) {
                            case 1: // LEFT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeRight(24)
                                                .build());
                                break;
                            case 2: // MIDDLE
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .forward(22)
                                                .turn(Math.toRadians((turnRadians)))
                                                .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeLeft(24)
                                                .turn(Math.toRadians((2*turnRadians)))
                                                .build());
                                break;
                        }
                    } else { // further from backboard, go through middle truss
                        switch (spikeMark) {
                            case 1: // LEFT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeLeft(24)
                                                .back(72)
                                                .build());
                                break;
                            case 2: // MIDDLE NEED TO WRITE
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeLeft(12)
                                                .back(24)
                                                .strafeRight(84)
                                                .turn(Math.toRadians(turnRadians))
                                                .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeRight(24)
                                                .forward(72)
                                                .turn(Math.toRadians((2*turnRadians)))
                                                .build());
                                break;
                        }
                    }
                } else { // RED ALLIANCE
                    if (side == Side.CLOSE) { // closer to backboard
                        switch (spikeMark) {
                            case 1: // LEFT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeRight(24)
                                                .turn(Math.toRadians(-2*turnRadians))
                                                .build());
                                break;
                            case 2: // MIDDLE
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .forward(22)
                                                .turn(Math.toRadians((-turnRadians)))
                                                .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeLeft(24)
                                                .build());
                                break;
                        }
                    } else { // further from backboard, go through middle truss NEED TO TUNE
                        switch (spikeMark) {
                            case 1: // LEFT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeLeft(24)
                                                .forward(72)
                                                .turn(Math.toRadians(-2*turnRadians))
                                                .build());
                                break;
                            case 2: // MIDDLE
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeRight(12)
                                                .back(24)
                                                .strafeLeft(84)
                                                .turn(Math.toRadians(-turnRadians))
                                                .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeRight(24)
                                                .back(72)
                                                .build());
                                break;
                        }
                    }
                }

                // Backing up to backboard position
                startPose = drive.getPoseEstimate();
                int backboardDrive = 34;
                if ((alliance == Alliance.BLUE && side == Side.CLOSE) || (alliance == Alliance.RED && side == Side.FAR)){
                    switch (spikeMark) {
                        case 1: backboardDrive = 29; break; // LEFT
                        case 2: backboardDrive = 33; break; // MIDDLE,
                        case 3: backboardDrive = 37; break; // RIGHT
                    }
                } else if ((alliance == Alliance.BLUE && side == Side.FAR) || (alliance == Alliance.RED && side == Side.CLOSE)) {
                    switch (spikeMark) {
                        case 1: backboardDrive = 37; break; // LEFT
                        case 2: backboardDrive = 33; break; // MIDDLE,
                        case 3: backboardDrive = 29; break; // RIGHT
                    }
                }
                if (side == Side.FAR) {
                    backboardDrive = backboardDrive + 48;
                }

                drive.followTrajectory(drive.trajectoryBuilder(startPose).back(backboardDrive).build());

                // Scoring on backboard
                startPose = drive.getPoseEstimate();
                int scoreStrafe = 0;
                if (alliance == Alliance.BLUE) {
                    switch (spikeMark) {
                        case 1: scoreStrafe = 13; break; // LEFT, close
                        case 2: scoreStrafe = 19; break; // MIDDLE,
                        case 3: scoreStrafe = 25; break; // RIGHT
                    }
                } else {
                    switch (spikeMark) {
                        case 1: scoreStrafe = 13; break; // LEFT, close
                        case 2: scoreStrafe = 19; break; // MIDDLE,
                        case 3: scoreStrafe = 25; break; // RIGHT
                    }
                }

                if (alliance == Alliance.BLUE) { // BLUE SIDE, strafe right
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(startPose)
                                    //.turn(Math.toRadians(90))
                                    .strafeLeft(scoreStrafe)
                                    .build());
                } else { // RED SIDE, strafe left
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(startPose)
                                    //.turn(Math.toRadians(-90))
                                    .strafeRight(scoreStrafe)
                                    .build());
                }

                // Pickup yellow/bottom pixel
                Thread pickupYellow = new Thread(() ->{
                    // pickup
                    bot.slides.runToBottom();
                    bot.claw.open();
                    sleep(100);
                    bot.fourbar.bottomPixel();
                    sleep(500);
                    bot.claw.close();
                    sleep(800);
                    bot.storage();
                    sleep(200);
                });
                pickupYellow.start();
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(13).build()); // run into backboard

                sleep(800);
                //bot.slides.runTo(200);
                // Place yellow/bottom pixel on backboard
                bot.outtakeOut();
                sleep(500);
                bot.claw.open();
                sleep(800);
                bot.storage();
                bot.claw.close();
                sleep(200);

                // Parking
                startPose = drive.getPoseEstimate();
                int parkStrafe = 0;
                if (alliance == Alliance.BLUE) {
                    switch (spikeMark) {
                        case 1: parkStrafe = 18; break;
                        case 2: parkStrafe = 25; break;
                        case 3: parkStrafe = 32; break;
                    }
                } else {
                    switch (spikeMark) {
                        case 1: parkStrafe = 32; break;
                        case 2: parkStrafe = 25; break;
                        case 3: parkStrafe = 18; break;
                    }
                }
                if (park) {
                    drive.followTrajectory(drive.trajectoryBuilder(startPose).forward(9).build());
                    if (alliance == Alliance.BLUE) {
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(parkStrafe).build());
                    } else {
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(parkStrafe).build());
                    }
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(9).build());
                }
            }

            // Stop op mode
            bot.setAutoEndPose(drive.getPoseEstimate());
            sleep(800);
            requestOpModeStop();
        }
        periodic.interrupt();
        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) {
            telemetry.addLine("Exception as followsL: " + e);
        }
    }

    // April tag detection telemetry
    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("Detected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.x)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.y)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.z)));
    }

}

//            // This code should be inside the runOpMode() loop
//            // Run to backboard spline
//            Vector2d scoreBlue = new Vector2d(42,30), scoreRed = new Vector2d(42,-30); // Vector2d spline end positions (backboard)
//            Trajectory backboard;
//            if (alliance == Alliance.RED) {
//                backboard = drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .splineTo(scoreRed, Math.toRadians(0))
//                        .build();
//            } else {
//                backboard = drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .splineTo(scoreBlue, Math.toRadians(0))
//                        .build();
//            }
//            drive.followTrajectory(backboard);

//            // Set april tag pipeline
//            camera.setPipeline(aprilTagDetectionPipeline);
//
//            // Strafing along backboard trajectory
//            Trajectory strafe;
//            if (alliance == Alliance.RED) {
//                strafe = drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .strafeLeft(26)
//                        .build();
//            } else { // alliance == Alliance.BLUE
//                strafe = drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .strafeRight(26)
//                        .build();
//            }
//            drive.followTrajectoryAsync(strafe);
//
//            // Repeatedly detecting april tag
//            while (strafe.duration() > 6.0 && strafe.duration() < 8.0) { // TODO: tune this time frame to be accurate (if it even works lmao)
//                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//                if (currentDetections.size() != 0) {
//                    boolean tagFound = false;
//                    for (AprilTagDetection tag : currentDetections) {
//                        if (tag.id == ID_ONE || tag.id == ID_TWO || tag.id == ID_THREE) {
//                            tagOfInterest = tag;
//                            tagFound = true;
//                            strafe.end();
//                            break;
//                        }
//                    }
//                    if (tagFound) {
//                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                        tagToTelemetry(tagOfInterest);
//                    } else {
//                        telemetry.addLine("Don't see tag of interest :(");
//                        if (tagOfInterest == null) {
//                            telemetry.addLine("(The tag has never been seen)");
//                        } else {
//                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                            tagToTelemetry(tagOfInterest);
//                        }
//                    }
//                } else {
//                    telemetry.addLine("Don't see tag of interest :(");
//                    if (tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//            }
