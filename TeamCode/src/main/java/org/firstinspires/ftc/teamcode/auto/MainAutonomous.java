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
    boolean toPark = false; // Parking position: true - park in corner after, false - don't move after scoring pixel
    boolean toBackboard = true; // Go to backboard: true - go to backboard and score, false - only score spike and stop
    boolean slidesUp = false; // Slide up: true - move slides up when scoring pixel on backboard, false - don't
    int backboardWait = 0; // How long (milliseconds) to wait before scoring on backboard: 0-5 seconds
    int dt = 0;
    // TODO: WRITE SCORE SPIKE CONDITION
    boolean scoreSpike = true; // Score spike: true - score spike, false - only park

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
            bot.claw.halfOpen();
            sleep(100);
            bot.fourbar.topPixel();
            sleep(800);
            bot.claw.fullClose();
            bot.calculateWristPos();
            bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
            sleep(300);
            bot.storage();
            sleep(200);
        });
        pickup.start();

        Thread update = new Thread(() -> {
            sleep(1000);
            dt++;
        });

        int spikeMark = 0;

        // Initialized, adjust values before start
        /*
        LIST OF CONFIG CONTROLS (so far) - Zachery:
        Driver 1 (gp1):
        Y - change alliance
        A - change side
        B - toggle slides up
        X - toggle park
        dpad up - change backboard wait time (increment by 1 second)
        dpad down - toggle to backboard
        START - re-pickup top pixel
         */
        while (!isStarted()) {
            gp1.readButtons();

            // Change alliance
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                if (alliance == Alliance.RED) alliance = Alliance.BLUE;
                else alliance = Alliance.RED;
            }
            telemetry.addData("Alliance (Y)", alliance);

            // Re-grip/pick up pixel
            if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                bot.claw.halfOpen();
                sleep(500);
                bot.fourbar.topPixel();
                sleep(800);
                bot.claw.fullClose();
                bot.calculateWristPos();
                bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
                sleep(300);
                bot.storage();
                sleep(200);
            }

            // Change side
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (side == Side.CLOSE) { // Rn close, switch to FAR
                    side = Side.FAR;
                    slidesUp = true;
                    toPark = false;
                } else { // Rn far, switch to CLOSE
                    side = Side.CLOSE;
                    slidesUp = false;
                    toPark = true;
                }
            }
            telemetry.addData("Side (A)", side);

            // Toggle slides up
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                slidesUp = !slidesUp;
            }
            telemetry.addData("Slides Up (B)", slidesUp);

            // Toggle park
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                toPark = !toPark;
            }
            telemetry.addData("Parking (X)", toPark);

            // Change backboard wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                if (backboardWait < 5000) backboardWait+=1000;
                else backboardWait = 0;
            }
            telemetry.addData("Backboard sleep (DPAD UP)", backboardWait + " milliseconds");

            // Toggle park
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                toBackboard = !toBackboard;
            }
            telemetry.addData("To Backboard (DPAD DOWN)", toBackboard);

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
                                .back(7)
                                .forward(7)
                                .build());
                    break;
                case 2: // MIDDLE
                    drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(startPose)
                            .back(47)
                            .forward(20)
                            .build());
                    break;
                case 3: // RIGHT
                    drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(startPose)
                            .back(27)
                            .turn(Math.toRadians(-90))
                            .back(7)
                            .forward(7)
                            .build());
                    break;
            }

            // Outtake purple/top pixel
            bot.outtakeGround();
            sleep(1000);
            bot.claw.halfOpen();
            sleep(600);
            bot.storage();
            bot.claw.fullClose();
            bot.calculateWristPos();
            bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
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
                                                .back(78)
                                                .strafeRight(4)
                                                .build());
                                break;
                            case 2: // MIDDLE
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeLeft(12)
                                                .back(24)
                                                .turn(Math.toRadians(turnRadians))
                                                .back(90)
                                                .strafeRight(5)
                                                .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeRight(24)
                                                .forward(78)
                                                .turn(Math.toRadians((2*turnRadians)))
                                                .strafeRight(6)
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
                                                .turn(Math.toRadians((turnRadians)))
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
                                                .forward(78)
                                                .turn(Math.toRadians(-2*turnRadians))
                                                .strafeLeft(10)
                                                .build());
                                break;
                            case 2: // MIDDLE
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeRight(12)
                                                .back(24)
                                                .turn(Math.toRadians(-turnRadians))
                                                .back(90)
                                                .strafeLeft(10)
                                                .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(
                                        drive.trajectorySequenceBuilder(startPose)
                                                .strafeRight(24)
                                                .back(78)
                                                .strafeLeft(10)
                                                .build());
                                break;
                        }
                    }
                }

                // Backing up to backboard position
                if (side == Side.CLOSE) {
                    startPose = drive.getPoseEstimate();
                    int backboardDrive = 34;
                    if (alliance == Alliance.BLUE){
                        switch (spikeMark) {
                            case 1: backboardDrive = 29; break; // LEFT
                            case 2: backboardDrive = 33; break; // MIDDLE,
                            case 3: backboardDrive = 37; break; // RIGHT
                        }
                    } else {
                        switch (spikeMark) {
                            case 1: backboardDrive = 37; break; // LEFT
                            case 2: backboardDrive = 33; break; // MIDDLE,
                            case 3: backboardDrive = 29; break; // RIGHT
                        }
                    }
                    drive.followTrajectory(drive.trajectoryBuilder(startPose).back(backboardDrive).build());
                }

                sleep(backboardWait); // How long to wait before strafing to score

                // Scoring on backboard
                startPose = drive.getPoseEstimate();
                int scoreStrafe = 0;
                if ((alliance == Alliance.BLUE && side == Side.CLOSE) || (alliance == Alliance.RED && side == Side.FAR)) {
                    switch (spikeMark) {
                        case 1: scoreStrafe = 13; break; // LEFT, close
                        case 2: scoreStrafe = 18; break; // MIDDLE,
                        case 3: scoreStrafe = 26; break; // RIGHT
                    }
                } else if ((alliance == Alliance.BLUE && side == Side.FAR) || (alliance == Alliance.RED && side == Side.CLOSE)) {
                    switch (spikeMark) {
                        case 1: scoreStrafe = 14; break; // LEFT, close
                        case 2: scoreStrafe = 19; break; // MIDDLE,
                        case 3: scoreStrafe = 27; break; // RIGHT
                    }
                }
                if ((alliance == Alliance.BLUE && side == Side.CLOSE) || (alliance == Alliance.RED && side == Side.FAR)) { // BLUE SIDE, strafe right
                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(startPose)
                                    //.turn(Math.toRadians(90))
                                    .strafeLeft(scoreStrafe)
                                    .build());
                } else if ((alliance == Alliance.BLUE && side == Side.FAR) || (alliance == Alliance.RED && side == Side.CLOSE)) { // RED SIDE, strafe left
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
                    bot.claw.halfOpen();
                    sleep(100);
                    bot.fourbar.bottomPixel();
                    sleep(500);
                    bot.claw.fullClose();
                    bot.calculateWristPos();
                    bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
                    sleep(800);
                    bot.storage();
                    sleep(200);
                });
                startPose = drive.getPoseEstimate();
                pickupYellow.start();

                // Run into backboard
                int slowerVelocity = 10; // Slower velocity that is the max constraint when running into backboard (in/s)
                drive.followTrajectory(drive.trajectoryBuilder(startPose).back(13,
                                SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build());

                sleep(200);

                // Place yellow/bottom pixel on backboard
                if (slidesUp) { // Slides run up
                    bot.slides.runTo(-400.0);
                } else {
                    bot.slides.runToBottom();
                }
                bot.outtakeOut();
                sleep(500);
                bot.claw.halfOpen();
                sleep(800);
                bot.storage();
                bot.claw.fullClose();
                bot.calculateWristPos();
                bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
                sleep(200);

                // Parking
                startPose = drive.getPoseEstimate();
                int parkStrafe = 0;
                switch (spikeMark) {
                    case 1: parkStrafe = 17; break;
                    case 2: parkStrafe = 23; break;
                    case 3: parkStrafe = 31; break;
                }
                if (toPark) {
                    drive.followTrajectory(drive.trajectoryBuilder(startPose).forward(7).build());
                    if (alliance == Alliance.BLUE) {
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(parkStrafe).build());
                    } else {
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(parkStrafe).build());
                    }
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(7).build());
                }
            }

            // Stop op mode
            sleep(800);
            requestOpModeStop();
        }

        // Run update dt during op mode in the background
        if (opModeIsActive() && !isStopRequested()) {
            update.start();
        }

        //periodic.interrupt();
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
