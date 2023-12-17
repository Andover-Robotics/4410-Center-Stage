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
    boolean toBackboard = true; // Go to backboard: true - go to backboard and score, false - only score spike and stop
    boolean slidesUp = false; // Slide up: true - move slides up when scoring pixel on backboard, false - don't
    boolean pixelStack = true; // Go to pixel stack: true - do 2+2, false - park/or stop after scoring yellow pixel
    boolean centerTruss = true; // Middle truss go under: true - center truss, false - side truss
    int park = 0; // Parking position: 0 - don't park, 1 - left, 2 - right
    int newTiles = 0;
    double  backboardWait = 0; // How long (seconds) to wait before scoring on backboard: 0-10 seconds
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

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        // Pickup top
        Thread pickup = new Thread(() -> {
            sleep(500);
            bot.slides.runToBottom();
            bot.claw.fullOpen();
            sleep(100);
            bot.fourbar.pickup();
            sleep(400);
            bot.claw.pickupClose();
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
        B - toggle slides up
        X - change park
        dpad up - change backboard wait time (increment by 1 second)
        dpad down - toggle to backboard
        START - re-pickup pixel
        BACK - change pixel stack parameters
         */
        while (!isStarted()) {
            gp1.readButtons();

            // Change alliance
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                if (alliance == Alliance.RED)  {
                    alliance = Alliance.BLUE;
                } else  {
                    alliance = Alliance.RED;
                }
            }
            telemetry.addData("Alliance (Y)", alliance);

            // Re-grip/pick up pixel
            if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                bot.claw.fullOpen();
                sleep(500);
                bot.slides.runToBottom();
                bot.claw.fullOpen();
                sleep(100);
                bot.fourbar.pickup();
                sleep(400);
                bot.claw.pickupClose();
                sleep(300);
                bot.storage();
                sleep(200);
            }

            // Toggle to pixel stack
            int backIncrement = 0;
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                switch (backIncrement) {
                    case 1: pixelStack = false; centerTruss = false; break;
                    case 2: pixelStack = true; centerTruss = true; break;
                    case 3: pixelStack = true; centerTruss = false; backIncrement = 0; break;
                    default: pixelStack = false; centerTruss = true; break;
                }
                backIncrement++;
            }
            telemetry.addData("Pixel stack (BACK)", pixelStack + "Center truss: " + centerTruss);

            // Change side
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (side == Side.CLOSE) { // Rn close, switch to FAR
                    side = Side.FAR;
                    slidesUp = true;
                } else { // Rn far, switch to CLOSE
                    side = Side.CLOSE;
                    slidesUp = false;
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
                switch (park) {
                    case 0: park = 1; break;
                    case 1: park = 2; break;
                    case 2: park = 0; break;
                }
            }
            String parkSpot = "";
            switch (park) {
                case 1: parkSpot = "Left"; break;
                case 2: parkSpot = "Right"; break;
                default: parkSpot = "None"; break;
            }
            telemetry.addData("Parking (X)", parkSpot);

            // Change backboard wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                if (backboardWait < 10.0) backboardWait+=0.25;
                else backboardWait = 0.0;
            }
            telemetry.addData("Backboard sleep (DPAD UP)", backboardWait + " seconds");

            // Toggle park
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                toBackboard = !toBackboard;
            }
            telemetry.addData("To Backboard (DPAD DOWN)", toBackboard);

            //Toggle Tile Type
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (newTiles == 0) newTiles = 1;
                else if (newTiles == 1) newTiles = 0;
            }
            telemetry.addData("New Tiles (DPAD LEFT)", newTiles);

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

        // Auto start
        if (opModeIsActive() && !isStopRequested()) {
            periodic.start();

            // Drive to spike mark
            startPose = drive.getPoseEstimate();
            if (alliance == Alliance.BLUE) {
                if (side == Side.CLOSE) {
                    switch (spikeMark) {
                        case 1: // LEFT
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .lineToLinearHeading(new Pose2d(35, 30, Math.toRadians(0))) // Line to spike mark
                                    .build());
                            break;
                        case 2: // CENTER
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .back(36)
                                    .forward(12) // Distance away from spike mark when scoring
                                    .build());
                            break;
                        case 3: // RIGHT
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .lineToLinearHeading(new Pose2d(13, 30, Math.toRadians(0))) // Line to spike mark
                                    .build());
                            break;
                    }
                }
            } else if (alliance == Alliance.RED) {
                if (side == Side.CLOSE) {
                    switch (spikeMark) {
                        case 1: // LEFT
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .lineToLinearHeading(new Pose2d(35, -30, Math.toRadians(0))) // Line to spike mark
                                    .build());
                            break;
                        case 2: // CENTER
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .back(36)
                                    .forward(12) // Distance away from spike mark when scoring
                                    .build());
                            break;
                        case 3: // RIGHT
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .lineToLinearHeading(new Pose2d(13, -30, Math.toRadians(0))) // Line to spike mark
                                    .build());
                            break;
                    }
                }
            }

            // Score purple/top pixel
            bot.outtakeGround();
            sleep(1000);
            bot.claw.open();
            sleep(600);
            bot.storage();
            bot.claw.close();
            bot.calculateWristPos();
            bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
            sleep(200);

            // Backstage actions
            if (toBackboard) {
                // To backboard
                int backboardY = 0;
                switch (spikeMark) {
                    case 1: backboardY = 40; break; // LEFT
                    case 2: backboardY = 35; break; // CENTER
                    case 3: backboardY = 30; break; // RIGHT
                }
                if (alliance == Alliance.RED) backboardY*=-1; // Flip sign if red alliance

                startPose = drive.getPoseEstimate();
                if (side == Side.CLOSE) {
                    if (spikeMark == 2) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .splineToLinearHeading(new Pose2d(42, backboardY, Math.toRadians(180)), Math.toRadians(180))
                                .back(10) // Back into backboard
                                .build());
                    } else {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(50, backboardY, Math.toRadians(180)))
                                .build());
                    }
                }

                // Score yellow/bottom pixel on backboard
                if (slidesUp) {
                    bot.slides.runTo(-400.0);
                } else {
                    bot.slides.runToBottom();
                }
                bot.outtakeOut(bot.claw.getClawState());
                bot.fourbar.topOuttake();
                sleep(800);
                bot.claw.open();
                sleep(800);
                // Return to storage
                bot.storage();
                bot.claw.close();
                bot.calculateWristPos();
                bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
                sleep(200);

                // Adjust left/right close blue
                if ((alliance == Alliance.BLUE && side == Side.CLOSE) && (spikeMark == 1 || spikeMark == 3)) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                            .forward(8)
                            .build());
                }

                // PIXEL STACK TRAJECTORY STARTS HERE
                if (side == Side.CLOSE && pixelStack) {
                    Thread reverseIntake = new Thread(() -> {
                        bot.intake(false); // Intake stack
                    });
                    reverseIntake.start();

                    // Drive across field
                    startPose = drive.getPoseEstimate();
                    if (centerTruss) { // Through center truss
                        int strafeAmount = 0;
                        switch (spikeMark) {
                            case 1: strafeAmount = 28; break;
                            case 2: strafeAmount = 23; break;
                            case 3: strafeAmount = 18; break;
                        }
                        if (alliance == Alliance.BLUE) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .strafeLeft(strafeAmount)
                                    .forward(100)
                                    .build()
                            );
                        } else if (alliance == Alliance.RED) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .strafeRight(strafeAmount)
                                    .forward(100)
                                    .build()
                            );
                        }
                    } else { // Through side truss
                        int strafeAmount = 0;
                        switch (spikeMark) {
                            case 1: strafeAmount = 18; break;
                            case 3: strafeAmount = 28; break;
                        }
                        if (spikeMark == 2) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .forward(100)
                                    .build()
                            );
                        }
                        if (alliance == Alliance.BLUE) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .strafeRight(strafeAmount)
                                    .forward(90) // Drive across field
                                    .strafeLeft(22)
                                    .forward(10)
                                    .build()
                            );
                        } else if (alliance == Alliance.RED) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .strafeLeft(18)
                                    .forward(90) // Drive across field
                                    .strafeRight(22)
                                    .forward(10)
                                    .build()
                            );
                        }
                    }

                    // Intake pixels
                    sleep(500); // Wait to knock over pixels?
                    reverseIntake.stop();
                    Thread intake = new Thread(() -> {
                        bot.intake(false); // Intake stack
                    });
                    intake.start();
                    sleep(300); // Wait for intake
                    intake.stop();

                    // Return to backboard
                    startPose = drive.getPoseEstimate();
                    if (centerTruss) { // Through center truss
                        if (alliance == Alliance.BLUE) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .back(100) // Drive across field
                                    .waitSeconds(backboardWait)
                                    .strafeRight(23) // Strafe to center of backboard
                                    .back(10) // Back into backboard
                                    .build()
                            );
                        } else if (alliance == Alliance.RED) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .back(100) // Drive across field
                                    .waitSeconds(backboardWait)
                                    .strafeLeft(23) // Strafe to center of backboard
                                    .back(10) // Back into backboard
                                    .build()
                            );
                        }
                    } else { // Through side truss
                        // LEFT AND RIGHT
                        if (spikeMark == 2) { // CENTER
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .back(110) // Drive across field
                                    .waitSeconds(backboardWait)
                                    .build()
                            );
                        } else {
                            if (alliance == Alliance.BLUE) {
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-50, 11, Math.toRadians(180)))
                                        .back(90)
                                        .waitSeconds(backboardWait)
                                        .strafeRight(23)
                                        .back(10)
                                        .build()
                                );
                            } else if (alliance == Alliance.RED) {
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-50, -11, Math.toRadians(180)))
                                        .back(90)
                                        .waitSeconds(backboardWait)
                                        .strafeLeft(23)
                                        .back(10)
                                        .build()
                                );
                            }
                        }
                    }

                    // Score pixels on backboard
                    bot.slides.runTo(-700.0); // Slides up
                    // First pixel
                    bot.outtakeOut(2);
                    sleep(1000);
                    bot.claw.open();
                    sleep(200);
                    // Second pixel
                    bot.claw.open();
                    sleep(800);
                    // Return to storage
                    bot.storage();
                    bot.claw.close();
                    bot.calculateWristPos();
                    bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
                    sleep(200);
                }

                // Parking
                drive.followTrajectory(drive.trajectoryBuilder(startPose).forward(5).build()); // Back away from backboard
                if (park != 0) {
                    startPose = drive.getPoseEstimate();
                    int parkStrafe = 0;
                    if (pixelStack) { // Pixel stack pixels are always scored on center
                        parkStrafe = 23;
                    } else {
                        if (park == 1) { // Left park
                            switch (spikeMark) {
                                case 1: parkStrafe = 17; break;
                                case 2: parkStrafe = 23; break;
                                case 3: parkStrafe = 31; break;
                            }
                        } else if (park == 2) { // Right park
                            switch (spikeMark) {
                                case 1: parkStrafe = 31; break;
                                case 2: parkStrafe = 23; break;
                                case 3: parkStrafe = 17; break;
                            }
                        }
                    }
                    if (park == 1) { // Left park
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(parkStrafe).build());
                    } else { // Right park
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(parkStrafe).build());
                    }
                }
            }

            // Stop op mode
            sleep(800);
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
