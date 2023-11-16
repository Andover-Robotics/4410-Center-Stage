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
        LEFT, RIGHT, NULL;
    }
    Side side = Side.LEFT;

    // Alliance
    public enum Alliance {
        RED, BLUE, NULL;
    }
    Alliance alliance = Alliance.RED;

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

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // Threads
        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
            }
        });

        // Initialized, adjust values before start
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
                if (side == Side.LEFT) side = Side.RIGHT;
                else side = Side.LEFT;
            }
            telemetry.addData("Side", side);

            // Initiate color detection
            if (alliance == Alliance.BLUE) {
                colorDetection.setAlliance(2);
            } else { // Automatically checks for red if no alliance is set
                colorDetection.setAlliance(1);
            }
            telemetry.addData("Current Camera FPS", camera.getFps());
            telemetry.addData("Spike (1-LEFT,2-MIDDLE,3-RIGHT)", colorDetection.getSpikeMark());

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
            startPose = drive.getPoseEstimate();

            // Spike mark trajectory
            switch(colorDetection.getSpikeMark()) {
                case 1: // LEFT
                    TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .turn(Math.toRadians(90))
                        .build();
                    drive.followTrajectorySequence(left);
                case 3: // RIGHT
                    TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .turn(Math.toRadians(-90))
                        .build();
                    drive.followTrajectorySequence(right);
                default: // MIDDLE (also case 2)
                    TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                        .forward(30)
                        .build();
                    drive.followTrajectorySequence(middle);
            }

            // Pick up purple pixel and place on ground
            Thread purple = new Thread(() -> {
                bot.slides.runToBottom();
                bot.claw.open();
                sleep(100);
                bot.fourbar.topPixel();
                sleep(400);
                bot.claw.close();
                sleep(300);
                bot.storage();
            });
            purple.start();
            bot.outtakeGround();

            // Run to backboard trajectory (checks side and alliance)
            Pose2d startPose2 = drive.getPoseEstimate();
            Vector2d scoreBlue = new Vector2d(42,30), scoreRed = new Vector2d(42,-30); // Vector2d spline end positions (backboard)
            if ((side == Side.LEFT)) {
                if (alliance == Alliance.RED) {
                    Trajectory redFar = drive.trajectoryBuilder(startPose2)
                            .splineTo(scoreRed,Math.toRadians(0))
                            .build();
                    drive.followTrajectory(redFar);
                }
                Trajectory blueClose = drive.trajectoryBuilder(startPose2)
                        .splineTo(scoreBlue,Math.toRadians(0))
                        .build();
                drive.followTrajectory(blueClose);
            } else {
                if (alliance == Alliance.BLUE) {
                    Trajectory blueFar = drive.trajectoryBuilder(startPose2)
                            .splineTo(scoreBlue,Math.toRadians(0))
                            .build();
                    drive.followTrajectory(blueFar);
                }
                Trajectory redClose = drive.trajectoryBuilder(startPose2)
                        .splineTo(scoreRed,Math.toRadians(0))

                        .build();
                drive.followTrajectory(redClose);
            }

            // TODO: IMPLEMENT APRIL TAG DETECTION AS WELL AS STRAFING!
            // Initiate april tag detection
//            camera.setPipeline(aprilTagDetectionPipeline);
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == ID_ONE || tag.id == ID_TWO || tag.id == ID_THREE) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if (tagFound) {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag of interest :(");
//                    if (tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//            } else {
//                telemetry.addLine("Don't see tag of interest :(");
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }

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

