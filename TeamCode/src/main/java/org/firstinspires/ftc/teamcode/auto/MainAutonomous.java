package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.ColorDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Map;

import java.util.Timer;

@Config
@Autonomous(name = "MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    Bot bot;

    private double moveDiff = -1;

    // Side - how close to backboard: LEFT - furthest away, RIGHT -
    public enum Side {
        LEFT, RIGHT, NULL;
    }
    Side side = Side.NULL;
    public enum Alliance {
        RED, BLUE, NULL;
    }
    Alliance alliance = Alliance.NULL;

    public static int driveTime = 2000, timeSlidesUp = 900, timeSlidesDown = 550, timeOuttake = 350, timeConeDrop = 150, timeIntakeDown = 200, timeIntakeOut = 700, timeIntakeClose = 350, timeIntakeUp = 450, timeIntakeIn = 400;//old 400

    //    private static int horizIntake = {}
    static final double FEET_PER_METER = 3.28084;

    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

    // UNITS ARE METERS
    double tagsize = 0.032; //ONLY FOR TESTING

    // Tag ID 1,2,3 from the 36h11 family
    int ID_ONE = 1;
    int ID_TWO = 2;
    int ID_THREE = 3;
    AprilTagDetection tagOfInterest = null;
    ColorDetectionPipeline colorDetection;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(true);
        bot = Bot.getInstance(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        //CAMERA STUFF =====================

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        switch (alliance) {
            case NULL: colorDetection = new ColorDetectionPipeline(telemetry, 0); break;
            case RED: colorDetection = new ColorDetectionPipeline(telemetry, 1); break;
            case BLUE: colorDetection = new ColorDetectionPipeline(telemetry, 2); break;
        }

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
            }
        });

        // Initialized, before started
        while (!isStarted()) {
            // Change move differential
            telemetry.addData("moveDiff (positive is more ???)", moveDiff);
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                moveDiff -= 0.5;
            } else if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                moveDiff += 0.5;
            }
            // Change alliance
            telemetry.addData("Alliance", alliance.toString());
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                alliance = Alliance.RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                alliance = Alliance.BLUE;
            }
            // Change side
            telemetry.addData("Side", side.toString());
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                side = Side.RIGHT;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                side = Side.LEFT;
            }
            telemetry.addData("Current Camera FPS:", camera.getFps());
//
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == ID_ONE || tag.id == ID_TWO || tag.id == ID_THREE) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if (tagFound) {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if (tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            } else {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }

            telemetry.update();
            sleep(20);
        }

//        try {
//            camera.stopStreaming();
//            camera.closeCameraDevice();
//        } catch (OpenCvCameraException e) {
//
//        }
        //END CAMERA STUFF ===============

        // TRAJECTORIES
        Pose2d startPoseBlueFar = new Pose2d(-36, 60, -90);
        Pose2d startPoseBlueClose = new Pose2d(12, 60, -90);
        Pose2d startPoseRedClose = new Pose2d(12, -60, 90);
        Pose2d startPoseRedFar = new Pose2d(-36, -60, 90);

        Vector2d parkingPosBlue = new Vector2d(56,56);
        Vector2d parkingPosRed = new Vector2d(56,-56);
        Vector2d scoreBlue = new Vector2d(42,30);
        Vector2d scoreRed = new Vector2d(42,-30);

        TrajectorySequence redClose = drive.trajectorySequenceBuilder(startPoseRedClose)
                .splineTo(new Vector2d(12,-36),Math.toRadians(90))
                .waitSeconds(1.5)
                .splineTo(scoreRed,Math.toRadians(0))
                .waitSeconds(1.5)
                .strafeRight(26)
                .splineTo(parkingPosRed,Math.toRadians(0))
                .build();
        TrajectorySequence blueClose = drive.trajectorySequenceBuilder(startPoseBlueClose)
                                .splineTo(new Vector2d(12,36),-Math.toRadians(90))
                                .waitSeconds(1.5)
                                .splineTo(scoreBlue,Math.toRadians(0))
                                .waitSeconds(1.5)
                                .strafeLeft(26)
                                .splineTo(parkingPosBlue,Math.toRadians(0))
                                .build();
        TrajectorySequence redFar = drive.trajectorySequenceBuilder(startPoseRedFar)
                                .splineTo(new Vector2d(-36,-36),Math.toRadians(90))
                                .waitSeconds(1.5)
                                .splineTo(scoreRed,Math.toRadians(0))
                                .waitSeconds(1.5)
                                .strafeRight(26)
                                .splineTo(parkingPosRed,Math.toRadians(0))
                                .build();
        TrajectorySequence blueFar = drive.trajectorySequenceBuilder(startPoseBlueFar)
                                .splineTo(new Vector2d(-36,36),-Math.toRadians(90))
                                .waitSeconds(1.5)
                                .splineTo(scoreBlue,Math.toRadians(0))
                                .waitSeconds(1.5)
                                .strafeLeft(26)
                                .splineTo(parkingPosBlue,Math.toRadians(0))
                                .build();
//        Trajectory forwardFar = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(52, moveDiff))
//                .build();
//        Trajectory forwardNear = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(26, moveDiff))
//                .build();
//
//        Trajectory strafeLeftFar = drive.trajectoryBuilder(forwardFar.end())
//                .strafeLeft(26)
//                .build();
//        Trajectory strafeLeftNear = drive.trajectoryBuilder(forwardNear.end())
//                .strafeLeft(26)
//                .build();
//        Trajectory strafeRightFar = drive.trajectoryBuilder(forwardFar.end())
//                .strafeRight(26)
//                .build();
//        Trajectory strafeRightNear= drive.trajectoryBuilder(forwardNear.end())
//                .strafeRight(26)
//                .build();

//        Trajectory parkLeft = drive.trajectoryBuilder(forward.end())
//                .strafeLeft(22)
//                .build();

//        Trajectory parkRight = drive.trajectoryBuilder()
//                .strafeRight(30)
//                .build();
//        Trajectory parkLeft = drive.trajectoryBuilder(forward.end())
//                .strafeRight(30)
//                .build();

//        Thread driveForward = new Thread(() -> drive.followTrajectory(forward));
        bot.resetIMU();

        waitForStart();
        if (!isStopRequested()) {
            TrajectorySequence sequence;
//            Trajectory forward;
//            Trajectory strafe;
//            if ((side == Side.LEFT && alliance == Alliance.RED) || (side == Side.RIGHT && alliance == Alliance.BLUE)) {
//                if (side == Side.LEFT && alliance == Alliance.RED) {
//                    strafe = strafeLeftFar;
//                } else {
//                    strafe = strafeLeftNear;
//                }
//                forward = forwardFar;
//            } else {
//                if (side == Side.RIGHT && alliance == Alliance.RED) {
//                    strafe = strafeRightFar;
//                } else {
//                    strafe = strafeRightNear;
//                }
//                forward = forwardFar;
//            }
            if ((side == Side.LEFT)) {
                switch (alliance) {
                    case RED:
                        sequence = redFar;
                        break;
                    default:
                        sequence = blueClose;
                }
            } else {
                switch (alliance) {
                    case BLUE:
                        sequence = blueFar;
                        break;
                    default:
                        sequence = redClose;
                        break;
                }
            }
//            camera.setPipeline(colorDetection);

            periodic.start();
            sequence.start();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            while (sequence.duration()>6.0 && sequence.duration() <8.0) {
                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == ID_ONE || tag.id == ID_TWO || tag.id == ID_THREE) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound) {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    } else {
                        telemetry.addLine("Don't see tag of interest :(");

                        if (tagOfInterest == null) {
                            telemetry.addLine("(The tag has never been seen)");
                        } else {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }
            }
            periodic.interrupt();

            try {
                camera.stopStreaming();
                camera.closeCameraDevice();
            } catch (OpenCvCameraException e) {
                telemetry.addLine("Exception as follows: "+e);
            }

        }
    }



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

