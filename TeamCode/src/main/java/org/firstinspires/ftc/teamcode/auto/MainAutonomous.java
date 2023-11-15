package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        TwoWheelTrackingLocalizer odometry = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        //CAMERA STUFF =====================

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);



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
            gp1.readButtons();
            // Change move differential
            telemetry.addData("moveDiff (positive is more ???)", moveDiff);
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                moveDiff -= 0.5;
            } else if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                moveDiff += 0.5;
            }
            // Change alliance
            telemetry.addData("Alliance", alliance);
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                switch (alliance) {
                    case RED: alliance = Alliance.BLUE; break;
                    case BLUE: alliance = Alliance.RED; break;
                    case NULL: alliance = Alliance.RED; break;
                }
            }
            // Change side
            telemetry.addData("Side", side);
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                switch (side) {
                    case LEFT: side = Side.RIGHT; break;
                    case RIGHT: side = Side.LEFT; break;
                    case NULL: side = Side.LEFT; break;
                }
            }
            telemetry.addData("Current Camera FPS:", camera.getFps());
            switch (alliance) {
                case NULL: colorDetection = new ColorDetectionPipeline(telemetry, 0); break;
                case RED: colorDetection = new ColorDetectionPipeline(telemetry, 1); break;
                case BLUE: colorDetection = new ColorDetectionPipeline(telemetry, 2); break;
            }
            telemetry.update();
            sleep(20);
        }

        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) {

        }
        //END CAMERA STUFF ===============

        // TRAJECTORIES
        //bot.resetIMU();

        waitForStart();
        if (!isStopRequested()) {
            periodic.start();

            camera.setPipeline(colorDetection);
            int aligned = bot.alignSpike();
            switch(aligned) {
                case 0: telemetry.addLine("spike mark not found");
                case 1: telemetry.addLine("spike mark found on left");
                case 2: telemetry.addLine("spike mark found on middle");
                case 3: telemetry.addLine("spike mark found on right");
            }

            Pose2d startP = drive.getPoseEstimate();
            TrajectorySequence[] sequence = makeTrajectories(drive, startP, aligned);
            drive.followTrajectorySequence(sequence[0]);
            drive.followTrajectorySequenceAsync(sequence[1]);

            camera.setPipeline(aprilTagDetectionPipeline);
            while (sequence[1].duration() < 13.0) {
                while (sequence[1].duration() > 6.0 && sequence[1].duration() < 8.0) {
                    ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
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

    public TrajectorySequence generateSpikeMarkTrajectory(SampleMecanumDrive drive, Pose2d startPose, int alignment) {
        switch(alignment) {
            case 1:
                return drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .turn(Math.toRadians(90))
                        .build();
            case 2:
                return drive.trajectorySequenceBuilder(startPose)
                        .forward(30)
                        .build();
            case 3:
                return drive.trajectorySequenceBuilder(startPose)
                        .forward(26)
                        .turn(Math.toRadians(-90))
                        .build();
            default:
                return drive.trajectorySequenceBuilder(startPose)
                        .build();
        }
    }
    public TrajectorySequence[] makeTrajectories(SampleMecanumDrive drive, Pose2d startPose, int alignment) {

        Vector2d parkingPosBlue = new Vector2d(56,56);
        Vector2d parkingPosRed = new Vector2d(56,-56);
        Vector2d scoreBlue = new Vector2d(42,30);
        Vector2d scoreRed = new Vector2d(42,-30);

        TrajectorySequence spikeMark = generateSpikeMarkTrajectory(drive, startPose, alignment);
        Pose2d startPose2 = drive.getPoseEstimate();


        TrajectorySequence redClose = drive.trajectorySequenceBuilder(startPose2)
                .splineTo(scoreRed,Math.toRadians(0))
                .waitSeconds(1.5)
                .strafeRight(26)
                .splineTo(parkingPosRed,Math.toRadians(0))
                .build();
        TrajectorySequence blueClose = drive.trajectorySequenceBuilder(startPose2)
                .splineTo(scoreBlue,Math.toRadians(0))
                .waitSeconds(1.5)
                .strafeLeft(26)
                .splineTo(parkingPosBlue,Math.toRadians(0))
                .build();
        TrajectorySequence redFar = drive.trajectorySequenceBuilder(startPose2)
                .splineTo(scoreRed,Math.toRadians(0))
                .waitSeconds(1.5)
                .strafeRight(26)
                .splineTo(parkingPosRed,Math.toRadians(0))
                .build();
        TrajectorySequence blueFar = drive.trajectorySequenceBuilder(startPose2)
                .splineTo(scoreBlue,Math.toRadians(0))
                .waitSeconds(1.5)
                .strafeLeft(26)
                .splineTo(parkingPosBlue,Math.toRadians(0))
                .build();
        if ((side == Side.LEFT)) {
            if (Objects.requireNonNull(alliance) == Alliance.RED) {
                return new TrajectorySequence[]{spikeMark, redFar};
            }
            return new TrajectorySequence[]{spikeMark, blueClose};
        } else {
            if (Objects.requireNonNull(alliance) == Alliance.BLUE) {
                return new TrajectorySequence[]{spikeMark, blueFar};
            }
            return new TrajectorySequence[]{spikeMark, redClose};
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

