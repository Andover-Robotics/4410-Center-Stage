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

    // Side - how close to backboard: LEFT - furthest, RIGHT - closest
    public enum Side {
        LEFT, RIGHT, NULL;
    }
    Side side = Side.NULL;

    // Alliance
    public enum Alliance {
        RED, BLUE, NULL;
    }
    Alliance alliance = Alliance.NULL;

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

        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
            }
        });

        // Initialized, adjust values before start
        while (!isStarted()) {
            gp1.readButtons();

            // Change alliance
            telemetry.addData("Alliance", alliance);
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                if (alliance == Alliance.RED) alliance = Alliance.BLUE;
                else alliance = Alliance.RED;
            }

            // Adjust side
            telemetry.addData("Side", side);
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (Objects.requireNonNull(side) == Side.LEFT) side = Side.RIGHT;
                else side = Side.LEFT;
            }

            // Adjust alliance and color detection
            switch (alliance) {
                case RED: colorDetection.setAlliance(1); break;
                case BLUE: colorDetection.setAlliance(2); break;
                case NULL: colorDetection.setAlliance(2); break;
            }
            telemetry.addData("Current Camera FPS", camera.getFps());
            telemetry.addData("Spike(1-LEFT,2-MIDDLE,3-RIGHT)", colorDetection.getSpikeMark());

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

            Pose2d startP = drive.getPoseEstimate();
            TrajectorySequence[] sequence = makeTrajectories(drive, startP, colorDetection.getSpikeMark());

            drive.followTrajectorySequence(sequence[0]);
            drive.followTrajectorySequenceAsync(sequence[1]);
            // Initiate april tag detection
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
            telemetry.addLine("Exception as followsL: " + e);
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
                        .forward(26)
                        .turn(Math.toRadians(90))
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
            if (alliance == Alliance.RED) {
                return new TrajectorySequence[]{spikeMark, redFar};
            }
            return new TrajectorySequence[]{spikeMark, blueClose};
        } else {
            if (alliance == Alliance.BLUE) {
                return new TrajectorySequence[]{spikeMark, blueFar};
            }
            return new TrajectorySequence[]{spikeMark, redClose};
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

