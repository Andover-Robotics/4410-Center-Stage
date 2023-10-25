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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    private boolean autoaim = false, isRight = false;

    enum Side {
        RIGHT, LEFT, NULL;
    }

    enum Color {
        RED, BLUE, NULL;
    }

    Side side = Side.NULL;
    Color color = Color.NULL;

    boolean isTestMode = false;

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

        if (color == Color.RED) {
            colorDetection = new ColorDetectionPipeline(telemetry, 1);
        } else if (color == Color.BLUE) {
            colorDetection = new ColorDetectionPipeline(telemetry, 2);
        } else {
            colorDetection = new ColorDetectionPipeline(telemetry, 0);
        }


        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);


        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
            }
        });

//        telemetry.setMsTransmissionInterval(50);


        while (!isStarted()) {
            telemetry.addData("moveDiff (positive is more ???)", moveDiff);
            if(gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                moveDiff -= 0.5;
            }else if(gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                moveDiff += 0.5;
            }

            telemetry.addData("side?", side.toString());
            telemetry.addData("testmode", isTestMode);
            telemetry.addData("AutoAim", autoaim);

            telemetry.addData("Current FPS:", camera.getFps());
            telemetry.addData("Current Max FPS:", camera.getCurrentPipelineMaxFps());
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

        Trajectory forwardFarSide = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(52, moveDiff))
                .build();

        Trajectory forwardNearSide = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(26, moveDiff))
                .build();


        Trajectory strafeLeftFarSide = drive.trajectoryBuilder(forwardFarSide.end())
                .strafeLeft(26)
                .build();
        Trajectory strafeLeftNearSide = drive.trajectoryBuilder(forwardNearSide.end())
                .strafeLeft(26)
                .build();
        Trajectory strafeRightFarSide = drive.trajectoryBuilder(forwardFarSide.end())
                .strafeRight(26)
                .build();
        Trajectory strafeRightNearSide = drive.trajectoryBuilder(forwardNearSide.end())
                .strafeRight(26)
                .build();

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
            Trajectory forward;
            Trajectory strafe;
            if ((side == Side.LEFT && color == Color.RED) || (side == Side.RIGHT && color == Color.BLUE)) {
                if (side == Side.LEFT && color == Color.RED) {
                    strafe = strafeLeftFarSide;
                } else {
                    strafe = strafeLeftNearSide;
                }
                forward = forwardFarSide;
            } else {
                if (side == Side.RIGHT && color == Color.RED) {
                    strafe = strafeRightNearSide;
                } else {
                    strafe = strafeRightFarSide;
                }
                forward = forwardNearSide;
            }

//            camera.setPipeline(colorDetection);

            periodic.start();

            drive.turn(Math.toRadians(90));
            forward.start();

            sleep(driveTime);

            strafe.start();

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            while (strafe.duration()>0.0) {
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
            } catch (OpenCvCameraException e) {}

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

