package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.auto.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.auto.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive.getVelocityConstraint;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Test April Tag Detection", group = "Test")
public class AprilTagDetectionTest extends LinearOpMode {
    private Bot bot;

    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;
    double tagsize = 0.032; // METERS -- ONLY FOR TESTING


    // Tag ID 1,2,3 from the 36h11 family
    int ID_ONE = 1;
    int ID_TWO = 2;
    int ID_THREE = 3;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        while (opModeIsActive() && !isStopRequested()) {
            Vector2d scoreRed = new Vector2d(42,-30);
            Vector2d parkingPosRed = new Vector2d(56,-56);

            TrajectorySequence sequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineTo(scoreRed,Math.toRadians(0))
                    .waitSeconds(1.5)
                    .setVelConstraint(getVelocityConstraint(MAX_VEL/2.0, MAX_ANG_VEL, TRACK_WIDTH))
                    .strafeRight(26)
                    .splineTo(parkingPosRed,Math.toRadians(0))
                    .build();

            drive.followTrajectorySequenceAsync(sequence);
            while (sequence.duration() < 6.0) {
                while (sequence.duration() >2.0 && sequence.duration() < 4.0) {
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
        while (!isStarted()) {
            telemetry.addData("Current Camera FPS:", camera.getFps());
            telemetry.update();
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        double FEET_PER_METER = 3.28084;
        telemetry.addLine(String.format("Detected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.x)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.y)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.z)));
    }
}
