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
    enum Side {
        CLOSE, FAR, NULL;
    }
    public static Side side = Side.CLOSE;

    // Alliance
    enum Alliance {
        RED, BLUE, NULL;
    }
    public static Alliance alliance = Alliance.BLUE;

    // Autonomous config values
    public static boolean toBackboard = true; // Go to backboard: true - go to backboard and score, false - only score spike and stop
    public static boolean pixelStack = true; // Go to pixel stack: true - do 2+2, false - park/or stop after scoring yellow pixel
    public static boolean centerTruss = true; // Middle truss go under: true - center truss, false - side truss
    public static int slidesPos = 0; // Slide up: 0-1000, increment by 200
    public static int park = 0; // Parking position: 0 - don't park, 1 - left, 2 - right

    // WAIT TIME CONFIG: In seconds from a range 0-15, increment by 1 second
    public static double backboardWait = 0.0; // How long to wait before going to and scoring on backboard
    public static double spikeWait = 0.0; // How long to wait before going to spike mark position and scoring
    public static double stackWait = 0.0; // How long to wait before scoring on backboard after stack (essentially backboard 2)

    int secondsElapsed = 0; // Track how many seconds have passed

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

        // THREADS

        // Run slides periodic
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
            }
        });

        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        // Pickup
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
        X - change park
        B - change pixel stack parameters

        (DPAD)
        up - change backboard wait time
        down - change spike wait time
        right - change stack wait time

        right bumper - increment slides height
        left bumper - decrement slides height

        START - re-pickup pixel
        BACK - toggle to backboard
         */
        while (!isStarted()) {
            gp1.readButtons();

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
            }

            // Change alliance
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                if (alliance == Alliance.RED)  {
                    alliance = Alliance.BLUE;
                } else  {
                    alliance = Alliance.RED;
                }
            }
            telemetry.addData("Alliance (Y)", alliance);

            // Toggle to pixel stack
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                if (pixelStack && centerTruss) { // Pixel stack through center truss -> pixel stack through side truss
                    pixelStack = true; // note: ik this logic can be simplified but i'm leaving it for readability - zachery
                    centerTruss = false;
                } else if (pixelStack && !centerTruss) { // Pixel stack through side truss -> none
                    pixelStack = false;
                    centerTruss = false;
                } else if (!pixelStack && !centerTruss) { // No pixel stack (no truss) -> pixel stack through center truss
                    pixelStack = true;
                    centerTruss = true;
                }
            }
            telemetry.addData("PixelStack (B)", pixelStack + " CenterTruss: " + centerTruss);

            // Change side
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (side == Side.CLOSE) { // Rn close, switch to FAR
                    side = Side.FAR;
                    pixelStack = false;
                    centerTruss = false;
                } else { // Rn far, switch to CLOSE
                    side = Side.CLOSE;
                    slidesPos = 0;
                }
            }
            telemetry.addData("Side (A)", side);

            // CHANGE SLIDES HEIGHT
            // Increment
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                if (slidesPos < 2000) slidesPos += 200;
                else slidesPos = 2000;
            }
            // Decrement
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                if (slidesPos > 0) slidesPos -= 200;
                else slidesPos = 0;
            }
            telemetry.addData("Slides (RB)", slidesPos);

            // Switch park
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

            // WAIT TIMES
            // Change backboard wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                if (backboardWait < 15.0) backboardWait+=1.0;
                else backboardWait = 0.0;
            }
            // Change spike mark wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                if (spikeWait < 15.0) spikeWait+=1.0;
                else spikeWait = 0.0;
            }
            // Change stack wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                if (stackWait < 15.0) stackWait+=1.0;
                else stackWait = 0.0;
            }
            telemetry.addData("WaitSeconds: Backboard(UP)", backboardWait + " Spike(DOWN): " + spikeWait + " Stack(RIGHT): " + stackWait);

            // Toggle to backboard
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                toBackboard = !toBackboard;
            }
            telemetry.addData("ToBackboard (BACK)", toBackboard);

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
            trackTime.start();

            // Set starting poses
            Pose2d blueCloseStart = new Pose2d(12,60,Math.toRadians(90));
            Pose2d blueFarStart = new Pose2d(-35,60,Math.toRadians(90));
            Pose2d redCloseStart = new Pose2d(12,-60,Math.toRadians(-90));
            Pose2d redFarStart = new Pose2d(-35,-60,Math.toRadians(-90));
            if (alliance == Alliance.BLUE && side == Side.CLOSE) {
                drive.setPoseEstimate(blueCloseStart);
            } else if (alliance == Alliance.BLUE && side == Side.FAR) {
                drive.setPoseEstimate(blueFarStart);
            } else if (alliance == Alliance.RED && side == Side.CLOSE) {
                drive.setPoseEstimate(redCloseStart);
            } else if (alliance == Alliance.RED && side == Side.FAR) {
                drive.setPoseEstimate(redFarStart);
            }
            bot.setAutoEndPose(drive.getPoseEstimate());

            periodic.start();

            // Drive to spike mark
            Pose2d startPose = drive.getPoseEstimate();
            Pose2d spikePose = new Pose2d();
            if (alliance == Alliance.BLUE) {
                if (side == Side.CLOSE) {
                    switch (spikeMark) {
                        case 1: spikePose = new Pose2d(34, 32, Math.toRadians(0)); break;
                        case 2: spikePose = new Pose2d(17, 34, Math.toRadians(90)); break;
                        case 3: spikePose = new Pose2d(13, 32, Math.toRadians(0)); break;
                    }
                } else if (side == Side.FAR) {
                    switch (spikeMark) {
                        case 1: spikePose = new Pose2d(-33, 34, Math.toRadians(180)); break;
                        case 2: spikePose = new Pose2d(-51, 22, Math.toRadians(180)); break;
                        case 3: spikePose = new Pose2d(-40,41, Math.toRadians(60)); break;
                    }
                }
            } else if (alliance == Alliance.RED) {
                if (side == Side.CLOSE) {
                    switch (spikeMark) {
                        case 1: spikePose = new Pose2d(13, -32, Math.toRadians(0)); break;
                        case 2: spikePose = new Pose2d(17, -36, Math.toRadians(-90)); break;
                        case 3: spikePose = new Pose2d(36, -32, Math.toRadians(0)); break;
                    }
                } else if (side == Side.FAR) {
                    switch (spikeMark) {
                        case 1: spikePose = new Pose2d(-39,-42, Math.toRadians(-60)); break;
                        case 2: spikePose = new Pose2d(-52, -22, Math.toRadians(180)); break;
                        case 3: spikePose = new Pose2d(-36, -33, Math.toRadians(180)); break;
                    }
                }
            }
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                    .waitSeconds(spikeWait) // Wait before going to spike mark
                    .lineToLinearHeading(spikePose) // Line to spike mark
                    .build());

            // Score purple pixel
            bot.outtakeGround();
            sleep(700);
            bot.claw.halfOpen();
            sleep(500);
            bot.storage();
            bot.claw.close();

            // Backstage actions
            if (toBackboard) {
                // To backboard
                int backboardY = 0;
                if (alliance == Alliance.BLUE) {
                    switch (spikeMark) {
                        case 1: backboardY = 40; break; // LEFT
                        case 2: backboardY = 36; break; // CENTER
                        case 3: backboardY = 30; break; // RIGHT
                    }
                } else if (alliance == Alliance.RED) {
                    switch (spikeMark) {
                        case 1: backboardY = -30; break; // LEFT
                        case 2: backboardY = -36; break; // CENTER
                        case 3: backboardY = -40; break; // RIGHT
                    }
                }

                startPose = drive.getPoseEstimate();
                if (side == Side.CLOSE) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                            .waitSeconds(backboardWait)
                            .lineToLinearHeading(new Pose2d(54, backboardY, Math.toRadians(180)))
                            .build());
                } else if (side == Side.FAR) {
                    if (alliance == Alliance.BLUE) {
                        switch (spikeMark) {
                            case 1: // LEFT
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-35, 12, Math.toRadians(180)))
                                        .back(91 - 10) // Drive to backboard
                                        .waitSeconds(backboardWait)
                                        .lineToLinearHeading(new Pose2d(54, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                            case 2: // CENTER
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .strafeLeft(13) // Strafe to center truss
                                        .back(106 - 10) // Drive to backboard
                                        .waitSeconds(backboardWait) // Wait for close auto to finish
                                        .lineToLinearHeading(new Pose2d(54, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-35, 55, Math.toRadians(180))) // Back to start but with different heading
                                        .lineToLinearHeading(new Pose2d(-35, 12, Math.toRadians(180))) // To center truss
                                        .back(88 - 10) // Drive across field
                                        .waitSeconds(backboardWait)
                                        .lineToLinearHeading(new Pose2d(54, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                        }
                    } else if (alliance == Alliance.RED) {
                        switch (spikeMark) {
                            case 1: // LEFT
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-35, -55, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(180)))
                                        .back(86) // Drive across field
                                        .strafeLeft(17)
                                        .build());
                                break;
                            case 2: // CENTER
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .strafeRight(13) // Strafe to center truss
                                        .back(104) // Drive to backboard
                                        .waitSeconds(backboardWait) // Wait for close auto to finish
                                        .strafeLeft(25) // Strafe to center
                                        .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(180)))
                                        .back(86) // Drive to backboard
                                        .waitSeconds(backboardWait)
                                        .strafeLeft(31) // Strafe to left
                                        .build());
                                break;
                        }
                    }
                }

                // Run into backboard
                int slowerVelocity = 20; // Slower velocity that is the max constraint when running into backboard (in/s)
//                if (side == Side.FAR) {
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(2,
//                                    SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                            .build());
//                }

                // Score yellow pixel on backboard
                if (slidesPos != 0) {
                    bot.slides.runTo(-slidesPos);
                } else {
                    bot.slides.runToBottom();
                }
                bot.outtakeOut(1);
                bot.fourbar.autoTopOuttake();
                sleep(1000);
                bot.claw.extraOpen();
                sleep(600);
                // Return to storage
                bot.storage();
                bot.calculateWristPos();
                bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
                bot.claw.close();
                sleep(100);

                // TODO: TUNE/CODE PIXEL STACK TRAJECTORY FOR 2+2
                // PIXEL STACK TRAJECTORY STARTS HERE
                if (side == Side.CLOSE && pixelStack) {
                    bot.intake(true); // Reverse intake

                    // Drive across field
                    startPose = drive.getPoseEstimate();
                    int acrossDistance = 90; // Tune this value across field
                    if (alliance == Alliance.BLUE) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(35, 12, Math.toRadians(180))) // Line to center truss position
                                .lineToLinearHeading(new Pose2d(-57, 13, Math.toRadians(180))) // Through field
                                .build()
                        );
                    } else if (alliance == Alliance.RED) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(180))) // Line to center truss position
                                .lineToLinearHeading(new Pose2d(-57, -13, Math.toRadians(180))) // Through field
                                .build()
                        );
                    }
                    sleep(600);



                    // AT PIXEL STACK, intake pixels
                    bot.intake(false);
                    //sleep(700); // Wait for intake
//                    // Go back
//                    if (alliance == Alliance.BLUE) {
//                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-53, 13, Math.toRadians(180)))
//                                .build()
//                        );
//                    } else if (alliance == Alliance.RED) {
//                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(-53, -12, Math.toRadians(180)))
//                                .build()
//                        );
//                    }
//                    bot.intake(false);
                    // Run forward into stack again
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(1)
                            .build()
                    );
                    sleep(1800);

                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .back(2)
                            .build()
                    );

                    bot.intake(true);

                    // RETURN TO BACKBOARD
                    startPose = drive.getPoseEstimate();
                    if (alliance == Alliance.BLUE) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .back(acrossDistance) // Drive across field
                                .waitSeconds(stackWait)
                                .build()
                        );
                    } else if (alliance == Alliance.RED) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .back(acrossDistance) // Drive across field
                                .waitSeconds(stackWait)
                                .build()
                        );
                    }
                    Thread pixelTap = new Thread(() -> {
                        bot.slides.runToBottom();
                        bot.claw.fullOpen();
                        sleep(100);
                        bot.fourbar.pickup();
                        sleep(400);
                        bot.claw.pickupClose();
                        sleep(300);
                        bot.claw.fullOpen();
                        bot.storage();
                        sleep(200);
                        // Pick up pixels
                        bot.slides.runToBottom();
                        bot.claw.fullOpen();
                        sleep(100);
                        bot.fourbar.pickup();
                        sleep(200);
                        bot.claw.pickupClose();
                        sleep(200);
                        bot.storage();
                        sleep(500);
                    });
                    pixelTap.start();
                    bot.intake.stopIntake();

                    // Line to backboard
                    startPose = drive.getPoseEstimate();
                    if (alliance == Alliance.BLUE) {
                        drive.followTrajectory(drive.trajectoryBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(55, 30, Math.toRadians(180)),
                                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                    } else if (alliance == Alliance.RED) {
                        drive.followTrajectory(drive.trajectoryBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(55, -30, Math.toRadians(180)),
                                    SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build());
                    }

                    // Score pixels on backboard
                    sleep(200);
                    bot.slides.runTo(-300); // Slides up
                    // First pixel
                    bot.outtakeOut(2);
                    sleep(500);
                    bot.claw.halfOpen();
                    sleep(600);
                    // Second pixel
                    bot.slides.runTo(-500);
                    sleep(200);
                    bot.outtakeOut(1);
                    bot.claw.extraOpen();
                    sleep(600);
                    // Return to storage
                    bot.storage();
                    bot.claw.close();
                    bot.calculateWristPos();
                    bot.fourbar.wrist.setPosition(bot.fourbar.wrist.getPosition()+ bot.wristUpPos);
                    sleep(200);
                }
                // END OF PIXEL STACK TRAJECTORY

                // Parking
                startPose = drive.getPoseEstimate();
                drive.followTrajectory(drive.trajectoryBuilder(startPose).forward(2).build()); // Back away from backboard
                if (park != 0) {
                    int parkStrafe = 0;
                    int num1 = 18, num2 = 21, num3 = 29;
                    if (pixelStack) { // Pixel stack pixels are always scored on center
                        parkStrafe = 23;
                    } else {
                        if (park == 1) { // Left park
                            switch (spikeMark) {
                                case 1: parkStrafe = num1; break;
                                case 2: parkStrafe = num2; break;
                                case 3: parkStrafe = num3; break;
                            }
                        } else if (park == 2) { // Right park
                            switch (spikeMark) {
                                case 1: parkStrafe = num3; break;
                                case 2: parkStrafe = num2; break;
                                case 3: parkStrafe = num1; break;
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
}