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
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline2;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.*;
import org.firstinspires.ftc.teamcode.auto.util.PoseStorage;
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
@Autonomous(name = "MainAutonomous")
public class MainAutonomous extends LinearOpMode {
    Bot bot;

    // Side - how close to backboard: LEFT - furthest, RIGHT - closest
    enum Side {
        CLOSE, FAR, NULL;
    }
    public static Side side = Side.CLOSE;

    // Alliance
    public enum Alliance {
        RED, BLUE, NULL;
    }
    public static Alliance alliance = Alliance.BLUE;

    // Autonomous config values
    public static boolean toBackboard = true; // Go to backboard: true - go to backboard and score, false - only score spike and stop
    public static boolean pixelStack = true; // Go to pixel stack: true - do 2+2, false - park/or stop after scoring yellow pixel
    public static int slidesPos = 0; // Slide up: 0-1000, increment by 200
    public static int park = 0; // Parking position: 0 - don't park, 1 - left, 2 - right

    // WAIT TIME CONFIG: In seconds from a range 0-10, increment by 1 second
    public static double backboardWait = 0.0; // How long to wait before going to and scoring on backboard
    public static double spikeWait = 0.0; // How long to wait before going to spike mark position and scoring
    public static double stackWait = 0.0; // How long to wait before going to pixel stack
    public static double backWait = 0.0; // How long to wait before scoring on backboard after stack (essentially backboard 2)
    public static double parkWait = 0.0; // How long to wait before going to parking position (run park spline)

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
        ColorDetectionPipeline2 colorDetection = new ColorDetectionPipeline2(telemetry);

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
                sleep(1000);
            }
        });

        bot.state = Bot.BotState.STORAGE;
        bot.storage();
        bot.claw.fullOpen();

        // Pickup
        Thread pickup = new Thread(() -> {
            sleep(500);
            bot.slides.runToBottom();
            bot.claw.fullOpen();
            sleep(400);
            bot.fourbar.pickup();
            sleep(400);
            bot.claw.pickupClose();
            sleep(300);
            bot.storage();
            sleep(200);
            stop();
        });
        pickup.start();

        Thread stopAuto = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                if (drive.getExternalHeadingVelocity() > (0.2 * DriveConstants.MAX_ANG_VEL)) {
                    telemetry.addLine("SHUTTING DOWN!!!");
                    requestOpModeStop();
                }
            }
        });

        int spikeMark = 0;
        boolean dropped = false;

        // Initialized, adjust values before start
        /*
        LIST OF CONFIG CONTROLS (so far) - Zachery:
        Driver 1 (gp1):

        (BUTTONS)
        Y - change alliance
        A - change side
        X - change park
        B - change pixel stack parameters

        (DPAD)
        up - change backboard wait time
        down - change spike wait time
        right - change back wait time
        left - change stack wait time

        (JOYSTICKS - IT WAS OUR LAST RESORT)
        left stick - change park wait time
        right stick -

        (BUMPERS)
        right bumper - increment slides height
        left bumper - decrement slides height

        (MISC.)
        START - once to drop, again to pick up again
        BACK - toggle to backboard
         */
        while (!isStarted()) {
            gp1.readButtons();

            // Re-grip/pick up pixel
            if (gp1.wasJustPressed(GamepadKeys.Button.START)){
                if (!dropped){ // drop
                    bot.claw.fullOpen();
                    dropped = true;
                } else { // re-grip
                    bot.slides.runToBottom();
                    bot.claw.fullOpen();
                    sleep(100);
                    bot.fourbar.pickup();
                    sleep(400);
                    bot.claw.pickupClose();
                    sleep(300);
                    bot.storage();
                    dropped = false;
                }
            }

            // Change alliance
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
            }
            // Change side
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (side == Side.CLOSE) { // Rn close, switch to FAR
                    side = Side.FAR;
                    pixelStack = false;
                } else { // Rn far, switch to CLOSE
                    side = Side.CLOSE;
                    slidesPos = 0;
                }
            }
            telemetry.addData("Alliance (Y)", alliance + " Side (A)" + side);

            // Toggle to pixel stack
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                pixelStack = !pixelStack;
            }
            telemetry.addData("PixelStack (B)", pixelStack);

            // CHANGE SLIDES HEIGHT
            // Increment
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                slidesPos += slidesPos < 1000 ? 200 : 0;
            }
            // Decrement
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                slidesPos -= slidesPos > 0 ? 200 : 0;
            }
            telemetry.addData("Slides (BUMPER)", slidesPos);

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
                if (backboardWait < 10.0) backboardWait+=1.0;
                else backboardWait = 0.0;
            }
            // Change spike mark wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                if (spikeWait < 10.0) spikeWait+=1.0;
                else spikeWait = 0.0;
            }
            // Change back wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                if (backWait < 10.0) backWait+=1.0;
                else backWait = 0.0;
            }
            // Change stack wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (stackWait < 10.0) stackWait+=1.0;
                else stackWait = 0.0;
            }
            // Change park wait time
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                if (parkWait < 10.0) parkWait+=1.0;
                else parkWait = 0.0;
            }
            telemetry.addData("DELAYS-Backboard(UP)", backboardWait + " Spike(DOWN): " + spikeWait + "Stack(LEFT): " + stackWait + " Back(RIGHT): " + backWait + " Park(LEFTSTICK): " + parkWait);

            // Toggle to backboard
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                toBackboard = !toBackboard;
            }
            telemetry.addData("ToBackboard (BACK)", toBackboard);

//            // Tune center tape min height
//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//                int height = colorDetection.getCenterTapeHeight() > 100 ? 10 : (colorDetection.getCenterTapeHeight()+1);
//                colorDetection.setCenterTapeHeight(height);
//            }
//            telemetry.addData("CenterTapeHeight (LEFT)", colorDetection.getCenterTapeHeight());

            // Initiate color detection
            if (alliance == Alliance.BLUE) {
                colorDetection.setAlliance(1);
            } else if (alliance == Alliance.RED) {
                colorDetection.setAlliance(2);
            }
            telemetry.addData("Current Camera FPS", camera.getFps());


            spikeMark = colorDetection.getSpikeMark();
            String spikeMarkString = "";
            switch (spikeMark) {
                case 1: spikeMarkString = "Left"; break;
                case 2: spikeMarkString = "Center"; break;
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
        } catch (OpenCvCameraException ignored) { }

        waitForStart();

        // Auto start
        if (opModeIsActive() && !isStopRequested()) {
            trackTime.start();
            stopAuto.start();

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

            periodic.start();

            // Drive to spike mark
            Pose2d startPose = drive.getPoseEstimate();
            Pose2d spikePose = new Pose2d();
            if (alliance == Alliance.BLUE) {
                if (side == Side.CLOSE) {
                    switch (spikeMark) {
                        case 1: spikePose = new Pose2d(34, 32, Math.toRadians(0)); break;
                        case 2: spikePose = new Pose2d(23, 32, Math.toRadians(60)); break;
                        case 3: spikePose = new Pose2d(20, 32, Math.toRadians(0)); break;
                    }
                } else if (side == Side.FAR) {
                    switch (spikeMark) {
                        case 1: spikePose = new Pose2d(-40, 34, Math.toRadians(180)); break;
                        case 2: spikePose = new Pose2d(-50, 22, Math.toRadians(180)); break;
                        case 3: spikePose = new Pose2d(-41,41, Math.toRadians(60)); break;
                    }
                }
            } else if (alliance == Alliance.RED) {
                if (side == Side.CLOSE) {
                    switch (spikeMark) {
                        case 1: spikePose = new Pose2d(20, -32, Math.toRadians(0)); break;
                        case 2: spikePose = new Pose2d(23, -32, Math.toRadians(-60)); break;
                        case 3: spikePose = new Pose2d(34, -32, Math.toRadians(0)); break;
                    }
                } else if (side == Side.FAR) {
                    switch (spikeMark) {
                        case 1: spikePose = new Pose2d(-41,-41, Math.toRadians(-60)); break;
                        case 2: spikePose = new Pose2d(-50, -22, Math.toRadians(180)); break;
                        case 3: spikePose = new Pose2d(-40, -34, Math.toRadians(180)); break;
                    }
                }
            }

            // Re-alignment code to avoid hitting truss
            Vector2d adjustVector = new Vector2d();
            boolean hasAdjust = false;
            if (alliance == Alliance.BLUE) {
                if (side == Side.CLOSE && spikeMark == 3) {
                    adjustVector = new Vector2d(13, 32); hasAdjust = true;
                }
                else if (side == Side.FAR && spikeMark == 1) {
                    adjustVector = new Vector2d(-35, 35); hasAdjust = true;
                }
            } else if (alliance == Alliance.RED) {
                if (side == Side.CLOSE && spikeMark == 1) {
                    adjustVector = new Vector2d(12, -32); hasAdjust = true;
                }
                else if (side == Side.FAR && spikeMark == 3) {
                    adjustVector = new Vector2d(-35, -35); hasAdjust = true;
                }
            }
            bot.outtakeGround(); // Go to outtake ground before trajectory
            bot.fourbar.setArm(0.1);
            if (hasAdjust) { // Avoid hitting truss
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                        .waitSeconds(spikeWait) // Wait before going to spike mark
                        .lineToLinearHeading(spikePose) // Line to position
                        .lineTo(adjustVector) // Line to spike mark
                        .build());
            } else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                        .waitSeconds(spikeWait) // Wait before going to spike mark
                        .lineToLinearHeading(spikePose) // Line to spike mark
                        .build());
            }
            bot.fourbar.setArm(0.0);

            // Score purple pixel
            bot.claw.halfOpen();
            sleep(300);
            bot.outtakeOut(1);
            sleep(100);
            bot.storage();
            bot.claw.close();
            sleep((long) (backboardWait * 1000));

            // Backstage actions
            if (toBackboard) {
                // To backboard
                int backboardY = 0;
                if (alliance == Alliance.BLUE) {
                    switch (spikeMark) {
                        case 1: backboardY = 41; break; // LEFT
                        case 2: backboardY = 37; break; // CENTER
                        case 3: backboardY = 29; break; // RIGHT
                    }
                } else if (alliance == Alliance.RED) {
                    switch (spikeMark) {
                        case 1: backboardY = -30; break; // LEFT
                        case 2: backboardY = -36; break; // CENTER
                        case 3: backboardY = -41; break; // RIGHT
                    }
                }

                startPose = drive.getPoseEstimate();
                if (side == Side.CLOSE) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                            //.waitSeconds(backboardWait)
                            .addDisplacementMarker(0.5, () -> { // Slides up
                                if (slidesPos != 0) bot.slides.runTo(-slidesPos);
                                else bot.slides.runToBottom();
                            })
                            .lineToLinearHeading(new Pose2d(51, backboardY, Math.toRadians(180))) // x value of Pose2d is how far away we are from backboard: lower the number - farther, closer the number - closer
                            .build());
                } else if (side == Side.FAR) {
                    if (alliance == Alliance.BLUE) {
                        switch (spikeMark) {
                            case 1: // LEFT
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(180)))
                                        .back(81) // Drive to backboard
                                        //.waitSeconds(backboardWait)
                                        .lineToLinearHeading(new Pose2d(51, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                            case 2: // CENTER
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .strafeLeft(13) // Strafe to center truss
                                        .back(96) // Drive to backboard
                                        //.waitSeconds(backboardWait) // Wait for close auto to finish
                                        .lineToLinearHeading(new Pose2d(51, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-34, 55, Math.toRadians(180))) // Back to start but with different heading
                                        .lineToLinearHeading(new Pose2d(-34, 12, Math.toRadians(180))) // To center truss
                                        .back(78) // Drive across field
                                        //.waitSeconds(backboardWait)
                                        .lineToLinearHeading(new Pose2d(51, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                        }
                    } else if (alliance == Alliance.RED) {
                        switch (spikeMark) {
                            case 1: // LEFT
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-34, -55, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(180)))
                                        .back(81) // Drive across field
                                        .lineToLinearHeading(new Pose2d(51, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                            case 2: // CENTER
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .strafeRight(13) // Strafe to center truss
                                        .back(96) // Drive to backboard
                                        //.waitSeconds(backboardWait) // Wait for close auto to finish
                                        .lineToLinearHeading(new Pose2d(51, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                            case 3: // RIGHT
                                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(180)))
                                        .back(78) // Drive to backboard
                                        //.waitSeconds(backboardWait)
                                        .lineToLinearHeading(new Pose2d(51, backboardY, Math.toRadians(180)))
                                        .build());
                                break;
                        }
                    }
                }

                // Score yellow pixel on backboard
                bot.autoOuttakeOut(1);
                bot.fourbar.setArm(0.21);
                sleep(700);
                bot.claw.setPosition(0.66); // extra open
                sleep(400);
                bot.fourbar.setWrist(0.65);
                sleep(200);
                // Return to storage
                bot.storage();
                bot.claw.close();

                // PIXEL STACK TRAJECTORY STARTS HERE
                if (side == Side.CLOSE && pixelStack) {
                    Thread block = new Thread(() -> {
                        sleep(100);
                        bot.fourbar.armBlock();
                        sleep(300);
                        bot.claw.setPosition(0.51);
                    });
                    block.start();

                    // If wait for stack drive to park location and then go
                    if (stackWait != 0.0) {
                        if (alliance == Alliance.BLUE) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .forward(4)
                                    .lineTo(new Vector2d(50, 11))
                                    .waitSeconds(stackWait)
                                    .build()
                            );
                        } else if (alliance == Alliance.RED) {
                            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                    .forward(4)
                                    .lineTo(new Vector2d(50, -11))
                                    .waitSeconds(stackWait)
                                    .build()
                            );
                        }
                    }

                    //bot.intake(true); // Reverse intake

                    // Drive across field
                    startPose = drive.getPoseEstimate();
                    if (alliance == Alliance.BLUE) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(35, 11, Math.toRadians(180))) // Line to center truss position
                                .lineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180))) // Through field
                                .build()
                        );
                    } else if (alliance == Alliance.RED) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(180))) // Line to center truss position
                                .lineToLinearHeading(new Pose2d(-59, -12, Math.toRadians(180))) // Through field
                                .build()
                        );
                    }
                    sleep(600);

                    // AT PIXEL STACK, intake pixels
                    //bot.intake(false);
                    if (alliance == Alliance.BLUE){
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(1)
                                .waitSeconds(1)
                                .back(2)
                                .waitSeconds(1)
                                .build()
                        );
                    } else if (alliance == Alliance.RED){
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(2)
                                .waitSeconds(1)
                                .back(3)
                                .waitSeconds(1)
                                .build()
                        );
                    }

//                    Thread reverseIntake = new Thread(() -> {
//                        bot.intake(true);
//                        sleep(300);
//                        bot.intake.stopIntake();
//                        sleep(500);
//                        bot.intake(false);
//                        sleep(1000);
//                    });
//                    reverseIntake.start();

                    Thread pixelTap = new Thread(() -> {
                        bot.slides.runToBottom();
                        bot.storage();
                        sleep(300);
                        bot.claw.fullOpen();
                        sleep(100);
                        bot.fourbar.pickup();
                        sleep(400);
                        bot.claw.pickupClose();
                        sleep(300);
                        bot.claw.fullOpen();
                        bot.storage();
                        sleep(600);
                        bot.claw.fullOpen();
                        sleep(100);
                        bot.fourbar.pickup();
                        sleep(400);
                        bot.claw.pickupClose();
                        sleep(300);
                        bot.claw.fullOpen();
                        bot.storage();
                    });
                    pixelTap.start();

                    // RETURN TO BACKBOARD
                    startPose = drive.getPoseEstimate();
                    if (alliance == Alliance.BLUE) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(34, 10, Math.toRadians(180))) // Drive across field
                                .waitSeconds(backWait)
                                .build()
                        );
                    } else if (alliance == Alliance.RED) {
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(34, -10, Math.toRadians(180))) // Drive across field
                                .waitSeconds(backWait)
                                .build()
                        );
                    }

                    Thread stackPickup = new Thread(() -> {
                        bot.intake.stopIntake();
                        bot.slides.runToBottom();
                        bot.claw.fullOpen();
                        sleep(100);
                        bot.fourbar.pickup();
                        sleep(500);
                        bot.claw.pickupClose();
                        sleep(400);
                        bot.storage();
                        sleep(200);
                    });
                    stackPickup.start();

                    // Line to backboard
                    int slowerVelocity = 20; // Slower velocity that is the max constraint when running into backboard (in/s)
                    startPose = drive.getPoseEstimate();
                    if (alliance == Alliance.BLUE) {
                        drive.followTrajectory(drive.trajectoryBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(52, 30, Math.toRadians(180)),
                                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                    } else if (alliance == Alliance.RED) {
                        drive.followTrajectory(drive.trajectoryBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(52, -32, Math.toRadians(180)),
                                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                    }

                    // Score pixels on backboard
                    // First pixel
                    bot.slides.runTo(-500.0); // Slides up
                    bot.outtakeOut(2);
                    bot.fourbar.setArm(0.21);
                    sleep(500);
                    bot.claw.halfOpen();
                    sleep(300);
                    // Second pixel
                    bot.slides.runTo(-700.0); // Slides up
                    bot.outtakeOut(1);
                    sleep(300);
                    bot.claw.setPosition(0.66); // extra open
                    sleep(400);
                    // Return to storage
                    bot.storage();
                    bot.claw.close();
                }
                // END OF PIXEL STACK TRAJECTORY

                // Parking
                startPose = drive.getPoseEstimate();
                int parkY = 0;
                if (alliance == Alliance.BLUE) {
                    if (park == 1) parkY = 59; else if (park == 2) parkY = 11;
                } else if (alliance == Alliance.RED) {
                    if (park == 1) parkY = -11;else if (park == 2) parkY = -59;
                }
                if (park != 0) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                            .waitSeconds(parkWait)
                            .splineToLinearHeading(new Pose2d(52, parkY, Math.toRadians(180)),Math.toRadians(15))
                            .build()
                    );
//                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
//                            .forward(4)
//                            .lineTo(new Vector2d(50, parkY))
//                            .build()
//                    );
//                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).back(2).build()); // Go forward into parking spot
                }


            }

            // Stop op mode
            sleep(500);
            PoseStorage.currentPose = drive.getPoseEstimate();
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