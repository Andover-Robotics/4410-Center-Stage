package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.MainAutonomous;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.auto.util.PoseStorage;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Slides;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.*;
import java.util.Map;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1;
    private GamepadEx gp1, gp2;
    private boolean fieldCentric = false;
    private Thread thread;
    DistanceSensor distanceSensor;
    DigitalChannel breakBeam;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        // Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");
        breakBeam = hardwareMap.get(DigitalChannel.class, "BreakBeam");


        // Initialize bot
        bot.stopMotors();
        bot.state = Bot.BotState.STORAGE;
        bot.storage();
        bot.claw.fullOpen();

        /*
        LIST OF DRIVER CONTROLS (so far) - Zachery:

        Driver 1 (gp1):
        joysticks - driving
        right trigger - slow down
        start - toggle field centric

        Y - manual intake
        left bumper - run intake
        right bumper - run reverse intake

        Driver 2 (gp2):
        B - pick up pixels, cancel outtake out
        A - tap pixel bottom, cancel outtake ground
        Y - outtake out
        X - outtake ground

        dpad up - slides to top
        dpad left - slides to middle
        dpad right - slides to low
        dpad down - slides to bottom

        right bumper - drop pixel
        left bumper - tap pixel top

        left joystick - slides up/down
        right joystick - arm in/out
        */

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            gp1.readButtons();
            gp2.readButtons();

            // FINITE STATES
            if (bot.state == Bot.BotState.STORAGE) { // INITIALIZED
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // fix pixels
                    bot.fixPixels();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) { // pickup pixel
                    if (breakBeam.getState()) {
                        bot.pickup(true);
                    } else {
                        bot.pickup();
                    }
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { // drop pixel while in storage
                    bot.drop();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // go to outtake out position
                    bot.outtakeOut(bot.claw.getClawState());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) { // go to outtake ground position
                    bot.outtakeGround();
                }
            } else if (bot.state == Bot.BotState.OUTTAKE_OUT) { // SCORING BACKBOARD
                bot.slides.runManual(gp2.getRightY()*-0.5);
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // drop
                    bot.drop();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) { // cancel and return to storage
                    bot.storage() ;
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    bot.outtakeGround();
                }
//                if (Math.abs(gp2.getLeftY()) > 0.01) {
//                    bot.fourbar.runAngle(bot.slides.motorLeft.getCurrentPosition());
//                }
            } else if (bot.state == Bot.BotState.OUTTAKE_DOWN) { // SCORING GROUND
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    bot.drop();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // cancel and return to storage
                    bot.storage();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // go to outtake out position
                    bot.outtakeOut(bot.claw.getClawState());
                }
            }

            // Distance sensor
            if (distanceSensor.getDistance(DistanceUnit.CM) < 30) { // Slows down driving when approaching object
                driveMultiplier = 0.3;
                if ((gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2))
                {
                    driveMultiplier = 0.6;
                }
            } else {
                driveMultiplier = 1;
            }

            // SLIDES
            // manual slides positioning with joystick
            bot.slides.runManual(gp2.getLeftY()*-0.5);
            // preset positions
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { // top
                bot.presetSlides(4);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { // middle
                bot.presetSlides(3);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { // low
                bot.presetSlides(2);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { // bottom
                bot.presetSlides(1);
            }

            // DRIVE
            drive();
            if (gp1.wasJustPressed(GamepadKeys.Button.START)) { // toggle field/robot centric
                fieldCentric = !fieldCentric;
            }
            if (gp1.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) { // reset heading
                drive.setPoseEstimate(PoseStorage.currentPose);
            }
//            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) { // auto align????
//                drive.setPoseEstimate(PoseStorage.currentPose);
//                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .splineToSplineHeading(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY()+1, drive.getPoseEstimate().getHeading()), Math.toRadians(180))
//                        .build());
//            }

            // INTAKE
            if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { // reverse intake
                bot.intake(true, bot.intake.intakeOut);
            } else if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER) && breakBeam.getState()) { // intake
                bot.intake(false, bot.intake.intakeOut);
            } else if (gp1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON) && breakBeam.getState()){ // up intake
                bot.intake(false, bot.intake.intakeUp);

            } else if (gp1.isDown(GamepadKeys.Button.Y)){ // manual intake, ignores break beams
                bot.intake(false, bot.intake.intakeOut);

            } else { // stop intake
                bot.intake.stopIntake();
                bot.intake.setIntakeHeight(bot.intake.intakeStorage);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { // increment power
                bot.intake.changePower(true);
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { // decrement power
                bot.intake.changePower(false);
            }

            // DRONE
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) { // launch drone
                bot.launcher.launch();
            } else if (gp1.wasJustPressed(GamepadKeys.Button.A)) { // dock drone
                bot.launcher.reset();
            }

            bot.fourbar.runArm(gp2.getRightY());

            // TELEMETRY
            telemetry.addData("Field Centric",fieldCentric + " Heading: " + Math.toDegrees(drive.getPoseEstimate().getHeading()));;

            telemetry.addData("Bot State",bot.state);
            telemetry.addData("Pixels", bot.claw.getClawState());
            telemetry.addData("Slides Position", bot.slides.getPosition() + " (pos: " + bot.slides.position + " current: " + bot.slides.getCurrent() + ")");

            telemetry.addData("Intake Power", Intake.power +"(running: " + bot.intake.getIsRunning() + ")");
            telemetry.addData("arm Left Position", bot.fourbar.armLeft.getPosition());
            telemetry.addData("arm Right Position", bot.fourbar.armRight.getPosition());
            telemetry.addData("wrist Position", bot.fourbar.wrist.getPosition());
            telemetry.addData("Distance to wall (cm)", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Break Beam", breakBeam.getState());

            telemetry.addData("X", gp1.getLeftX());
            telemetry.addData("Y", gp1.getLeftY());
            telemetry.addData("RX", gp1.getRightX());

            telemetry.update();
            bot.slides.periodic();
            bot.setHeading(drive.getPoseEstimate().getHeading());
        }
    }

    // Driving
    private void drive() { // Robot centric, drive multiplier default 1, 1/2 when distance sensor
        driveSpeed = driveMultiplier - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();

        if (fieldCentric) {
            Vector2d driveVector = new Vector2d(-gp1.getLeftX(), -gp1.getLeftY()),
                    turnVector = new Vector2d(-gp1.getRightX(), 0);
            bot.driveFieldCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        } else {
            Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                    turnVector = new Vector2d(gp1.getRightX(), 0);
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        }
    }

}