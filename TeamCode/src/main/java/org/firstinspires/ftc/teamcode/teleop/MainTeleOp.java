package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.MainAutonomous;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Slides;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;

import java.lang.*;
import java.util.Map;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1;
    private GamepadEx gp1, gp2;
    double leftX, rightX, leftY, rightY;
    Thread thread;

    double ikCoefficient = 1;

    // PID
    //public static double kp = 0.025, ki = 0, kd = 0;
    //private PIDController headingAligner = new PIDController(kp, ki, kd);

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(bot.getAutoEndPose());

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        leftX = gp2.getLeftX();
        rightX = gp2.getRightX();
        leftY = gp2.getLeftY();
        rightY = gp2.getRightY();

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
        left bumper - run intake
        right bumper - run reverse intake
        start - align robot

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
            Pose2d poseEstimate = drive.getPoseEstimate();

            gp1.readButtons();
            gp2.readButtons();

            // FINITE STATES
            if (bot.state == Bot.BotState.STORAGE) { // INITIALIZED
                // TRANSFER
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // fix bottom pixel
                    thread = new Thread(() -> {
                        bot.slides.runToBottom();
                        bot.claw.fullOpen();
                        sleep(100);
                        bot.fourbar.bottomPixel();
                        sleep(400);
                        bot.claw.pickupClose();
                        sleep(300);
                        bot.claw.fullOpen();
                        bot.storage();
                    });
                    thread.start();
                } else if(gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    thread = new Thread(() -> {
                        bot.slides.runToBottom();
                        bot.claw.fullOpen();
                        sleep(100);
                        bot.fourbar.topPixel();
                        sleep(400);
                        bot.claw.pickupClose();
                        sleep(300);
                        bot.claw.fullOpen();
                        bot.storage();
                    });
                    thread.start();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) { // pickup pixel
                    thread = new Thread(() -> {
                        bot.slides.runToBottom();
                        bot.claw.fullOpen();
                        sleep(100);
                        bot.fourbar.pickup();
                        sleep(400);
                        bot.claw.pickupClose();
                        sleep(300);
                        bot.storage();
                    });
                    thread.start();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { // drop pixel while in storage
                    sleep(100);
                    bot.claw.open();
                    sleep(100);
                    bot.fourbar.storage();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // go to outtake out position
                    bot.outtakeOut(bot.claw.getClawState());
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) { // go to outtake ground position
                    bot.outtakeGround();
                }
            } else if (bot.state == Bot.BotState.OUTTAKE_OUT) { // SCORING BACKBOARD
                bot.slides.runManual(gp2.getRightY()*-0.5);
//                if (Math.abs(gp2.getRightY()) > 0.001) {
//                    bot.fourbar.runAngle(bot.slides.motorLeft.getCurrentPosition()); // calculate arm angle
//                }

                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // drop and return to storage
                    drop();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) { // cancel and return to storage
                    bot.storage();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) { // go to outtake ground position
                    bot.outtakeGround();
                }
            } else if (bot.state == Bot.BotState.OUTTAKE_DOWN) { // SCORING GROUND
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    dropGround();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // cancel and return to storage
                    bot.storage();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // go to outtake out position
                    bot.outtakeOut(bot.claw.getClawState());
                }
            }

            // SLIDES
            // manual slides positioning with joystick
            bot.slides.runManual(gp2.getLeftY()*-0.5);

            // preset positions
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { // TOP
                bot.claw.close();
                bot.slides.runToTop();
                thread = new Thread(() -> {
                    sleep(900);
                    bot.outtakeOut((bot.claw.getClawState()));
                });
                thread.start();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { // MIDDLE
                bot.claw.close();
                bot.slides.runToMiddle();
                thread = new Thread(() -> {
                    sleep(500);
                    bot.outtakeOut((bot.claw.getClawState()));
                });
                thread.start();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { // LOW
                bot.claw.close();
                bot.slides.runToLow();
                bot.outtakeOut(bot.claw.getClawState());
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { // BOTTOM
                bot.slides.runToBottom();
                bot.storage();
            }

            // DRIVE
            gp1drive();

            // INTAKE (driver 1)
            if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) { // intake
                bot.intake(false);
            } else if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { // reverse intake
                bot.intake(true);
            } else {
                bot.intake.stopIntake();
            }

            // LAUNCH DRONE
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                bot.launch();
            }

            // Fourbar/arm kinematics
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                if (bot.slides.getPosition() < -630 ){
                    bot.ikDemo1();
                }
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                if (bot.slides.getPosition() > - 1700){
                    bot.ikDemo2();
                }
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                bot.intake.power = bot.intake.power - 0.01;
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                bot.intake.power = bot.intake.power + 0.01;
            }
            inverseKinematics(gp2.getRightY(), ikCoefficient);

            // Alignment
            if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                double angle = Math.toRadians(90);
                drive.turnAsync(Angle.normDelta(angle - poseEstimate.getHeading()));
            }

            // TELEMETRY
            telemetry.addData("Bot State",bot.state);
            telemetry.addData("Intake Power", bot.intake.power +"(running=" + bot.intake.getIsRunning() + ")");
            telemetry.addData("Slides Position", bot.slides.getPosition() + " (pos=" + bot.slides.position + " current=" + bot.slides.getCurrent() + ")");
            telemetry.addData("Pixels", bot.claw.getClawState());
            telemetry.addData("Arm Position", bot.fourbar.getArmPosition());
            //telemetry.addData("IK Coefficent", ikCoefficient );

            telemetry.update();
            bot.slides.periodic();
        }
    }

    // Drop pixel thread
    private void drop() {
        thread = new Thread(() -> {
            bot.claw.open();
            sleep(300);
            if (bot.state == Bot.BotState.OUTTAKE_OUT) {
                if (bot.claw.getClawState() == 0) {
                    bot.storage();// if claw is empty go to storage
                } else if (bot.claw.getClawState() == 1) {
                    if (bot.slides.getPosition() > -1850) {
                        bot.slides.runTo(bot.slides.getPosition() - 250);
                    } else if (bot.slides.getPosition() <=-1850){
                        bot.slides.runTo(-2300);
                    }
                    bot.fourbar.topOuttake();
                }
            } else {
                bot.storage();
            }
            //bot.claw.close();
        });
        thread.start();
    }

    private void dropGround() {
        thread = new Thread(() -> {
            bot.claw.open();
            sleep(300);
            if (bot.state == Bot.BotState.OUTTAKE_DOWN) {
                if (bot.claw.getClawState() == 0) {
                    bot.storage();// if claw is empty go to storage
                } else if (bot.claw.getClawState() == 1) {
                    bot.fourbar.ground();
                }
            } else {
                bot.storage();
            }
            //bot.claw.close();
        });
        thread.start();
    }

    public void inverseKinematics(double manual, double ikCoefficient) {
        //bot.slides.runManual(ikCoefficient * -manual * -0.5);//-0.5 and coefficient and - on the manual arent doing anything
            bot.fourbar.runArm(manual);

    }

    // Driving
    private void gp1drive() { // Driver 1
        driveSpeed = 1 - 0.8 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(gp1.getRightX(), 0);

        bot.drive(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed
        );
    }

    private void gp2strafe() { // strafing left/right, no turning or forward/backward
        driveSpeed = 0.25; // strafing speed for driver 2 to adjust when scoring
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();

        Vector2d driveVector = new Vector2d(-gp2.getLeftX(), -gp2.getRightY());

        bot.drive(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                0.0
        );
    }

}
