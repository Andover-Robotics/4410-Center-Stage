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

import java.lang.*;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1;
    private GamepadEx gp1, gp2;
    double leftX, rightX, leftY, rightY;
    Thread thread;

    // PID
    public static double kp = 0.025, ki = 0, kd = 0;
    private PIDController headingAligner = new PIDController(kp, ki, kd);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

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

        Driver 2 (gp2):
        A - pick up pixel (top)
        B - pick up pixel (bottom)
        dpad up - slides to top
        dpad left - slides to middle
        dpad right - slides to low
        dpad down - slides to bottom
        right bumper - drop pixel
        left joystick - slides up/down
        right joystick - arm in/out
        */

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            // FINITE STATES
            if (bot.state == Bot.BotState.STORAGE) { // INITIALIZED
                // TRANSFER
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // pickup pixel
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
                    bot.claw.fullOpen();
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
                //bot.fourbar.runAngle(bot.slides.motorLeft.getCurrentPosition()); // calculate arm angle
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
                    drop();
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

            // TELEMETRY
            telemetry.addData("Bot State",bot.state);
            telemetry.addData("Intake Power", bot.intake.power +"(running=" + bot.intake.getIsRunning() + ")");
            telemetry.addData("Slides Position", bot.slides.getPosition() + " (pos=" + bot.slides.position + " current=" + bot.slides.getCurrent() + ")");
            telemetry.addData("Pixels", bot.claw.getClawState());

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
                        bot.slides.runTo(bot.slides.getPosition() - 400);
                    }
                    bot.fourbar.wristTopOuttake();
                }
            } else {
                bot.storage();
            }
            bot.claw.close();
        });
        thread.start();
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
