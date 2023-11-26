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
    public static double kp = 0.025, ki = 0, kd = 0;
    //private PIDController headingAligner = new PIDController(kp, ki, kd);
    private final int manualSlideAmt = 1;
    double leftX, rightX, leftY, rightY;
    Thread thread;
    private boolean hasPixel = false;

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
        Y - discard pixel
        X - toggle between claw positions
        dpad up - slides to top
        dpad right - slides to middle
        dpad left - slides to low
        dpad down - slides to bottom
        right bumper - manual position slides up
        left bumper - manual position slides down
        */

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            // FINITE STATES
            if (bot.state == Bot.BotState.STORAGE) { // INITIALIZED
                // TRANSFER
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // top pixel
                    if (!hasPixel) { // pick up
                        hasPixel = true;
                        thread = new Thread(() -> {
                            bot.slides.runToBottom();
                            bot.claw.open();
                            sleep(100);
                            bot.fourbar.topPixel();
                            sleep(400);
                            bot.claw.close();
                            sleep(300);
                            bot.storage();
                        });
                        thread.start();
                    } else { // has pixel in claw, drop
                        hasPixel = false;
                        bot.claw.open();
                    }
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) { // bottom pixel
                    if (!hasPixel) { // pick up
                        hasPixel = true;
                        thread = new Thread(() -> {
                            bot.slides.runToBottom();
                            bot.claw.open();
                            sleep(100);
                            bot.fourbar.bottomPixel();
                            sleep(400);
                            bot.claw.close();
                            sleep(300);
                            bot.storage();
                        });
                        thread.start();
                    } else {
                        hasPixel = false;
                        bot.fourbar.dropPixel(2);
                        sleep(100);
                        bot.claw.open();
                        sleep(100);
                        bot.fourbar.storage();
                    }
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // go to outtake out position
                    bot.outtakeOut();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) { // go to outtake ground position
                    bot.outtakeGround();
                }
            } else if (bot.state == Bot.BotState.OUTTAKE_OUT) {
                // SCORING BACKBOARD

                bot.slides.runManual(gp2.getLeftY()*-0.5); //pls work
                //bot.fourbar.runAngle(bot.slides.motorLeft.getCurrentPosition());

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
                    bot.outtakeOut();
                }
            }

            // SLIDES
            // manual slides positioning with joystick
            bot.slides.runManual(gp2.getLeftY()*-0.5);
            // preset positions
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { // TOP
                bot.claw.close();
                bot.outtakeOut();
                bot.slides.runToTop();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { // MIDDLE
                bot.claw.close();
                bot.outtakeOut();
                bot.slides.runToMiddle();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { // LOW
                bot.claw.close();
                bot.outtakeOut();
                bot.slides.runToLow();
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

            telemetry.update();
            bot.slides.periodic();
        }
    }

    // Drop pixel thread
    private void drop() {
        hasPixel = false;
        thread = new Thread(() -> {
            bot.claw.open();
            sleep(300);
            bot.storage();
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
