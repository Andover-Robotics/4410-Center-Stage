package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Slides;

import java.lang.*;

@Config
@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private boolean autoAlignForward = false;
    private double driveSpeed = 1;
    private GamepadEx gp1, gp2;
    public static double kp = 0.025, ki = 0, kd = 0;
    private PIDController headingAligner = new PIDController(kp, ki, kd);
    private final int manualSlideAmt = 1;
    double leftX, rightX, leftY, rightY;
    Thread thread;

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

            //headingAligner.setPID(kp, ki, kd);
            gp1.readButtons();
            gp2.readButtons();

            // FINITE STATES
            if (bot.state == Bot.BotState.STORAGE) { // INITIALIZED
                // TRANSFER
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // top pixel
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
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) { // bottom pixel
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
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // go to outtake out position
                    bot.outtakeOut();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) { // go to outtake ground position
                    bot.outtakeGround();
                }
            } else if (bot.state == Bot.BotState.OUTTAKE_OUT) { // SCORING BACKBOARD
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
            }

            // DRIVE
//            if (bot.state == Bot.BotState.OUTTAKE_OUT) {
//                gp2strafe();
//            } else {
//                gp1drive();
//            }
            gp1drive();

            // INTAKE (driver 1)
            if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) { // intake
                bot.intake(false);
            } else if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { // reverse intake
                bot.intake(true);
            } else {
                bot.intake.stopIntake();
            }

            // AUTO ALIGN
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.resetIMU();
                autoAlignForward = !autoAlignForward;
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
        thread = new Thread(() -> {
            bot.claw.open();
            sleep(250);
            bot.claw.close();
            bot.storage();
        });
        thread.start();
    }

    // Drving
    private void gp1drive() { // Driver 1
        driveSpeed = 1;
        driveSpeed *= 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(

                        gp1.getRightX(), 0);

        if (autoAlignForward) { // with auto align
            double power = headingAligner.calculate(bot.getIMU());
            bot.drive(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    -power
            );
        } else { // without auto align
            bot.drive(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        }
    }

//    private void gp2strafe() { // strafing left/right, no turning or forward/backward
//        driveSpeed = 0.25; // strafing speed for driver 2 to adjust when scoring
//        driveSpeed = Math.max(0, driveSpeed);
//        bot.fixMotors();
//
//        Vector2d driveVector = new Vector2d(-gp2.getLeftX(), -gp2.getRightY());
//
//        bot.drive(driveVector.getX() * driveSpeed,
//                driveVector.getY() * driveSpeed,
//                0.0
//        );
//        if (autoAlignForward) {
//            double power = headingAligner.calculate(bot.getIMU());
//            bot.drive(driveVector.getX() * driveSpeed,
//                    driveVector.getY() * driveSpeed,
//                    -power
//            );
//        } else {
//            bot.drive(driveVector.getX() * driveSpeed,
//                    driveVector.getY() * driveSpeed,
//                    0.0
//            );
//        }
//    }

}
