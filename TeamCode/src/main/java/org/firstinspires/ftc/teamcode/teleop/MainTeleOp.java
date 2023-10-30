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

@Config
@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private boolean autoAlignForward = false;
    private double driveSpeed = 1;
    private GamepadEx gp1, gp2;
    public static double kp = 0.025, ki = 0, kd = 0;
    private PIDController headingAligner = new PIDController(kp, ki, kd);
    private final int manualSlideAmt = 1;
    double leftX = gp2.getLeftX(), rightX = gp2.getRightX(), leftY = gp2.getLeftY(), rightY = gp2.getRightY();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        // Initialize bot
        bot.state = Bot.BotState.STORAGE;
        bot.storage();

        /*
        COMPLETE LIST OF DRIVER CONTROLS (so far):
        Driver 1 (gp1):
        joysticks - driving
        right trigger - slow down
        left bumper - run intake
        right bumper - run reverse intake
        back - auto align
        Driver 2 (gp2):
        A - pick up pixel (top)
        B - pick up pixel (bottom)
        Y - discard pixel
        X - toggle between claw positions
        dpad up - slides to top
        dpad right - slides to middle
        dpad down - slides to bottom
        right bumper - manual position slides up
        left bumper - manual position slides down
        */
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            headingAligner.setPID(kp, ki, kd);
            gp1.readButtons();
            gp2.readButtons();

            // FINITE STATES
            if (bot.state == Bot.BotState.STORAGE) { // INITIALIZED
                // INTAKE (driver 1)
                if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) { // intake
                    bot.intake(false);
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { // reverse intake
                    bot.intake(true);
                }

                // TRANSFER
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // top pixel
                    bot.pickup(1);
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) { // bottom pixel
                    bot.pickup(2);
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // go to outtake out position
                    bot.outtakeOut();
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) { // go to outtake ground position
                    bot.outtakeDown();
                }
            } else if (bot.state == Bot.BotState.OUTTAKE_OUT) { // SCORING BACKBOARD
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // drop and return to storage
                    bot.drop();
                }
            } else if (bot.state == Bot.BotState.OUTTAKE_DOWN) { // SCORING GROUND
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) { // drop and return to storage
                    bot.drop();
                }
            }
            //DRIVING FSM kept separate from other fsm part because didnt want to put gp1drive in every other state in case something changes or smt unexpected happens

            if (bot.state == Bot.BotState.OUTTAKE_OUT) {
                gp2strafe();
            } else {
                gp1drive();
            }

            // SLIDES
            // preset positions
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slides.runToTop();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slides.runToMiddle();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.runToBottom();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slides.runToLow();
            }
            // manual slides positioning with joystick
            if (gp2.getLeftY() > 0.1) {
                bot.slides.runManual(leftY*0.5);
            }

            // OTHER
            // auto align
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.resetIMU();
                autoAlignForward = !autoAlignForward;
            }
            telemetry.update();
            bot.slides.periodic();
            //gp1drive(); put in fsm of outtake out -> gp2strafe
        }
    }

    private void gp1drive() { // all directions
        driveSpeed = 1;
        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);

        bot.drive(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed / 1.7
        );
        if (autoAlignForward) {
            double power = headingAligner.calculate(bot.getIMU());
            bot.drive(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    -power
            );
        } else {
            bot.drive(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed / 1.7
            );
        }
    }
    private void gp2strafe() { // strafing left right
        driveSpeed = 0.1;
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();

        Vector2d driveVector = new Vector2d(gp2.getLeftX(), -gp2.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);

        bot.drive(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed / 1.7
        );
        if (autoAlignForward) {
            double power = headingAligner.calculate(bot.getIMU());
            bot.drive(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    -power
            );
        } else {
            bot.drive(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed / 1.7
            );
        }
    }

}
