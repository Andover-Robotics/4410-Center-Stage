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

    private double driveSpeed = 1;
    private int clawPos = 1;
    private GamepadEx gp1, gp2;
    public static double kp = 0.025, ki = 0, kd = 0;
    private PIDController headAlign = new PIDController(kp, ki, kd);
    private final int manualSlideAmt = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        // Initialize bot
        bot.state = Bot.BotState.INITIALIZED;
        bot.initialized();

        /*
        COMPLETE LIST OF DRIVER CONTROLS (so far):
        Driver 1 (gp1):
        joysticks - driving
        right trigger - slow down
        left bumper - run intake
        right bumper - run reverse intake
        Driver 2 (gp2):
        A - pick up pixel
        Y - discard pixel
        B - outtake (open claw)
        X - toggle between claw positions
        dpad up - slides to top
        dpad right - slides to middle
        dpad down - slides to bottom
        right bumper - manual position slides up
        left bumper - manual position slides down
         */

        while (opModeIsActive() && !isStopRequested()) {

            headAlign.setPID(kp, ki, kd);
            gp1.readButtons();
            gp2.readButtons();

            // FINITE STATES
            if (bot.state == Bot.BotState.INITIALIZED) { // INITIALIZED
                // INTAKE (driver 1)
                if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) { // intake
                    bot.intake(false);
                }
                if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { // reverse intake
                    bot.intake(true);
                }

                // TRANSFER (driver 2 from here on)
                if (gp2.wasJustPressed(GamepadKeys.Button.A)) { // pick up
                    bot.pickup(clawPos);
                }
            } else if (bot.state == Bot.BotState.SCORE) { // SCORING POSITION
                if (gp2.wasJustPressed(GamepadKeys.Button.Y)) { // discard pixel
                    bot.discardPixel();
                }

                // OUTTAKE
                if (gp2.wasJustPressed(GamepadKeys.Button.B)) { // open claw then return to initialized position
                    bot.outtake();
                    bot.initialized(); // return to init position
                }
            }

            // SLIDES
            // preset positions
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slides.runToTop();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slides.runToMiddle();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.runToLow();
            }
            // manual positioning
            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.slides.runTo(bot.slides.getPosition() + manualSlideAmt);
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.slides.runTo(bot.slides.getPosition() - manualSlideAmt);
            }

            // CLAW POSITIONS
            if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                switch (clawPos) {
                    case 1: clawPos = 2; break; // top pixel
                    case 2: clawPos = 1; break; // bottom pixel
                }
            }

            // OTHER
            telemetry.update();
            bot.slides.periodic();
            drive();
        }
    }

    private void drive() {
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
    }

}
