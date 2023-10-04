package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.Math;

@Config
@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, cycleTime = 1, turretslidespeed = 1;

    private boolean debugMode = false;
    private boolean cancelPrevAction = false, autoAlignForward = false, autoMode = false, isRight = false;
    private int index = 4;
    public static double kp = 0.025, ki = 0, kd = 0;

    private PIDController headingAligner = new PIDController(kp, ki, kd);

    Thread thread, otherThread; // TODO: implement threads(later if neccessary in the future)

    private GamepadEx gp1, gp2;

    @Override
    public void runOpMode() {

        headingAligner.setTolerance(1);
        headingAligner.setSetPoint(0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);
        waitForStart();

        bot.ready_pos();
        while (opModeIsActive() && !isStopRequested()) {
            headingAligner.setPID(kp, ki, kd);
            telemetry.addData("cycle", time - cycleTime);
            cycleTime = time;

            gp1.readButtons();
            gp2.readButtons();

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.toodles.runIntake();
            } else {
                bot.toodles.stopIntake();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.toodles.runReverseIntake();
            } else {
                bot.toodles.stopIntake();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.outtake_pickup(1);
                bot.outtake_ground();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.outtake_pickup(1);
                bot.outtake_backboard(3);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.outtake_pickup(1);
                bot.outtake_backboard(2);
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.outtake_pickup(1);
                bot.outtake_backboard(1);
            }

            if (bot.botState == Bot.BotState.OUTTAKE) {
                double upPower = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                double downPower = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

                if (upPower != 0 && downPower !=0) {
                    bot.slides.runManual((upPower+downPower)/2.0);
                } else if (upPower == 0) {
                    bot.slides.runManual(downPower);
                } else {
                    bot.slides.runManual(upPower);
                }

                if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                    bot.claw.toggle();
                }
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.X)) { // for closing for travel underneath bars or such
                bot.ready_pos();
            }

            telemetry.update();

            bot.slides.periodic();
            drive();
        }
    }


    private void drive() {
        driveSpeed = 1;
//        driveSpeed -= bot.slides.isHigh()/3;
        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();
        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);
        bot.drive(driveVector.x * driveSpeed,
                driveVector.y * driveSpeed,
                turnVector.x * driveSpeed / 1.7
        );
    }

}
