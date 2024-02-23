package org.firstinspires.ftc.teamcode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.auto.TestAutonomous;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

// This class was made to demonstrate how the break beam prevents intake from running when there is a pixel

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    private Bot bot;
    private GamepadEx gp1;
    DigitalChannel breakBeam;

    private double intakeTime = 0.0;
    private boolean stopIntake = false;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        breakBeam = hardwareMap.get(DigitalChannel.class, "BreakBeam");

        bot.intake.setIntakeHeight(0.38);

        Thread runIntake = new Thread(() -> {
            if (!stopIntake) {
                bot.intake.runIntake();
            }
        });

        waitForStart();
        while (!isStopRequested()) {
            gp1.readButtons();

            runIntake.start();

            if (!breakBeam.getState()) {
                bot.intake.stopIntake();
                stopIntake = true;
                telemetry.addLine("START to run intake again");
                if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                    intakeTime = 0.0;
                    stopIntake = true;
                }
            } else {
                sleep(100);
                intakeTime+=0.1;
            }

            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                bot.intake.setIntakeHeight(bot.intake.intakeLeft.getPosition() + 0.01);
            } else if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                bot.intake.setIntakeHeight(bot.intake.intakeLeft.getPosition() - 0.01);
            }

            telemetry.addData("break beam", breakBeam.getState());
            telemetry.addData("arm pos", bot.intake.intakeLeft.getPosition());
            telemetry.addData("intake time", intakeTime + " seconds");
            telemetry.update();
        }
    }
}
