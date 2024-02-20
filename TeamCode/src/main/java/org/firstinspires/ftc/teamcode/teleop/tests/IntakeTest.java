package org.firstinspires.ftc.teamcode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

// This class was made to demonstrate how the break beam prevents intake from running when there is a pixel

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    private Bot bot;
    DigitalChannel breakBeam;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        breakBeam = hardwareMap.get(DigitalChannel.class, "BreakBeam");

        waitForStart();

        while (!isStopRequested()) {
            if (!breakBeam.getState()) {
                bot.intake.stopIntake();
            } else {
                bot.intake(true);
            }

            telemetry.addData("Break Beam", breakBeam.getState());
            telemetry.update();
        }
    }
}
