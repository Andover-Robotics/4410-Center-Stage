package org.firstinspires.ftc.teamcode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.TestAutonomous;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.lang.Math;

@TeleOp(name = "IK Test", group = "Test")
public class inverseKinematicsTest extends LinearOpMode {

    private Bot bot;
    private GamepadEx gp1;
    DistanceSensor distanceSensor;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");

        bot.fourbar.setWrist(bot.fourbar.wristBottomOuttake);

        waitForStart();
        while (!isStopRequested()) {

            gp1.readButtons();

            if (distanceSensor.getDistance(DistanceUnit.INCH) < 3.5) {
                bot.fourbar.setArm(0.01*(90-Math.toDegrees(Math.acos((distanceSensor.getDistance(DistanceUnit.INCH)+4.2)/7.87)))/3.55+0.54); //0.54
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                bot.claw.close();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.claw.fullOpen();
            }

            telemetry.addData("Distance = ", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Arm Position", bot.fourbar.getArmPosition());
            telemetry.update();


        }
    }
}
