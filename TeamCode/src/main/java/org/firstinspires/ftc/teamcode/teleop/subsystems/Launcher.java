package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Launcher {
    private final Servo launcher;

    public static double launchPos = 0.0;

    public Launcher(OpMode opMode) {
        launcher = opMode.hardwareMap.servo.get("launcher");
        launcher.setDirection(Servo.Direction.FORWARD);
    }

    public void activate() {
        launcher.setPosition(launchPos);
    }
}
