package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Launcher {
    private final Servo launcher;
    public static double initialized = 0.18;
    public static double launched = 0.28;

    public Launcher(OpMode opMode){
        launcher = opMode.hardwareMap.servo.get("launcher");
        launcher.setDirection(Servo.Direction.FORWARD);
        reset();
    }

    public void reset() {
        launcher.setPosition(initialized);
    }

    public void launch(){
        launcher.setPosition(launched);
    }
}
