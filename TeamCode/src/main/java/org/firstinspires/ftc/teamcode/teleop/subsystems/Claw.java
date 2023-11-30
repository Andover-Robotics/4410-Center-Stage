package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    private final Servo claw;

    // TODO: Tune open and close values
    public static double open = 0.65;
    public static double close = 0.78;
    public boolean isOpen = true;

    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        claw.setPosition(open);
        isOpen = true;
    }

    public void close(){
        claw.setPosition(close);
        isOpen = false;
    }
}
