package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    private final Servo claw;
    public static double open = 0.3, close = 0.12; //TODO: NEED TO TUNE VALUES
    public boolean isOpen = false;

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

    public void toggle() {
        if (isOpen) {
            claw.setPosition(close);
        } else {
            claw.setPosition(open);
        }
    }
}
