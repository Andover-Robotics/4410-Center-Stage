package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    private final Servo claw;

    // TODO: Tune open and close values
    public static double open = 0.3;
    public static double close1 = 0.12, close2 = 0.12;
    public boolean isOpen = false;

    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        claw.setPosition(open);
        isOpen = true;
    }

    public void close(int storeVal){
        switch (storeVal) {
            case 1:
                claw.setPosition(close1);
                break;
            case 2:
                claw.setPosition(close2);
                break;
        }
        isOpen = false;
    }
}
