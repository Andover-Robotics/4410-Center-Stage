package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    private final Servo claw;

    // TODO: Tune open and close values
    public static double open = 0.3;
    public static double close = 0;
    public boolean isOpen = true;

    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        claw.setPosition(open);
        isOpen = true;
    }

//    public void close(int storeVal){ dont need two diff close positions for the claw, but need for the v4b
    // we need to make some type of thing that switches a boolean(isTop) if it picks up the top pixel, so it knows if it needs to go to the bottom pixel next time
//        switch (storeVal) {
//            case 1:
//                claw.setPosition(close1);
//                break;
//            case 2:
//                claw.setPosition(close2);
//                break;
//        }
//        isOpen = false;
    public void close(){
        claw.setPosition(close);
        isOpen = false;
    }
}
