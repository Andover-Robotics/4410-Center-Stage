package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    private final Servo claw;

    // TODO: Tune open and close values
    public static double fullOpen = 0.65;
    public static double halfOpen = 0.71;
    public static double fullClose = 0.77;
//    public static double open = 0.65;//old
//    public static double close = 0.77;//old code
    public enum ClawState{
        EMPTY,
        ONE,
        BOTH
    }
    public ClawState clawState = ClawState.EMPTY;

    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void fullOpen(){
        claw.setPosition(fullOpen);
        clawState = ClawState.EMPTY;
    }

    public void halfOpen(){
        claw.setPosition(halfOpen);
        clawState = ClawState.ONE;
    }

    public void fullClose(){
        claw.setPosition(fullClose);
        clawState = ClawState.BOTH;
    }

}
