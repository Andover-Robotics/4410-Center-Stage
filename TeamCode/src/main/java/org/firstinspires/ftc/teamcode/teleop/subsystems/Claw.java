package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    private final Servo claw;

    public static double fullOpen = 0.54, close = 0.36, halfOpen = 0.42;


    public enum ClawState{
        EMPTY, // Has no pixels
        SINGLE, // Only one pixel (top one)
        BOTH // Both pixels (top and bottom)
    }
    public ClawState clawState = ClawState.EMPTY;
    public int getClawState() {
        switch (clawState) {
            case SINGLE: return 1;
            case BOTH: return 2;
            default: return 0; // default or empty
        }
    }

    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) { // Only used in autonomous to set claw servo to specific pos !DOES NOT TYPICALLY CHANGE CLAW STATE!
        claw.setPosition(position);
        if (position == 0.66) { // position is extra open, set claw state to empty
            clawState = ClawState.EMPTY;
        }
    }

    // Open methods
    public void open() { // open full, drop both top and bottom or top
        if (clawState == ClawState.BOTH) {
            halfOpen();
        } else {
            fullOpen();
        }
    }
    public void fullOpen() { // open full, drop both top and bottom or top
        claw.setPosition(fullOpen);
        clawState = ClawState.EMPTY;
    }
    public void halfOpen() {
        claw.setPosition(halfOpen);
        clawState = ClawState.SINGLE;
    }

    // Close methods
    public void pickupClose() { // close, cover
        claw.setPosition(close);
        clawState = ClawState.BOTH;
    }
    public void close() { // close, cover
        claw.setPosition(close);
    }
}
