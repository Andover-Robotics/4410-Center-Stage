package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    private final Servo claw;

    // TODO: Tune open and close values
    public static double fullOpen = 0.65;
    public static double halfOpen = 0.70;
    public static double close = 0.77;
    public enum ClawState{
        EMPTY, // Has no pixels
        SINGLE, // Only one pixel (top one)
        BOTH // Both pixels (top and bottom)
    }
    public ClawState clawState = ClawState.EMPTY;

    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);
    }

    // Open methods
    public void open() { // open full, drop both top and bottom or top
        if (clawState == ClawState.BOTH) {
            claw.setPosition(halfOpen);

            clawState = ClawState.SINGLE;
        } else {
            claw.setPosition(fullOpen);
            clawState = ClawState.EMPTY;
        }
    }

    public void fullOpen() { // open full, drop both top and bottom or top
            claw.setPosition(fullOpen);
            clawState = ClawState.EMPTY;
    }

    // Close methods
    public void pickupClose() { // close, cover
        claw.setPosition(close);
        clawState = ClawState.BOTH;
    }

    public void close() { // close, cover
        claw.setPosition(close);
    }

    public int getClawState() {
        switch (clawState) {
            case SINGLE: return 1;
            case BOTH: return 2;
            default: return 0; // default or empty
        }
    }


}
