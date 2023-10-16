package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    private final Servo fourbar;
    private final Servo clawRotation;

    //values need to be changed
    public static double fourOuttake = 0.6, fourStorage=0.6, fourGround = 0.6, fourReady = 0.6;
    public static double clawOuttake = 0.6, clawStorage=0.6, clawGround = 0.6, clawReady = 0.6;


    public V4B(OpMode opMode) {
        fourbar = opMode.hardwareMap.servo.get("fourbar");
        fourbar.setDirection(Servo.Direction.FORWARD);
        clawRotation = opMode.hardwareMap.servo.get("clawRotation");
        clawRotation.setDirection(Servo.Direction.FORWARD);
    }

    public void outtake(){
        fourbar.setPosition(fourOuttake);
        clawRotation.setPosition(clawOuttake);
    }

    public void ground() {
        fourbar.setPosition(fourStorage);
        clawRotation.setPosition(clawStorage);
    }

    public void storage() {
        fourbar.setDirection(Servo.Direction.REVERSE);
        clawRotation.setDirection(Servo.Direction.REVERSE);
        fourbar.setPosition(fourGround);
        clawRotation.setPosition(clawGround);
    }

    public void ready() {
        fourbar.setPosition(fourReady);
        clawRotation.setPosition(clawReady);
    }

    public void runManualOuttake(double fourpos, double clawpos) {
        fourbar.setPosition(fourpos);
        clawRotation.setPosition(clawpos);
    }

    public void runManualStorage(double fourpos, double clawpos) {
        fourbar.setDirection(Servo.Direction.REVERSE);
        clawRotation.setDirection(Servo.Direction.REVERSE);
        fourbar.setPosition(fourpos);
        clawRotation.setPosition(clawpos);
    }

}