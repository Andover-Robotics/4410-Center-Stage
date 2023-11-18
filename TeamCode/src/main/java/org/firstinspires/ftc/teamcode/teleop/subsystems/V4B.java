package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    private final Servo armLeft, armRight, wrist;

    public static double armOuttake = 0.25, armStorage = 0.76, armGround = 0.08, armTopPixel = 0.89, armBottomPixel = 0.92, armDiscard = 0.8;
    public static double wristOuttake = 0.6, wristStorage=0.375, wristGround = 0.42, wristTopPixel = 0.35, wristBottomPixel = 0.36, wristTransfer = 0.26, wristDiscard = 0.28;

    public V4B(OpMode opMode) {
        armLeft = opMode.hardwareMap.servo.get("armLeft");
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight = opMode.hardwareMap.servo.get("armRight");
        armRight.setDirection(Servo.Direction.FORWARD);
        wrist = opMode.hardwareMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
    }

    private void setArm(double position) {
        armLeft.setPosition(position);
        armRight.setPosition(0.96 - position);
    }

    private void setWrist(double position) {
        wrist.setPosition(position);
    }

    public void outtake() {
        setWrist(wristTransfer);
        setArm(armOuttake);
        setWrist(wristOuttake);
    }

    public void discard() {
        setWrist(wristDiscard);
        setArm(armDiscard);
    }

    public void ground() {
        setWrist(wristGround);
        setArm(armGround);
    }

    public void storage() {
        setWrist(wristStorage);
        setArm(armStorage);
    }

    public void topPixel() {
        setWrist(wristTopPixel);
        setArm(armTopPixel);
    }

    public void bottomPixel() {
        setWrist(wristBottomPixel);
        setArm(armBottomPixel);
    }

    public void runManualOuttake(double fourpos, double clawpos) {
        setArm(fourpos);
        setWrist(clawpos);
    }

}