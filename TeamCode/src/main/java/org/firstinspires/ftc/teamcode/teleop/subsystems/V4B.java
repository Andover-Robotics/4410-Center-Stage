package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    private final Servo armLeft, armRight, wrist;
//0.88 - top
//0.9 - bottom
//0.35 - outtake
//0.79 - ground
    //values need to be changed
    public static double armOuttake = 0.35, armStorage=0.76, armGround = 0.08, armTopPixel = 0.92, armBottomPixel = 0.96;
    public static double wristOuttake = 0.62, wristStorage=0.28, wristGround = 0.34, wristTopPixel = 0.28, wristBottomPixel = 0.32, wristTransfer = 0.18;


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