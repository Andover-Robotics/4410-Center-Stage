package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    private final Servo armLeft, armRight, wrist;

    //values need to be changed
//    0.88 - top
//0.9 - bottom
//0.35 - outtake
//0.79 - ground
    public static double armOuttake = 0.35, armStorage=0.82, armGround = 0.79, armTopPixel = 0.88, armBottomPixel = 0.9;
    public static double wristOuttake = 0.5, wristStorage=0.2;


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
        armRight.setPosition(0.95 - position);
    }

    private void setWrist(double position) {
        wrist.setPosition(position);
    }

    public void outtake(){
        setArm(armOuttake);
        setWrist(wristOuttake);
    }

    public void ground() {
        setArm(armGround);
        setWrist(wristStorage);
    }

    public void storage() {
        setArm(armStorage);
        setWrist(wristStorage);
    }

    public void topPixel() {
        setArm(armTopPixel);
        setWrist(wristStorage);
    }

    public void bottomPixel() {
        setArm(armBottomPixel);
        setWrist(wristStorage);
    }

    public void runManualOuttake(double fourpos, double clawpos) {
        setArm(fourpos);
        setWrist(clawpos);
    }

}