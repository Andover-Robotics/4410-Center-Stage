package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    private final Servo armLeft, armRight, wrist;

    //values need to be changed
    public static double armOuttake = 0.6, armStorage=0.6, armGround = 0.6, armTopPixel = 0.6, armBottomPixel = 0.6;
    public static double wristOuttake = 0.6, wristStorage=0.6;


    public V4B(OpMode opMode) {
        armLeft = opMode.hardwareMap.servo.get("armLeft");
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight = opMode.hardwareMap.servo.get("armRight");
        armRight.setDirection(Servo.Direction.FORWARD);
        wrist = opMode.hardwareMap.servo.get("claw");
        wrist.setDirection(Servo.Direction.FORWARD);
    }

    private void setArm(double position) {
        armLeft.setPosition(position);
        armRight.setPosition(1 - position);
    }

    private void setWrist(double position) {
        armLeft.setPosition(position);
        armRight.setPosition(1 - position);
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