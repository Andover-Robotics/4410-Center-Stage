package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    public final Servo armLeft, armRight, wrist;

    public static double armBottomOuttake = 0.28, armTopOuttake = 0.26, armStorage = 0.82, armGround = 0.10, armDualPickup = 0.97, armBlock = 0.84;
    public static double wristBottomOuttake = 0.12, wristTopOuttake = 0.13,  wristStorage = 0.97, wristGround = 0.0, wristDualPickup = 0.95, wristBlock = 0.08;

    public V4B(OpMode opMode) {
        armLeft = opMode.hardwareMap.servo.get("armLeft");
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight = opMode.hardwareMap.servo.get("armRight");
        armRight.setDirection(Servo.Direction.FORWARD);
        wrist = opMode.hardwareMap.servo.get("wrist");
        wrist.setDirection(Servo.Direction.FORWARD);
    }

    public void setArm(double position) {
        armLeft.setPosition(position);
        armRight.setPosition(1 - position);
    }

    public void setWrist(double position) {
        wrist.setPosition(position);
    }

    public void armBlock(){
        setArm(armBlock);
        setWrist(wristBlock);
    }

    public void dualOuttake(int pixel) {
        if (pixel == 1){
            setArm(armTopOuttake);
            setWrist(wristTopOuttake);
        } else {
            setArm(armBottomOuttake);
            setWrist(wristBottomOuttake);
        }
    }

    public void ground() {
        setWrist(wristGround);
        setArm(armGround);
    }

    public void storage() {
        Thread thread = new Thread(() -> {
            setWrist(wristStorage);
            try { Thread.sleep(500); } catch (InterruptedException ignored) {}
            setArm(armStorage);
        });
        thread.start();
    }

    public void pickup() {
        setWrist(wristDualPickup);
        setArm(armDualPickup);
    }

    public void bottomPixel() {
        setWrist(0.96);
        setArm(0.94);
    }

    public void runArm(double manual) {
        if (manual > 0.1) {
            if (armLeft.getPosition() > 0.15 && armLeft.getPosition() < 0.37) {//limit values are different because it usually overshoots by 0.002, if it was equal it would get stuck
                setArm(armLeft.getPosition() + 0.005);
            }
        } else if (manual < -0.1){
            if (armLeft.getPosition() > 0.16 && armLeft.getPosition() < 0.38) {
                setArm(armLeft.getPosition() - 0.005);
            }
        }
    }

    public double getArmPosition() {
        return armLeft.getPosition();
    }

    public void topOuttake(boolean auto){
        setWrist(wristTopOuttake);
        if (!auto) setArm(armTopOuttake); else setArm(0.21);
    }

}