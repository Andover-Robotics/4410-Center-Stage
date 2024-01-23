package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    public final Servo armLeft, armRight, wrist;
    public static double armOuttake = 0.26, armTopOuttake = 0.23, armStorage = 0.74, armGround = 0.0, armTopPixel = 0.82, armBottomPixel = 0.9, armDualPickup = 0.93, armBlock = 0.84;
    public static double wristBottomOuttake = 0.44, wristTopOuttake = 0.55,  wristStorage = 0.24, wristGround = 0.30, wristTopPixel = 0.245, wristBottomPixel = 0.24, wristDualPickup = 0.22, wristTransfer = 0.22, wristBlock = 0.08;

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
        setArm(wristTransfer);
        if (pixel == 1){
            setArm(armTopOuttake);
            setWrist(wristTopOuttake);
        } else {
            setArm(armOuttake);
            setWrist(wristBottomOuttake);
        }
    }

    public void ground() {
        setWrist(wristGround);
        setArm(armGround);
    }

    public void storage() {
        setWrist(wristStorage);
        setArm(armStorage);
    }

    public void tapPixel(int i) {
        switch (i) {
            case 1: setWrist(wristTopPixel); setArm(armTopPixel); break;
            case 2: setWrist(wristBottomPixel); setArm(armBottomPixel); break;
        }
    }

    public void pickup() {
        setWrist(wristDualPickup);
        setArm(armDualPickup);
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

    // TODO: GET IK WORKING :D
    public void runAngle(double slidePosition) {
        double desiredAngle = 180 - 60.001 - Math.toDegrees(Math.asin((Math.sin(60.001) * 268.754 - (slidePosition / 8.558)) / 170.0));
        double newPosition = 0.00333 * desiredAngle;
        if (slidePosition > -1500) {
            setArm(newPosition);
        }
    }

}