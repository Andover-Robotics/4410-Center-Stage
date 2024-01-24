package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    public final Servo armLeft, armRight, wrist;

    public static double armBottomOuttake = 0.26, armTopOuttake = 0.23, armStorage = 0.82, armGround = 0.10, armDualPickup = 0.95, armBlock = 0.84;
    public static double wristBottomOuttake = 0.12, wristTopOuttake = 0.12,  wristStorage = 0.98, wristGround = 0.0, wristDualPickup = 0.94, wristBlock = 0.08;

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
        setWrist(wristStorage);
        setArm(armStorage);
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

    // TODO: GET IK WORKING :D greyson gay
    // i am gay - greyson 2024 1/23/24
    //cap - Vignesha 1/23/24
    // our robot is gay - anushka 1/23/24

    public void runAngle(double slidePosition) {
        double desiredAngle = 180 - 60.001 - Math.toDegrees(Math.asin((Math.sin(60.001) * 268.754 - (slidePosition / 8.558)) / 170.0));
        double newPosition = 0.00333 * desiredAngle;
        if (slidePosition > -1500) {
            setArm(newPosition);
        }
    }

}