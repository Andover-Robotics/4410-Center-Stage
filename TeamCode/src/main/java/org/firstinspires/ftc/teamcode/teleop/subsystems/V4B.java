package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

// TODO: Set servo values for four bar

@Config
public class V4B {
    public final Servo armLeft, armRight, wrist;

    public static double armOuttake = 0.31, armTopOuttake = 0.33, armStorage = 0.805, armGround = 0.08, armTopPixel = 0.88, armBottomPixel = 0.91, armDualPickup = 0.94;
    public static double wristBottomOuttake = 0.49, wristTopOuttake = 0.54,  wristStorage = 0.24, wristGround = 0.36, wristTopPixel = 0.245, wristBottomPixel = 0.24, wristDualPickup = 0.23, wristTransfer = 0.22;

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
        armRight.setPosition(0.95 - position);
    }

    private void setWrist(double position) {
        wrist.setPosition(position);
    }

//    public void outtake() {
//        setWrist(wristTransfer);
//        setArm(armOuttake);
//        setWrist(wristBottomOuttake);
//    }

    public void dualOuttake(int pixel) {
        setArm(wristTransfer);
        setArm(armOuttake);
        if (pixel == 1){
            setWrist(wristTopOuttake);
        } else {
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

    public void topPixel() {
        setWrist(wristTopPixel);
        setArm(armTopPixel);
    }

    public void bottomPixel() {
        setWrist(wristBottomPixel);
        setArm(armBottomPixel);
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

    public void topOuttake(){
        setWrist(wristTopOuttake);
        setArm(armTopOuttake);
    }

    public void autoTopOuttake() {
        setWrist(wristTopOuttake);
        setArm(0.21);
    }

    public void runManualOuttake(double fourpos, double clawpos) {
        setArm(fourpos);
        setWrist(clawpos);
    }

    public void runAngle(double slidePosition) {
        double desiredAngle = 180 - 60.001 - Math.toDegrees(Math.asin((Math.sin(60.001) * 268.754 - (slidePosition / 8.558)) / 170.0));
        double newPosition = 0.00333 * desiredAngle;
        if (slidePosition > -1500) {
            setArm(newPosition);
        }
    }

}