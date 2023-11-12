package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Bot {

    public enum BotState {
        STORAGE, // Initialized position: slides and 4B are down
        OUTTAKE_OUT, // Outtaking with arm out at angle
        OUTTAKE_DOWN, //Outtaking on ground
    }

    public static Bot instance;
    public BotState state = BotState.STORAGE; // Default bot state
    private final MotorEx fl, fr, bl, br;
    public OpMode opMode;
    public BHI260IMU imu0;
    private double imuOffset = 0;

    // Define subsystem objects
    public Intake intake;
    public Slides slides;
    public V4B fourbar;
    public Claw claw;
    Thread thread;

    // get bot instance
    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized!");
        }
        return instance;
    }
    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    private Bot(OpMode opMode) {
        this.opMode = opMode;

        fl = new MotorEx(opMode.hardwareMap, "motorFL", Motor.GoBILDA.RPM_435);
        fr = new MotorEx(opMode.hardwareMap, "motorFR", Motor.GoBILDA.RPM_435);
        bl = new MotorEx(opMode.hardwareMap, "motorBL", Motor.GoBILDA.RPM_435);
        br = new MotorEx(opMode.hardwareMap, "motorBR", Motor.GoBILDA.RPM_435);

        intake = new Intake(opMode);
        slides = new Slides(opMode);
        fourbar = new V4B(opMode);
        claw = new Claw(opMode);
    }

    // BOT STATES
    public void storage() {//was initialized
        fourbar.storage();
        slides.runToBottom();
        claw.close();
        state = BotState.STORAGE;
    }

    public void outtakeOut() { // go to outtake backboard position
        fourbar.outtake();
        state = BotState.OUTTAKE_OUT;
    }

    public void outtakeDown() { // go to outtake down/ground position
        fourbar.ground();
        slides.runToBottom();
        state = BotState.OUTTAKE_DOWN;
    }

    public void intake(boolean isReverse) {
        if (!isReverse) {
            intake.runIntake();
        } else intake.runReverseIntake();
    }

    // TODO: FIX THIS DISCARD
    public void discard() { // discard pixel
        fourbar.discard();
        claw.open();
        storage();
    }


    // TODO: Figure out how much to turn and drive forward
    public void alignSpike() {
        double turn = 1.7;
        if (ColorDetectionPipeline.spikeMark == ColorDetectionPipeline.SpikeMark.LEFT) {
            drive(0,0, -1* turn * Math.abs((ColorDetectionPipeline.camwidth/2.0)-ColorDetectionPipeline.midpointrect));
        }
        if (ColorDetectionPipeline.spikeMark == ColorDetectionPipeline.SpikeMark.RIGHT) {
            drive(0,0, turn * Math.abs((ColorDetectionPipeline.camwidth/2.0)-ColorDetectionPipeline.midpointrect));
        }
        if (ColorDetectionPipeline.spikeMark == ColorDetectionPipeline.SpikeMark.MIDDLE) {
            drive(0,0,0);
        }
    }

    public void fixMotors() {
        fl.setInverted(false);
        fr.setInverted(true);
        bl.setInverted(false);
        br.setInverted(true);

        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);

        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void stopMotors() {
        fl.set(0.0);
        fr.set(0.0);
        bl.set(0.0);
        br.set(0.0);
    }

    public void drive(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double[] speeds = {
                forwardBackSpeed - strafeSpeed - turnSpeed,
                forwardBackSpeed + strafeSpeed + turnSpeed,
                forwardBackSpeed + strafeSpeed - turnSpeed,
                forwardBackSpeed - strafeSpeed + turnSpeed
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(speeds[2]);
        br.set(speeds[3]);
    }

    // IMU
    public void setImuOffset(double offset) {
        imuOffset += offset;
    }

    public void initializeImus() {
        imu0 = opMode.hardwareMap.get(BHI260IMU.class, "imu");
        final BHI260IMU.Parameters parameters = new BHI260IMU.Parameters((ImuOrientationOnRobot) imu0.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));

        imu0.initialize(parameters);
        resetIMU();
    }

    public void resetIMU() {
        imuOffset += getIMU();
    }

    public double getIMU() {
        double angle = (imu0.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - imuOffset) % 360;
        if (angle > 180) {
            angle = angle - 360;
        }
        return angle;
    }

}
