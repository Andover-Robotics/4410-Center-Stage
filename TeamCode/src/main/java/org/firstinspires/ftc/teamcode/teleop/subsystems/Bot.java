package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Bot {

    public enum BotState {
        INITIALIZED, // Initialized position: slides and 4B are down
        SCORE // Scoring position:
    }

    public static Bot instance;
    public BotState state = BotState.INITIALIZED; // Default bot state
    private final MotorEx fl, fr, bl, br;
    public OpMode opMode;
    public BNO055IMU imu0;
    private double imuOffset = 0;

    // Define subsystem objects
    public Intake intake;
    public Slides slides;
    public V4B fourbar;
    public Claw claw;

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

        fl = new MotorEx(opMode.hardwareMap, "motorFL");
        fr = new MotorEx(opMode.hardwareMap, "motorFR");
        bl = new MotorEx(opMode.hardwareMap, "motorBL");
        br = new MotorEx(opMode.hardwareMap, "motorBR");

        intake = new Intake(opMode);
        slides = new Slides(opMode);
        fourbar = new V4B(opMode);
        claw = new Claw(opMode);
    }

    // BOT STATES
    public void initialized() {
        fourbar.ready();
        slides.runToBottom();
        claw.open();
        state = BotState.INITIALIZED;
    }

    public void pickup(int storeVal) {
        claw.open();
        fourbar.storage();
        slides.runToBottom();
        claw.close(storeVal); // changes val based on amount of pixels
        fourbar.ready();
        state = BotState.SCORE; // bot is now ready to score
    }

    public void outtake() {
        claw.open();
        initialized();
    }

    public void intake(boolean isReverse) {
        if (!isReverse) {
            intake.runIntake();
        } else intake.runReverseIntake();
    }

    public void discardPixel() {
        fourbar.ground();
        slides.runTo(0); // TODO: Find slides discard position
        claw.open();
        initialized(); // return to init position
    }

    // TODO: Figure out how much to turn and drive forward
    public void alignJunction() {
        double turn = 1.7;
        if (ColorDetection.spikeMark == ColorDetection.SpikeMark.LEFT) {
            drive(0,0, -1* turn * Math.abs((ColorDetection.camwidth/2.0)-ColorDetection.midpointrect));
        }
        if (ColorDetection.spikeMark == ColorDetection.SpikeMark.RIGHT) {
            drive(0,0, turn * Math.abs((ColorDetection.camwidth/2.0)-ColorDetection.midpointrect));
        }
        if (ColorDetection.spikeMark == ColorDetection.SpikeMark.MIDDLE) {
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
        imu0 = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu0.initialize(parameters);
        resetIMU();
    }

    public void resetIMU() {
        imuOffset += getIMU();
    }

    public double getIMU() {
        double angle = (imu0.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle - imuOffset) % 360;
        if (angle > 180) {
            angle = angle - 360;
        }
        return angle;
    }

}
