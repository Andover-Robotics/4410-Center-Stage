package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    public BNO055IMU imu0;
    private double imuOffset = 0;

    // Define subsystem objects
    public Intake intake;
    public final CRServo counterRoller;
    public static final double COUNTERROLLER_SPEED = 0.6; // input is in motor power control; see how it is converted in intake function
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
        counterRoller = opMode.hardwareMap.crservo.get("counterRoller");
        slides = new Slides(opMode);
        fourbar = new V4B(opMode);
        claw = new Claw(opMode);
    }

    // BOT STATES
    public void intake(boolean isReverse) {
        // setPower needs a double between 0 and 1 where 0.5 is no motion, 0 is 100% power in one direction and 1 is 100% power in the opposite
        // the expression below turns motor power control (-1 to 1) to this type of control by reducing the domain to (-0.5 to 0.5) with 0 being no motion
        // and then adding 0.5 to acheive the (0 to 1) and the 0.5 origin
        // multiplying by -1 allows it to spin in the opposite way
        if (!isReverse) {
            intake.runIntake();
            counterRoller.setPower(COUNTERROLLER_SPEED/2.0+0.5);
        } else{
            intake.runReverseIntake();
            counterRoller.setPower(COUNTERROLLER_SPEED/-2.0+0.5);
        }
    }

    public void storage() {//was initialized
        fourbar.storage();
        slides.runToBottom();
        claw.open();
        state = BotState.STORAGE;
    }

//    public void pickup(boolean isTop) {
//        claw.open();
//        if(isTop){
//            fourbar.topPixel();
//        } else{
//            fourbar.bottomPixel();
//        }
//        slides.runToBottom();
//        claw.close(); // changes val based on amount of pixels
//        fourbar.storage();
//        state = BotState.STORAGE; // bot is now ready to score I WANT TO INTEGRATE THIS WITH THE CLAW SO THE CLAW CLOSING AND PICKING UP SWITCHES IT huh when is the bot in this state
//        //also i just realized it shouldnt be a boolean because there could also be no pixels yeah it's best if it was an int so we can do 0,1,2, etc. it goes Storage(above pixel) -> pickup -> outtake
//    }

    public void pickup(int place) {
        claw.open();
        slides.runToBottom();
        switch(place) {
            case 1: fourbar.topPixel();
            case 2: fourbar.bottomPixel();
            default: fourbar.storage();
        }
        claw.close();
        fourbar.storage();
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
    public void drop(){ // drop pixel and return to storage
        claw.open();
        storage();
        state = BotState.STORAGE;
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
