package org.firstinspires.ftc.teamcode.teleop.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Bot {

    //TODO: write sus-system FUNCTIONALITY (function names have been called in bot functions to aid in what is needed)
    //TODO: revise order of instructions that is written for every bot-state

    public enum BotState {
        INTAKE, // Intaking pixels
        STORAGE_1, // Storage contains 1 pixel
        STORAGE_2, // Storage contains 2 pixels, full
        OUTTAKE, // Claw is ready to pick up pixel
        SCORING, // Claw has pixel and is ready to place on backboard
        CLIMBING // Robot is climbing/hanging on bar
    }

    public static Bot instance;

    public BotState botState = BotState.INTAKE;
    private final MotorEx fl, fr, bl, br;

    private final double kTurn = 1.7;

    //storage counters to find Status
    private int pixelsInStorage = 0;

    // initalize objects of sus-systems (once they have functionality)

//    public Climber climber;
    public Intake intake;
//    public Launcher launcher;
//    public Slides slides;
//    public V4B fourBar;
    public Claw claw;



    public OpMode opMode;

    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
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

//        climber = new Climber(opMode);
        intake = new Intake(opMode);
//        launcher = new Launcher(opMode);
//        slides = new Slides(opMode);
//        fourBar = new V4B(opMode);
        claw = new Claw(opMode);
    }


    public boolean checkIfFull(){return (pixelsInStorage==2);}
    public int getPixelsInStorage(){return pixelsInStorage;}

    public void intake() {
        //check if full
        // run tubing until it hits storage
        if (checkIfFull()) {
            return;
        }
        botState = BotState.INTAKE;
        intake.runIntake(0.5);
    }

    public void intoStorage(){
        pixelsInStorage++;
        switch (pixelsInStorage) {
            case 1:
                botState = BotState.STORAGE_1;
            case 2:
                botState = BotState.STORAGE_2;
        }
    }

    public void scorePosition(int numPixels) {
        botState = BotState.SCORING;
        claw.open();
        switch (numPixels) {
            case 1:
//                slides.run(certain length for one pixels);
//                V4B.rotateTo(certain degrees);
                claw.close();
                pixelsInStorage--;
            case 2:
//                slides.run(certain length for two pixels);
//                V4B.rotateTo(certain degrees);
                claw.close();
                pixelsInStorage-=2;
        }
//        V4B.runToFront();
    }

    public void outtake(int option) {
        // option could be the idfferent places we could drop our pixels
        // ex. 1 is on the ground, 2 is on the backdrop, 3 is discard
        // TODO: brainstorm options
        botState = BotState.OUTTAKE;

        //make sure bar and claw is facing outward
//        V4B.runToFront();

        switch (option) {
            case 1:
//                slides.runToBottom();
                claw.open();
            case 2:
//                slides.run(backdrop height);
                claw.open();
            case 3:
                claw.open();
        }
    }

    public void climbOn() {
        botState = BotState.CLIMBING;
//        V4B.runToFront();
//        slides.runToBottom();
        claw.close();
//        climber.runSlidesUp();
//        climber.hookClose();
//        climber.runSlidesDown();
    }

    public void climbOff() {
        botState = BotState.INTAKE;
//        climber.runSlidesUp();
//        climber.hookOpen();
//        climber.runSlidesDown();
    }

//    public void droneLaunch() {
////        dunno what the frinkiewrikle how to code for drone launching
//    }

    public void alignJunction() {
        if (ColorDetection.spikeMark == ColorDetection.SpikeMark.LEFT) {
            drive(0,0, -1* kTurn * Math.abs((ColorDetection.camwidth/2.0)-ColorDetection.midpointrect));
        }
        if (ColorDetection.spikeMark == ColorDetection.SpikeMark.RIGHT) {
            drive(0,0, kTurn * Math.abs((ColorDetection.camwidth/2.0)-ColorDetection.midpointrect));
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

}
