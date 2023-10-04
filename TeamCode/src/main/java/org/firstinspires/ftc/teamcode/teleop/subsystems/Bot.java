package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Bot {
    //TODO: revise order of instructions that is written for every bot-state and finite state farm
    // TODO: write climbing and drone sus-system

    public enum BotState {
        INTAKE_STORAGE, // surgical tubing picking up pixels
        READY_POS, // sldies closed V4B closed, claw closed
        OUTTAKE_PICKUP, // CLAW is turned inward and is ready to pick up pixel
        OUTTAKE, // CLAW has pixel and is ready to place
        HANGING // ROBOT is attempting to or has climb and hang on the BAR
    }

    public static Bot instance;

    public BotState botState = BotState.INTAKE_STORAGE;
    private final MotorEx fl, fr, bl, br;

    //storage counters to find Status
    private int pixelsInStorage = 0;

    // initalize objects of sus-systems (once they have functionality)

    public Climber climber;
    public Intake toodles;
//    public Launcher launcher;
    public Slides slides;
    public V4B phobar;
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
        toodles = new Intake(opMode);
//        launcher = new Launcher(opMode);
        slides = new Slides(opMode);
        phobar = new V4B(opMode);
        claw = new Claw(opMode);
    }


    public boolean checkIfFull(){return (pixelsInStorage==2);}
    public int getPixelsInStorage(){return pixelsInStorage;}

//    public void intake() {
//        //check if full
//        // run tubing until it hits storage
//        if (checkIfFull()) {
//            return;
//        }
//        botState = BotState.INTAKE_STORAGE;
////        toodles.runIntake(0.5);
//    }

    public void intoStorage(){
        pixelsInStorage++;
    }

    public void ready_pos() {
        botState = BotState.READY_POS;
        phobar.ready_pos();
        claw.open();
        slides.runToBottom();
    }

    public void outtake_pickup(int numPixels) {
        botState = BotState.OUTTAKE_PICKUP;
        claw.open();
        switch (numPixels) {
            case 1:
                phobar.storage();
                slides.runTo((int) slides.ONE_PIXEL_LENGTH);
                claw.close();
                pixelsInStorage--;
            case 2:
                phobar.storage();
                slides.runTo((int) slides.TWO_PIXEL_LENGTH);
                claw.close();
                pixelsInStorage-=2;
        }
//        V4B.runToFront();
    }

    public void outtake_backboard(int pos) { // low, med, high -> 1,2,3
        botState = BotState.OUTTAKE;

        //make sure bar and claw is facing outward
        phobar.outtake();
        switch (pos) {
            case 1:
                slides.runToTop();
                break;
            case 2:
                slides.runToMiddle();
                break;
            case 3:
                slides.runToLow();
                break;
        }

    }

    public void outtake_ground() {
        botState = BotState.OUTTAKE;

        //make sure bar and claw is facing outward
        phobar.outtake();
        slides.runToBottom();

    }

    public void climbOn() {
        botState = BotState.HANGING;
        ready_pos();
//        climber.runSlidesUp();
//        climber.hookClose();
//        climber.runSlidesDown();
    }

    public void climbOff() {
        botState = BotState.INTAKE_STORAGE;
//        climber.runSlidesUp();
//        climber.hookOpen();
//        climber.runSlidesDown();
    }

//    public void droneLaunch() {
////        dunno what the frinkiewrikle how to code for drone launching
//    }

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
