package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline;

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
    public double heading = 0.0;
    private MecanumDrive drive;

    // Define subsystem objects
    public Intake intake;
    public Slides slides;
    public V4B fourbar;
    public Claw claw;
    public Launcher launcher;

    public double wristUpPos = 0.0;


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

        drive = new MecanumDrive(fl, fr, bl, br);

        intake = new Intake(opMode);
        slides = new Slides(opMode);
        fourbar = new V4B(opMode);
        claw = new Claw(opMode);
        launcher = new Launcher(opMode);
    }

    // BOT STATES
    public void storage() { // go to storage position
        fourbar.storage();
        slides.runToBottom();
        intake.setIntakeHeight(intake.intakeStorage);
        state = BotState.STORAGE;
    }

    public void outtakeOut(int pixel) { // go to outtake backboard position
        fourbar.dualOuttake(pixel);
        state = BotState.OUTTAKE_OUT;
    }

    public void outtakeGround() { // go to outtake ground position
        fourbar.ground();
        slides.runToBottom();
        state = BotState.OUTTAKE_DOWN;
    }

    public void pickup() { // pick up pixel from storage
        Thread thread = new Thread(() -> {
            try {
                slides.runToBottom();
                claw.fullOpen();
                Thread.sleep(100);
                fourbar.pickup();
                Thread.sleep(400);
                claw.pickupClose();
                Thread.sleep(300);
                storage();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void pickup(boolean single) { // pick up single pixel
        Thread thread = new Thread(() -> {
            try {
                slides.runToBottom();
                claw.fullOpen();
                Thread.sleep(100);
                fourbar.pickup();
                Thread.sleep(400);
                claw.pickupClose();
                Thread.sleep(300);
                storage();
                claw.clawState = Claw.ClawState.SINGLE;
            } catch (InterruptedException ignored) {}
        });
        if (single) {
            thread.start();
        }
    }

    public void fixPixels() { // align pixels in storage
        Thread thread = new Thread(() -> {
            try {
                slides.runToBottom();
                claw.fullOpen();
                intake.setIntakeHeight(0.1);
                Thread.sleep(100);
                fourbar.bottomPixel();
                Thread.sleep(300);
                claw.pickupClose();
                Thread.sleep(250);
                storage();
                Thread.sleep(50);
                claw.fullOpen();
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void presetSlides(int pos) { // run to preset slide position, the pos variable has a range of 1-4, representing bottom, low, middle, and top
        claw.close();
        switch (pos) {
            case 1: slides.runToBottom(); storage();  break;
            case 2: slides.runToLow(); outtakeOut(claw.getClawState()); break;
            case 3: slides.runToMiddle(); break;
            case 4: slides.runToTop(); break;
        }
        if (pos == 4) {
            Thread thread = new Thread(() -> {
                try {
                    Thread.sleep(900);
                    outtakeOut((claw.getClawState()));
                } catch (InterruptedException ignored) {}
            });
            thread.start();
        } else if (pos == 3) {
            Thread thread = new Thread(() -> {
                try {
                    Thread.sleep(500);
                    outtakeOut((claw.getClawState()));
                } catch (InterruptedException ignored) {}
            });
            thread.start();
        }
    }


    public void drop() { // drop pixel in outtake or storage position
        Thread thread = new Thread(() -> {
            try {
                claw.open();
                Thread.sleep(300);
                if (state == Bot.BotState.OUTTAKE_OUT) { // drop pixel on backboard
                    if (claw.getClawState() == 0) {
                        storage();
                    } else if (claw.getClawState() == 1) {
                        fourbar.setArm(0.65);
                        fourbar.setWrist(0.72);
                        if (slides.getPosition() > -2400) {
                            slides.runTo(slides.getPosition() - 200);
                        } else if (slides.getPosition() <= -2400){
                            slides.runTo(-2300);
                        }
                        Thread.sleep(500);
                        fourbar.topOuttake(false);
                        claw.close();
                    }
                } else if (state == BotState.OUTTAKE_DOWN) { // drop pixel on ground
                    if (claw.getClawState() == 0) {
                        storage();
                    } else if (claw.getClawState() == 1) {
                        fourbar.ground();
                    }
                } else if (state == BotState.STORAGE) { // drop pixel in storage
                    fourbar.setArm(0.2);
                    fourbar.setWrist(0.03);
                    intake.setIntakeHeight(0.1);
                    Thread.sleep(100);
                    claw.open();
                    Thread.sleep(100);
                    fourbar.storage();
                } else {
                    storage();
                }
            } catch (InterruptedException ignored) {}
        });
        thread.start();
    }

    public void drop(boolean manual) { // Extra manual drop open for tele-op in case only 1 pixel
        if (manual) {
            fourbar.setArm(0.65);
            fourbar.setWrist(0.72);
            if (slides.getPosition() > -2400) {
                slides.runTo(slides.getPosition() - 300);
            } else if (slides.getPosition() <= -2400){
                slides.runTo(-2300);
            }
            try {
                Thread.sleep(900);
            } catch (InterruptedException ignored) {}
            storage();
        }
    }

    public void intake(boolean isReverse, double height) { // intake/reverse intake
        if (intake.getIntakeHeight() != height) intake.setIntakeHeight(height);
        if (!isReverse) {
            intake.runIntake();
        } else {
            intake.runReverseIntake();
        }
    }

    // MOTORS
    public void fixMotors() {
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        fl.setInverted(false);
        fr.setInverted(true);
        bl.setInverted(false);
        br.setInverted(true);

        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);
    }
    public void stopMotors() {
        fl.set(0.0);
        fr.set(0.0);
        bl.set(0.0);
        br.set(0.0);
    }
    public double getMotorCurrent() {
        return fl.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + fr.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + bl.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + br.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    // DRIVE METHODS
    public void driveRobotCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
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
    public void driveFieldCentric(double strafeSpeed, double forwardBackSpeed, double turnSpeed) {
        double magnitude = Math.sqrt(strafeSpeed * strafeSpeed + forwardBackSpeed * forwardBackSpeed);
        double theta = (Math.atan2(forwardBackSpeed, strafeSpeed) - heading) % (2 * Math.PI);
        double[] speeds = {
                magnitude * Math.sin(theta + Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) - turnSpeed,
                magnitude * Math.sin(theta - Math.PI / 4) + turnSpeed,
                magnitude * Math.sin(theta + Math.PI / 4) - turnSpeed
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

    public void setHeading (double heading) {
        this.heading = heading;
    }

}
