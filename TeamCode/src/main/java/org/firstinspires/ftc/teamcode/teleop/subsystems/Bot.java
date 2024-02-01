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
            slides.runToBottom();
            claw.fullOpen();
            try { Thread.sleep(100); } catch (InterruptedException ignored) {}
            fourbar.pickup();
            try { Thread.sleep(400); } catch (InterruptedException ignored) {}
            claw.pickupClose();
            try { Thread.sleep(300); } catch (InterruptedException ignored) {}
            storage();
        });
        thread.start();
    }

    public void fixPixels() { // align pixels in storage
        Thread thread = new Thread(() -> {
            slides.runToBottom();
            claw.fullOpen();
            intake.setIntakeHeight(0.1);
            try { Thread.sleep(100); } catch (InterruptedException ignored) {}
            fourbar.bottomPixel();
            try { Thread.sleep(300); } catch (InterruptedException ignored) {}
            claw.pickupClose();
            try { Thread.sleep(250); } catch (InterruptedException ignored) {}
            storage();
            try { Thread.sleep(50); } catch (InterruptedException ignored) {}
            claw.fullOpen();
        });
        thread.start();
    }

    public void presetSlides(int pos) { // run to preset slide position, the pos variable has a range of 1-4, representing bottom, low, middle, and top
        claw.close();
        switch (pos) {
            case 1: slides.runToBottom(); storage(); break;
            case 2: slides.runToLow(); outtakeOut(claw.getClawState()); break;
            case 3: slides.runToMiddle(); break;
            case 4: slides.runToTop(); break;
        }
        if (pos == 4) {
            Thread thread = new Thread(() -> {
                try { Thread.sleep(900); } catch (InterruptedException ignored) {}
                outtakeOut((claw.getClawState()));
            });
            thread.start();
        } else if (pos == 3) {
            Thread thread = new Thread(() -> {
                try { Thread.sleep(500); } catch (InterruptedException ignored) {}
                outtakeOut((claw.getClawState()));
            });
            thread.start();
        }
    }

    public void drop() { // drop pixel in outtake or storage position
        Thread thread = new Thread(() -> {
            claw.open();
            try { Thread.sleep(300); } catch (InterruptedException ignored) {}
            if (state == Bot.BotState.OUTTAKE_OUT) {
                if (claw.getClawState() == 0) {
                    storage();
                } else if (claw.getClawState() == 1) {
                    fourbar.setArm(0.3);
                    fourbar.setWrist(0.22);
                    if (slides.getPosition() > -1700) {
                        slides.runTo(slides.getPosition() - 500);
                    } else if (slides.getPosition() <=-1700){
                        slides.runTo(-2300);
                    }
                    try { Thread.sleep(300); } catch (InterruptedException ignored) {}
                    fourbar.topOuttake(false);
                    claw.close();
                }
            } else if (state == BotState.OUTTAKE_DOWN) {
                if (claw.getClawState() == 0) {
                    storage();
                } else if (claw.getClawState() == 1) {
                    fourbar.ground();
                }
            } else if (state == BotState.STORAGE) {
                fourbar.setArm(0.93);
                intake.setIntakeHeight(0.1);
                try { Thread.sleep(100); } catch (InterruptedException ignored) {}
                claw.open();
                try { Thread.sleep(100); } catch (InterruptedException ignored) {}
                fourbar.storage();
            } else {
                storage();
            }
            //bot.claw.close();
        });
        thread.start();
    }

    public void intake(boolean isReverse) { // intake/reverse intake
        if (intake.getIntakeHeight() != intake.intakeOut) intake.setIntakeHeight(intake.intakeOut);
        if (!isReverse) {
            intake.runIntake();
        } else {
            intake.runReverseIntake();
        }
    }

    // IK DEMOS?
    public void ikDemo1() { fourbar.setArm(0.35); slides.runTo(slides.getPosition()+600);}
    public void ikDemo2() { fourbar.setArm(0.18); slides.runTo(slides.getPosition()-600);}

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
