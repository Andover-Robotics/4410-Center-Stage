package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfiler;

// TODO: Set values for slide positions

@Config
public class Slides {

    public final MotorEx motorLeft;
    public final MotorEx motorRight;
    private PIDFController controller;

    public enum Position {
        // HIGH, MID, LOW, BOTTOM - Position on lines on backboard
        HIGH,
        MID,
        LOW,
        BOTTOM, //storage and outtake round position
    }

    public Position position = Position.BOTTOM;
    public static double p = 0.015, i = 0, d = 0, f = 0, staticF = 0.25;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1, powerMin = 0.1;
    private double manualPower = 0;
    public static int MAXHEIGHT = -1550, high = -1550, maxteleop = -1200, mid = -800, low = -400, bottom = -0, inc = 100, dec = 300;
    private final OpMode opMode;
    private double target = 0;
    private boolean goingDown = false;
    private double profile_init_time = 0;
    private MotionProfiler profiler = new MotionProfiler(30000, 20000);

    public Slides(OpMode opMode) {
        motorLeft = new MotorEx(opMode.hardwareMap, "slidesLeft", Motor.GoBILDA.RPM_435);
        motorRight = new MotorEx(opMode.hardwareMap, "slidesRight", Motor.GoBILDA.RPM_435);
        motorRight.setInverted(true);
        motorLeft.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRight.setRunMode(Motor.RunMode.RawPower);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.opMode = opMode;
    }

    public void runTo(double t) {
        motorLeft.setRunMode(Motor.RunMode.RawPower);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRight.setRunMode(Motor.RunMode.RawPower);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.init_new_profile(motorLeft.getCurrentPosition(), t);
        profile_init_time = opMode.time;

        goingDown = t > target;
        target = t;
    }

    public void runToTop() {
        runTo(high);
        position = Position.HIGH;
    }

    public void runToMiddle() {
        runTo(mid);
        position = Position.MID;
    }

    public void runToLow() {
        runTo(bottom);
        position = Position.LOW;
    }

    public void runToBottom() {
        runTo(bottom);
        position = Position.BOTTOM;
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin || motorLeft.getCurrentPosition() > -300 || motorLeft.getCurrentPosition() < MAXHEIGHT) {
            manualPower = manual;
            // Changing Position enum manually!
            if (motorLeft.getCurrentPosition() == bottom) {
                position = Position.BOTTOM;
            } else if ((motorLeft.getCurrentPosition() > bottom) && (motorLeft.getCurrentPosition() <= low)) {
                position = Position.LOW;
            } else if ((motorLeft.getCurrentPosition() > low) && (motorLeft.getCurrentPosition() <= mid)) {
                position = Position.MID;
            } else if ((motorLeft.getCurrentPosition() > mid) && (motorLeft.getCurrentPosition() <= high)) {
                position = Position.HIGH;
            }
        } else {
            manualPower = 0;
        }
    }

    public void periodic() {
        motorRight.setInverted(false);
        motorLeft.setInverted(true);
        controller.setPIDF(p, i, d, f);
        double dt = opMode.time - profile_init_time;
        if (!profiler.isOver()) {
            controller.setSetPoint(profiler.motion_profile_pos(dt));
            double power = powerUp * controller.calculate(motorLeft.getCurrentPosition());
            if (goingDown) {
                power = powerDown * controller.calculate(motorLeft.getCurrentPosition());
            }
            motorLeft.set(power);
            motorRight.set(power);
        } else {
            if (profiler.isDone()) {
                profiler = new MotionProfiler(30000, 20000);
            }
            if (manualPower != 0) {
                controller.setSetPoint(motorLeft.getCurrentPosition());
                motorLeft.set(manualPower / manualDivide);
                motorRight.set(manualPower / manualDivide);
            } else {
                double power = staticF * controller.calculate(motorLeft.getCurrentPosition());
                motorLeft.set(power);
                motorRight.set(power);
            }
        }
    }

    public double isHigh() {
        return (double) motorLeft.getCurrentPosition() / MAXHEIGHT;
    }

    public double getCurrent() {
        return motorLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS) + motorRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void resetEncoder() {
        motorLeft.resetEncoder();
    }

    public int getPosition() {
        return motorLeft.getCurrentPosition();
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(30000, 20000);
    }

    public Position getState() {
        return position;
    }
}
