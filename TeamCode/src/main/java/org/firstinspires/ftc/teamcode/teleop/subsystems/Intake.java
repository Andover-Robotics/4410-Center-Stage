package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    private final MotorEx noodles;
    private final CRServo counterRoller;
    public final Servo intakeLeft;
    private final Servo intakeRight;

    public static double power = 0.4, counterPower = 1.0; // optimal speed for intake
    public double intakeStorage = 0.03, intakeOut = 0.34, intakeUp = 0.2;
    private boolean isRunning = false;
    DigitalChannel breakBeam;

    public Intake(OpMode opMode){
        noodles = new MotorEx(opMode.hardwareMap, "intake");
        noodles.setRunMode(Motor.RunMode.RawPower);
        noodles.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        noodles.setInverted(false);

        counterRoller = new CRServo(opMode.hardwareMap, "counterRoller");

        intakeLeft = opMode.hardwareMap.servo.get("intakeLeft");
        intakeLeft.setDirection(Servo.Direction.FORWARD);
        intakeRight = opMode.hardwareMap.servo.get("intakeRight");
        intakeRight.setDirection(Servo.Direction.FORWARD);
    }

    public void runIntake(){
        noodles.setInverted(false);
        noodles.set(Math.abs(power));
        counterRoller.set(-counterPower);
        isRunning = true;
    }

    public void runReverseIntake(){
        noodles.setInverted(true);
        noodles.set(Math.abs(power));
        counterRoller.set(1*counterPower);
        isRunning = true;
    }

    public void stopIntake() {
        noodles.set(0);
        counterRoller.set(0);
        isRunning = false;
    }

    public void changePower(boolean increment) {
        if (increment) power+=0.01;
        else power-=0.01;
    }

    public void setIntakeHeight(double position) {
        intakeLeft.setPosition(position);
        intakeRight.setPosition(1.02-position);
    }

    public double getIntakeHeight() {
        return intakeLeft.getPosition();
    }

    public boolean getIsRunning() {
        return isRunning;
    }
}
