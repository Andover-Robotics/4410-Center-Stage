package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Intake {
    private final MotorEx noodles;
    private final CRServo counterRoller;
    public double power = 0.25; // optimal speed for intake
    private double counterPower = 1.0;
    private boolean isRunning = false;

    public Intake(OpMode opMode){
        noodles = new MotorEx(opMode.hardwareMap, "intake");
        counterRoller = new CRServo(opMode.hardwareMap, "counterRoller");
        noodles.setRunMode(Motor.RunMode.RawPower);
        noodles.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        noodles.setInverted(false);
    }

    public void runIntake(){
        noodles.setInverted(false);
        noodles.set(Math.abs(power));
        counterRoller.set(counterPower);
        isRunning = true;
    }

    public void runReverseIntake(){
        noodles.setInverted(true);
        noodles.set(Math.abs(-0.28));
        counterRoller.set(-1*counterPower);
        isRunning = true;
    }

    public void stopIntake() {
        noodles.set(0);
        counterRoller.set(0);
        isRunning = false;
    }

    public boolean getIsRunning() {
        return isRunning;
    }
}
