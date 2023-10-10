package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Intake {
    private final MotorEx toodles;
    private double power = 0.7;

    public Intake(OpMode opMode){
        toodles = new MotorEx(opMode.hardwareMap, "intake");
        toodles.setRunMode(Motor.RunMode.RawPower);
        toodles.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        toodles.setInverted(false);
    }

    public void runIntake(){
        toodles.setInverted(false);
        toodles.set(Math.abs(power));
    }

    public void runReverseIntake(){
        toodles.setInverted(true);
        toodles.set(Math.abs(power));
    }

    public void stopIntake() {
        toodles.set(0);
    }
}
