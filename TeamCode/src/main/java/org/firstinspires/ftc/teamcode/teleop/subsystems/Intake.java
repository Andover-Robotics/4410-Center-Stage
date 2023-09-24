package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Intake {
    private final MotorEx toodles;

    public Intake(OpMode opMode){
        toodles = new MotorEx(opMode.hardwareMap, "intake");
        toodles.setRunMode(Motor.RunMode.RawPower);
        toodles.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        toodles.setInverted(false);
    }

    public void runIntake(double power){
        toodles.setInverted(false);
        toodles.set(Math.abs(power));
    }

    public void runReverseIntake(double power){
        toodles.setInverted(true);
        toodles.set(Math.abs(power));
    }
}
