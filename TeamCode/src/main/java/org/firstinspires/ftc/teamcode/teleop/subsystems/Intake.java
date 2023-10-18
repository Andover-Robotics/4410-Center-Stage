package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Intake {
    private final MotorEx noodles;
    private double power = 0.28;

    public Intake(OpMode opMode){
        noodles = new MotorEx(opMode.hardwareMap, "intake");
        noodles.setRunMode(Motor.RunMode.RawPower);
        noodles.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        noodles.setInverted(false);
    }

    public void runIntake(){
        noodles.setInverted(false);
        noodles.set(Math.abs(power));
    }

    public void runReverseIntake(){
        noodles.setInverted(true);
        noodles.set(Math.abs(power));
    }

    public void stopIntake() {
        noodles.set(0);
    }
}
