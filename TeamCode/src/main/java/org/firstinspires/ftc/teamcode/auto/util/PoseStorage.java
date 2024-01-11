package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

// This class is to store the Autonomous end pose and transfer it to TeleOp through a static variable
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
}
