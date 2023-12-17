package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.*;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueClose = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(90))) // Starting position
                                .back(36)
                                .forward(12) // Distance away from spike mark when scoring
                                .waitSeconds(0.5) // Wait to score
                                .splineToLinearHeading(new Pose2d(42, 35, Math.toRadians(180)), Math.toRadians(180)) // Spline to backboard
                                .back(10) // Back into backboard
                                .waitSeconds(0.5) // Wait to score
                                .forward(10) // Back up

                                // GO THROUGH SIDE TRUSS
//                                .forward(100) // Drive across field
//                                .waitSeconds(5) // Wait for intake from stack
//                                .back(100)
//                                .splineToConstantHeading(new Vector2d(50,42), Math.toRadians(180))
//                                .waitSeconds(0.5) // Wait to score
//                                .forward(5) // Back up
//                                .strafeRight(16) // Park on side


                                // GO THROUGH MIDDLE TRUSS
                                .strafeLeft(23) // Start pixel stack trajectory
                                .forward(100) // Drive across field
                                .waitSeconds(3) // Wait for intake from stack
                                .back(100)
                                .strafeRight(23) // Strafe to center of backboard
                                .back(10) // Back into backboard
                                .waitSeconds(0.5) // Wait to score
                                .forward(5) // Back up
                                .strafeLeft(23) // Park on side

                                .waitSeconds(2)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .addEntity(blueClose)
                .start();
    }
}