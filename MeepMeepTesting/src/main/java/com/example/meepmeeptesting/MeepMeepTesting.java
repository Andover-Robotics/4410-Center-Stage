package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.*;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        // CLOSE
        Pose2d blueCloseStart = new Pose2d(12,60,Math.toRadians(90));
        RoadRunnerBotEntity blueCenter1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueCloseStart)
                        // 2+0
                        .lineToLinearHeading(new Pose2d(28, 25, Math.toRadians(180))) // To spike mark
                        .waitSeconds(0.5) // Place purple pixel
                        .lineToLinearHeading(new Pose2d(51, 35, Math.toRadians(180))) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        // 2+2
                        .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
                        .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180))) // To stack across field
                        .waitSeconds(1) // Intake from stack
                        .lineToLinearHeading(new Pose2d(38, 11, Math.toRadians(180))) // Return across field
                        .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(90)) // To backboard
                        .waitSeconds(1) // Place pixels on backboard
                        // 2+4
                        .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
                        .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180))) // To stack across field
                        .waitSeconds(1) // Intake from stack
                        .lineToLinearHeading(new Pose2d(38, 11, Math.toRadians(180))) // Return across field
                        .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(90)) // To backboard
                        .waitSeconds(1) // Place pixels on backboard
                        .splineToLinearHeading(new Pose2d(52, 11, Math.toRadians(180)),Math.toRadians(15)) // To park
                .build());
        RoadRunnerBotEntity blueLeft1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueCloseStart)
                        // 2+0
                        .lineToLinearHeading(new Pose2d(35, 35, Math.toRadians(180))) // To spike mark
                        .waitSeconds(0.5) // Place purple pixel
                        .lineToLinearHeading(new Pose2d(51, 41, Math.toRadians(180))) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        .build());
        RoadRunnerBotEntity blueRight1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueCloseStart)
                        // 2+0
                        .back(2)
                        .splineToConstantHeading(new Vector2d(25, 40), Math.toRadians(300))
                        .splineToSplineHeading(new Pose2d(12, 33, Math.toRadians(180)), Math.toRadians(180))
                        .waitSeconds(0.5) // Place purple pixel
                        .lineToLinearHeading(new Pose2d(51, 29, Math.toRadians(180))) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        .build());
        // FAR
        Pose2d blueFarStart = new Pose2d(-35,60,Math.toRadians(90));
        RoadRunnerBotEntity blueRight2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueFarStart)
                        // 2+1
                        .lineToSplineHeading(new Pose2d(-46, 15, Math.toRadians(90))) // To spike mark
                        .waitSeconds(0.5) // Place purple pixel
                        .splineToLinearHeading(new Pose2d(-35, 11, Math.toRadians(180)), Math.toRadians(0))
                        .forward(25)
                        .waitSeconds(1) // Intake from stack
                        .lineToSplineHeading(new Pose2d(25, 11, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(51, 29), Math.toRadians(0)) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        .splineToLinearHeading(new Pose2d(52, 11, Math.toRadians(180)),Math.toRadians(15)) // To park
                        .build());
        RoadRunnerBotEntity blueLeft2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueFarStart)
                        // 2+1
                        .lineToSplineHeading(new Pose2d(-46, 15, Math.toRadians(90))) // To spike mark
                        .waitSeconds(0.5) // Place purple pixel
                        .splineToLinearHeading(new Pose2d(-35, 11, Math.toRadians(180)), Math.toRadians(0))
                        .forward(25)
                        .waitSeconds(1) // Intake from stack
                        .lineToSplineHeading(new Pose2d(25, 11, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(51, 29), Math.toRadians(0)) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        .splineToLinearHeading(new Pose2d(52, 11, Math.toRadians(180)),Math.toRadians(15)) // To park
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .addEntity(blueRight2)
                .start();
    }
}


// OLD MEEP MEEP TRAJECTORIES TO SAVE :D
//        RoadRunnerBotEntity blueCloseRight = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeBlueDark())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(blueCloseStart) // Starting position
//                                        .splineToLinearHeading(new Pose2d(13, 32, Math.toRadians(0)), Math.toRadians(180))
//                                        .waitSeconds(0.5) // Wait for score
//                                        .lineToLinearHeading(new Pose2d(50, 29, Math.toRadians(180))) // To backboard
//                                        .waitSeconds(0.5) // Wait for score
//                                        .splineToLinearHeading(new Pose2d(55, 11, Math.toRadians(180)),Math.toRadians(15))
//                                        .waitSeconds(2)
//                                        .build()
//                );
//        Pose2d redCloseStart = new Pose2d(12,-60,Math.toRadians(-90));
//        RoadRunnerBotEntity redCloseCenter = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeRedDark())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(redCloseStart) // Starting position
//                                        .lineToLinearHeading(new Pose2d(17, -36, Math.toRadians(-90)))
//                                        .waitSeconds(0.5) // Wait to score
//                                        .splineToLinearHeading(new Pose2d(42, -35, Math.toRadians(180)), Math.toRadians(180)) // Spline to backboard
//                                        .back(10) // Back into backboard
//                                        .waitSeconds(0.5) // Wait to score
//                                        .forward(10) // Back up
//                                        .strafeRight(23) // Start pixel stack trajectory
//                                        .forward(100) // Drive across field
//                                        .waitSeconds(3) // Wait for intake from stack
//                                        .back(100)
//                                        .strafeLeft(23) // Strafe to center of backboard
//                                        .back(10) // Back into backboard
//                                        .waitSeconds(0.5) // Wait to score
//                                        .forward(5) // Back up
//                                        .strafeRight(23) // Park on right side
//                                        .waitSeconds(2)
//                                        .build()
//                );