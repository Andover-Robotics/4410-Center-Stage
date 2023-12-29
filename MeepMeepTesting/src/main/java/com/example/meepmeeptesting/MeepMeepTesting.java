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

        // BLUE ALLIANCE TRAJECTORIES
        Pose2d blueCloseStart = new Pose2d(12,60,Math.toRadians(90));
        Pose2d blueFarStart = new Pose2d(-35,60,Math.toRadians(90));

        // CLOSE SIDE
        RoadRunnerBotEntity blueCloseLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(blueCloseStart) // Starting position
                                        .lineToLinearHeading(new Pose2d(35, 30, Math.toRadians(0))) // Line to spike mark
                                        .waitSeconds(0.5) // Wait for score
                                        .lineToLinearHeading(new Pose2d(35, 55, Math.toRadians(180)))
                                        .lineToLinearHeading(new Pose2d(45, 40, Math.toRadians(180))) // To backboard
                                        .back(5)
                                        .waitSeconds(0.5) // Wait for score
                                        .forward(8)

//                                // GO THROUGH SIDE TRUSS
//                                .strafeRight(18)
//                                .forward(90) // Drive across field
//                                .strafeLeft(22)
//                                .forward(10)
//                                .waitSeconds(3) // Wait for intake from stack
//                                .lineToLinearHeading(new Pose2d(-50, 11, Math.toRadians(180)))
//                                .back(90)
//                                .strafeRight(23)
//                                .back(10)
//                                .waitSeconds(0.5) // Wait to score
//                                .forward(5) // Back up

                                        // GO THROUGH MIDDLE TRUSS
                                        .strafeLeft(28) // Start pixel stack trajectory
                                        .forward(100) // Drive across field
                                        .waitSeconds(3) // Wait for intake from stack
                                        .back(100)
                                        .strafeRight(23) // Strafe to center of backboard
                                        .back(10) // Back into backboard
                                        .waitSeconds(0.5) // Wait to score
                                        .forward(5) // Back up
                                        .strafeLeft(23) // Park on right side

                                        .waitSeconds(2)
                                        .build()
                );

        RoadRunnerBotEntity blueCloseCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(blueCloseStart) // Starting position
                                        .lineToLinearHeading(new Pose2d(18, 38, Math.toRadians(90)))
                                        .waitSeconds(0.5) // Wait to score
                                        .splineToLinearHeading(new Pose2d(42, 35, Math.toRadians(180)), Math.toRadians(180)) // Spline to backboard
                                        .back(10) // Back into backboard
                                        .waitSeconds(0.5) // Wait to score
                                        .forward(8) // Back up

                                        // GO THROUGH SIDE TRUSS
                                        .forward(100) // Drive across field
                                        .waitSeconds(3) // Wait for intake from stack
                                        .back(110)
                                        .waitSeconds(0.5) // Wait to score
                                        .forward(5) // Back up
                                        .strafeRight(23) // Park on left side

////
//                                // GO THROUGH MIDDLE TRUSS
//                                .strafeLeft(25) // Start pixel stack trajectory
//                                .forward(103) // Drive across field
//                                .waitSeconds(3) // Wait for intake from stack
//                                .back(100)
//                                .strafeRight(23) // Strafe to center of backboard
//                                .back(10) // Back into backboard
//                                .waitSeconds(0.5) // Wait to score
//                                .forward(5) // Back up
//                                .strafeLeft(23) // Park on right side

                                        .waitSeconds(2)
                                        .build()
                );

        RoadRunnerBotEntity blueCloseRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(blueCloseStart) // Starting position
                                        .lineToLinearHeading(new Pose2d(13, 30, Math.toRadians(0))) // Line to spike mark
                                        .waitSeconds(0.5) // Wait for score
                                        .lineToLinearHeading(new Pose2d(45, 30, Math.toRadians(180))) // To backboard
                                        .back(5)
                                        .waitSeconds(0.5) // Wait for score
                                        .forward(8)

//                                // GO THROUGH SIDE TRUSS
//                                .strafeRight(28)
//                                .forward(90) // Drive across field
//                                .strafeLeft(22)
//                                .forward(10)
//                                .waitSeconds(3) // Wait for intake from stack
//                                .lineToLinearHeading(new Pose2d(-50, 11, Math.toRadians(180)))
//                                .back(90)
//                                .strafeRight(23)
//                                .back(10)
//                                .waitSeconds(0.5) // Wait to score
//                                .forward(5) // Back up

                                        // GO THROUGH MIDDLE TRUSS
                                        .strafeLeft(18) // Start pixel stack trajectory
                                        .forward(100) // Drive across field
                                        .waitSeconds(3) // Wait for intake from stack
                                        .back(100)
                                        .strafeRight(23) // Strafe to center of backboard
                                        .back(10) // Back into backboard
                                        .waitSeconds(0.5) // Wait to score
                                        .forward(5) // Back up
                                        .strafeLeft(23) // Park on right side

                                        .waitSeconds(2)
                                        .build()
                );

        // FAR SIDE
        RoadRunnerBotEntity blueFarLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueFarStart) // Starting position
                                .lineToLinearHeading(new Pose2d(-37, 30, Math.toRadians(180))) // Line to spike mark
                                .waitSeconds(0.5) // Wait to score
                                .strafeLeft(17) // Strafe to center truss
                                .back(78) // Drive to backboard
                                .strafeRight(27) // Strafe to center
                                .back(10) // Run into backboard
                                .waitSeconds(0.5) // Wait to score
                                .forward(5)
                                .strafeRight(17)

                                .waitSeconds(2)
                                .build()
                );

        RoadRunnerBotEntity blueFarCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueFarStart) // Starting position
                                .lineToLinearHeading(new Pose2d(-50, 25, Math.toRadians(180))) // Line to spike mark
                                .waitSeconds(0.5) // Wait to score
                                .lineToLinearHeading(new Pose2d(-35, 14, Math.toRadians(180)))
                                .strafeLeft(12) // Strafe to center truss
                                .back(90) // Drive to backboard
                                .waitSeconds(2) // Wait for close auto to finish
                                .strafeRight(22) // Strafe to center
                                .back(10) // Run into backboard
                                .waitSeconds(0.5) // Wait to score
                                .forward(5)
                                .strafeLeft(23)

                                .waitSeconds(2)
                                .build()
                );

        RoadRunnerBotEntity blueFarRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueFarStart) // Starting position
                                .lineToLinearHeading(new Pose2d(-35,50, Math.toRadians(60))) // Line to spike mark
                                .waitSeconds(0.5) // Wait to score
                                .lineToLinearHeading(new Pose2d(-35, 13, Math.toRadians(180)))
                                .back(80) // Drive across field
                                .strafeRight(15)
                                .waitSeconds(0.5) // Wait to score

                                .waitSeconds(2)
                                .build()
                );

        // RED ALLIANCE TRAJECTORIES
        Pose2d redCloseStart = new Pose2d(12,-60,Math.toRadians(-90));
        RoadRunnerBotEntity redCloseCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(redCloseStart) // Starting position
//                                .back(36)
//                                .forward(12) // Distance away from spike mark when scoring
                                        .lineToLinearHeading(new Pose2d(17, -36, Math.toRadians(-90)))
                                        .waitSeconds(0.5) // Wait to score
                                        .splineToLinearHeading(new Pose2d(42, -35, Math.toRadians(180)), Math.toRadians(180)) // Spline to backboard
                                        .back(10) // Back into backboard
                                        .waitSeconds(0.5) // Wait to score
                                        .forward(10) // Back up

//                                // GO THROUGH SIDE TRUSS
//                                .forward(100) // Drive across field
//                                .waitSeconds(3) // Wait for intake from stack
//                                .back(110)
//                                .waitSeconds(0.5) // Wait to score
//                                .forward(5) // Back up
//                                .strafeRight(23) // Park on left side


                                        // GO THROUGH MIDDLE TRUSS
                                        .strafeRight(23) // Start pixel stack trajectory
                                        .forward(100) // Drive across field
                                        .waitSeconds(3) // Wait for intake from stack
                                        .back(100)
                                        .strafeLeft(23) // Strafe to center of backboard
                                        .back(10) // Back into backboard
                                        .waitSeconds(0.5) // Wait to score
                                        .forward(5) // Back up
                                        .strafeRight(23) // Park on right side

                                        .waitSeconds(2)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .addEntity(blueFarRight)
                .start();
    }
}