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

        // CLOSE BLUE
        Pose2d blueCloseStart = new Pose2d(12,60,Math.toRadians(90));
        RoadRunnerBotEntity blueCenter1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueCloseStart)
                        // 2+0
                        .lineToLinearHeading(new Pose2d(23, 32, Math.toRadians(60))) // To spike mark
                        .waitSeconds(1) // Place purple pixel
                        .lineToLinearHeading(new Pose2d(51, 35, Math.toRadians(180))) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        // 2+2
                        .splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)), Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d( -59+8, 10, Math.toRadians(180)))
//                        .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
//                        .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180))) // To stack across field
                        .waitSeconds(1) // Intake from stack
                        .lineToLinearHeading(new Pose2d(38, 11, Math.toRadians(180))) // Return across field
                        .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(0)) // To backboard
                        .waitSeconds(1) // Place pixels on backboard
                        // 2+4
                        .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
                        .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180))) // To stack across field
                        .waitSeconds(1) // Intake from stack
                        .lineToLinearHeading(new Pose2d(38, 11, Math.toRadians(180))) // Return across field
                        .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(0)) // To backboard
                        .waitSeconds(1) // Place pixels on backboard
                        .splineToLinearHeading(new Pose2d(50, 11, Math.toRadians(180)),Math.toRadians(15)) // To park
                .build());
        RoadRunnerBotEntity blueLeft1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueCloseStart)
                        // 2+0
                        .lineToLinearHeading(new Pose2d(17, 40, Math.toRadians(120))) // To spike mark
                        .waitSeconds(1) // Place purple pixel
                        .splineToLinearHeading(new Pose2d(48, 41, Math.toRadians(180)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(51, 41, Math.toRadians(180))) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        .splineToLinearHeading(new Pose2d(50, 59, Math.toRadians(180)),Math.toRadians(15)) // To park
                        .build());
        RoadRunnerBotEntity blueRight1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueCloseStart)
                        // 2+0
                        .lineToSplineHeading(new Pose2d(13,58,Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(20, 40), Math.toRadians(225))
                        .splineToSplineHeading(new Pose2d(12, 33, Math.toRadians(0)), Math.toRadians(225))
                        .waitSeconds(1) // Place purple pixel
                        .lineToLinearHeading(new Pose2d(51, 29, Math.toRadians(180))) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        .lineToLinearHeading(new Pose2d(50, 59, Math.toRadians(180))) // To park
                        .build());
        // FAR
        Pose2d blueFarStart = new Pose2d(-35,60,Math.toRadians(90));
        RoadRunnerBotEntity blueCenter2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueFarStart)
                        // 2+1
                        .lineToLinearHeading(new Pose2d(-50, 22, Math.toRadians(180))) // To spike mark
                        .waitSeconds(1) // Place purple pixel
                        .lineToSplineHeading(new Pose2d(-59, 11, Math.toRadians(180)))
                        .waitSeconds(1) // Intake from stack
                        .lineToSplineHeading(new Pose2d(25, 11, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(51, 29), Math.toRadians(0)) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        // 2+3
                        .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
                        .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180))) // To stack across field
                        .waitSeconds(1) // Intake from stack
                        .lineToLinearHeading(new Pose2d(38, 11, Math.toRadians(180))) // Return across field
                        .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(0)) // To backboard
                        .waitSeconds(1)
                        // 2+5
                        .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
                        .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180))) // To stack across field
                        .waitSeconds(1) // Intake from stack
                        .lineToLinearHeading(new Pose2d(38, 11, Math.toRadians(180))) // Return across field
                        .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(0)) // To backboard
                        .waitSeconds(1)
                        .build());
        RoadRunnerBotEntity blueLeft2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueFarStart)
                        // 2+1
                        .lineToSplineHeading(new Pose2d(-42, 46, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(-35, 34, Math.toRadians(180)), Math.toRadians(0)) // To spike mark
                        .waitSeconds(0.5) // Place purple pixel
                        .lineToSplineHeading(new Pose2d(-48, 16, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(-60, 11), Math.toRadians(180)) // To stack
                        .waitSeconds(1) // Intake from stack
                        .lineToSplineHeading(new Pose2d(25, 11, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(51, 29), Math.toRadians(0)) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        .build());
        RoadRunnerBotEntity blueRight2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(blueFarStart)
                        // 2+1
                        .lineToLinearHeading(new Pose2d(-47, 13, Math.toRadians(270))) // To spike mark
                        .waitSeconds(0.5) // Place purple pixel
                        .lineToLinearHeading(new Pose2d(-60, 11, Math.toRadians(180)))
                        .waitSeconds(1) // Intake from stack
                        .lineToSplineHeading(new Pose2d(25, 11, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(51, 29), Math.toRadians(0)) // To backboard
                        .waitSeconds(1) // Place yellow pixel
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .addEntity(blueCenter1)
                .start();
    }
}

//    // CLOSE RED
//    Pose2d redCloseStart = new Pose2d(12,-60,Math.toRadians(90));
//    RoadRunnerBotEntity redCenter1 = new DefaultBotBuilder(meepMeep)
//            .setColorScheme(new ColorSchemeRedLight())
//            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
//            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(redCloseStart)
//                    // 2+0
//                    .lineToLinearHeading(new Pose2d(28, -25, Math.toRadians(180))) // To spike mark
//                    .waitSeconds(0.5) // Place purple pixel
//                    .lineToLinearHeading(new Pose2d(51, -35, Math.toRadians(180))) // To backboard
//                    .waitSeconds(1) // Place yellow pixel
//                    .splineToLinearHeading(new Pose2d(55, -11, Math.toRadians(180)),Math.toRadians(20))
//                    .waitSeconds(1)
//                    // 2+2
//                    .splineToLinearHeading(new Pose2d(30, -11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
//                    .lineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(180))) // To stack across field
//                    .waitSeconds(1) // Intake from stack
//                    .lineToLinearHeading(new Pose2d(38, -11, Math.toRadians(180))) // Return across field
//                    .splineToLinearHeading(new Pose2d(50, -30, Math.toRadians(180)), Math.toRadians(0)) // To backboard
//                    .waitSeconds(1) // Place pixels on backboard
//                    // 2+4
//                    .splineToLinearHeading(new Pose2d(30, -11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
//                    .lineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(180))) // To stack across field
//                    .waitSeconds(1) // Intake from stack
//                    .lineToLinearHeading(new Pose2d(38, -11, Math.toRadians(180))) // Return across field
//                    .splineToLinearHeading(new Pose2d(50, -30, Math.toRadians(180)), Math.toRadians(0)) // To backboard
//                    .waitSeconds(1) // Place pixels on backboard
//                    .splineToLinearHeading(new Pose2d(52, -11, Math.toRadians(180)),Math.toRadians(15)) // To park
//                    .build());
//    // RED FAR
//    Pose2d redFarStart = new Pose2d(-36,-60,Math.toRadians(90));
//    RoadRunnerBotEntity redLeft2 = new DefaultBotBuilder(meepMeep)
//            .setColorScheme(new ColorSchemeRedLight())
//            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
//            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(redFarStart)
//                    // 2+1
//                    .lineToSplineHeading(new Pose2d(-44, -45, Math.toRadians(130)))
//                    .splineToSplineHeading(new Pose2d(-35, -35, Math.toRadians(180)), Math.toRadians(1)) // To spike mark
//                    .waitSeconds(0.5) // Place purple pixel
//                    .lineToLinearHeading(new Pose2d(-37, -24, Math.toRadians(180)))
//                    .splineToConstantHeading(new Vector2d(-59, -11), Math.toRadians(225))
//                    .waitSeconds(1) // Intake from stack
//                    .lineToSplineHeading(new Pose2d(25, -11, Math.toRadians(180)))
//                    .splineToConstantHeading(new Vector2d(51, -29), Math.toRadians(0)) // To backboard
//                    .waitSeconds(1) // Place yellow pixel
//                    // 2+3
//                    .splineToLinearHeading(new Pose2d(30, -11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
//                    .lineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(180))) // To stack across field
//                    .waitSeconds(1) // Intake from stack
//                    .lineToLinearHeading(new Pose2d(38, -11, Math.toRadians(180))) // Return across field
//                    .splineToLinearHeading(new Pose2d(50, -30, Math.toRadians(180)), Math.toRadians(0)) // To backboard
//                    .waitSeconds(1)
//                    // 2+5
//                    .splineToLinearHeading(new Pose2d(30, -11, Math.toRadians(180)), Math.toRadians(180)) // To stage door
//                    .lineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(180))) // To stack across field
//                    .waitSeconds(1) // Intake from stack
//                    .lineToLinearHeading(new Pose2d(38, -11, Math.toRadians(180))) // Return across field
//                    .splineToLinearHeading(new Pose2d(50, -30, Math.toRadians(180)), Math.toRadians(0)) // To backboard
//                    .waitSeconds(1)
//                    .build());
//    RoadRunnerBotEntity redCenter2 = new DefaultBotBuilder(meepMeep)
//            .setColorScheme(new ColorSchemeRedLight())
//            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
//            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(redFarStart)
//                    // 2+1
//                    .lineTo(new Vector2d(-38,-14)) // To spike mark
//                    .waitSeconds(0.5) // Place purple pixel
//                    .lineToSplineHeading(new Pose2d(-59, -11, Math.toRadians(180)))
//                    .waitSeconds(1) // Intake from stack
//                    .lineToSplineHeading(new Pose2d(25, -11, Math.toRadians(180)))
//                    .splineToConstantHeading(new Vector2d(51, -29), Math.toRadians(0)) // To backboard
//                    .waitSeconds(1) // Place yellow pixel
//                    .build());