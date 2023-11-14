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

        Pose2d startPoseBlueFar = new Pose2d(-36, 60, -90);
        Pose2d startPoseBlueClose = new Pose2d(12, 60, -90);
        Pose2d startPoseRedClose = new Pose2d(12, -60, 90);
        Pose2d startPoseRedFar = new Pose2d(-36, -60, 90);

        Vector2d parkingPosBlue = new Vector2d(56,56);
        Vector2d parkingPosRed = new Vector2d(56,-56);
        Vector2d scoreBlue = new Vector2d(42,30);
        Vector2d scoreRed = new Vector2d(42,-30);

        RoadRunnerBotEntity redCloseBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRedClose)
                                .splineTo(scoreRed,Math.toRadians(0))
                                .waitSeconds(1.5)
                                .strafeRight(26)
                                .splineTo(parkingPosRed,Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity blueCloseBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseBlueClose)
                                .splineTo(new Vector2d(12,36),-Math.toRadians(90))
                                .waitSeconds(1.5)
                                .splineTo(scoreBlue,Math.toRadians(0))
                                .waitSeconds(1.5)
                                .strafeLeft(26)
                                .splineTo(parkingPosBlue,Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity redFarBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRedFar)
                                .splineTo(new Vector2d(-36,-36),Math.toRadians(90))
                                .waitSeconds(1.5)
                                .splineTo(scoreRed,Math.toRadians(0))
                                .waitSeconds(1.5)
                                .strafeRight(26)
                                .splineTo(parkingPosRed,Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity blueFarBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseBlueFar)
                                .splineTo(new Vector2d(-36,36),-Math.toRadians(90))
                                .waitSeconds(1.5)
                                .splineTo(scoreBlue,Math.toRadians(0))
                                .waitSeconds(1.5)
                                .strafeLeft(26)
                                .splineTo(parkingPosBlue,Math.toRadians(0))
                                .build()
                );

        //red close is done
        // blue close is done
        // blue far is done
        //red far is done

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .addEntity(blueCloseBot)
                .addEntity(redCloseBot)
                .addEntity(redFarBot)
                .addEntity(blueFarBot)
                .start();
    }
}