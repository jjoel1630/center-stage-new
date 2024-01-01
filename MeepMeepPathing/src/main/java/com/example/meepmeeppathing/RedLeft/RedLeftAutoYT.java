package com.example.meepmeeppathing.RedLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeftAutoYT {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .followTrajectorySequence(drive ->
                                // place pixel 1
//                        drive.trajectorySequenceBuilder(new Pose2d(-38.87, -62.65, Math.toRadians(90.00)))
//                                .splineTo(new Vector2d(-47.36, -35.47), Math.toRadians(103.83))
//                                .splineTo(new Vector2d(-44.39, -19.96), Math.toRadians(64.13))
//                                .lineToLinearHeading(new Pose2d(-32.92, -6.16, Math.toRadians(0.77)))
//                                .splineTo(new Vector2d(0.85, -8.28), Math.toRadians(-3.53))
//                                .splineTo(new Vector2d(35.04, -11.04), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(49.06, -34.83), Math.toRadians(-45.00))
//                                .build()

                        // place pixel 2
//                                drive.trajectorySequenceBuilder(new Pose2d(-38.87, -62.87, Math.toRadians(90.00)))
//                                        .lineToLinearHeading(new Pose2d(-41.20, -24.64, Math.toRadians(104.46)))
//                                        .splineTo(new Vector2d(-24.64, -9.77), Math.toRadians(0.00))
//                                        .splineTo(new Vector2d(27.61, -10.41), Math.toRadians(-2.39))
//                                        .lineToSplineHeading(new Pose2d(49.06, -32.92, Math.toRadians(0.00)))
//                                        .build()
                        // place pixel 3
                                drive.trajectorySequenceBuilder(new Pose2d(-38.87, -62.87, Math.toRadians(90.00)))
                                        .lineToLinearHeading(new Pose2d(-47.36, -37.59, Math.toRadians(90.00)))
                                        .lineToLinearHeading(new Pose2d(-25.06, -35.89, Math.toRadians(90.00)))
                                        .lineToLinearHeading(new Pose2d(-50.76, -30.37, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(-24.64, -9.77), Math.toRadians(0.00))
                                        .splineTo(new Vector2d(27.61, -10.41), Math.toRadians(-2.39))
                                        .lineToSplineHeading(new Pose2d(49.06, -32.92, Math.toRadians(0.00)))
                                        .build()


                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}