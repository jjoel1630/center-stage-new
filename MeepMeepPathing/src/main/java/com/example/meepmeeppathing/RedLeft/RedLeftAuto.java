package com.example.meepmeeppathing.RedLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeftAuto {
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
//                                .splineTo(new Vector2d(-47.36, -34.41), Math.toRadians(103.83))
//                                .lineToLinearHeading(new Pose2d(-33.35, -35.26, Math.toRadians(0.77)))
//                                .splineTo(new Vector2d(49.91, -36.74), Math.toRadians(0.00))
//                                .build()

                        // place pixel 2
//                        drive.trajectorySequenceBuilder(new Pose2d(-38.87, -62.65, Math.toRadians(90.00)))
//                                .splineTo(new Vector2d(-40.99, -24.85), Math.toRadians(103.83))
//                                .lineToLinearHeading(new Pose2d(-33.35, -35.26, Math.toRadians(0.77)))
//                                .splineTo(new Vector2d(49.91, -36.74), Math.toRadians(0.00))
//                                .build()


                        // place pixel 3
                                drive.trajectorySequenceBuilder(new Pose2d(-38.87, -62.65, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(-40.14, -37.81), Math.toRadians(103.83))
                                        .lineToLinearHeading(new Pose2d(-40.57, -35.89, Math.toRadians(0.77)))
                                        .splineTo(new Vector2d(49.91, -36.74), Math.toRadians(0.00))
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