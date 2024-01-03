package com.example.meepmeeppathing.BlueLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeftAuto {
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
//                            drive.trajectorySequenceBuilder(new Pose2d(-33.13, 62.87, Math.toRadians(270.00)))
//                                .lineToLinearHeading(new Pose2d(-47.15, 34.83, Math.toRadians(270.00)))
//                                .lineToSplineHeading(new Pose2d(-37.81, 34.83, Math.toRadians(0.00)))
//                                .lineToLinearHeading(new Pose2d(48.64, 34.83, Math.toRadians(0.00)))
//                                .build()
                            // place pixel 2
//                              drive.trajectorySequenceBuilder(new Pose2d(-33.13, 62.87, Math.toRadians(270.00)))
//                                  .lineToLinearHeading(new Pose2d(-40.78, 24.64, Math.toRadians(270.00)))
//                                  .lineToSplineHeading(new Pose2d(-37.81, 34.83, Math.toRadians(0.00)))
//                                  .lineToLinearHeading(new Pose2d(48.64, 34.83, Math.toRadians(0.00)))
//                                  .build()

                            // place pixel 3
                                drive.trajectorySequenceBuilder(new Pose2d(-33.35, 62.87, Math.toRadians(270.00)))
                                        .splineTo(new Vector2d(-30.37, 35.68), Math.toRadians(0.00))
                                        .lineToLinearHeading(new Pose2d(48.64, 36.11, Math.toRadians(1.15)))
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