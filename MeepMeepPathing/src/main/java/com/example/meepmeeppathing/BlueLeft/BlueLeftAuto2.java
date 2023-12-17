package com.example.meepmeeppathing.BlueLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeftAuto2 {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .followTrajectorySequence(drive ->
                                // place pixel 1
/*                               drive.trajectorySequenceBuilder(new Pose2d(-28.23, 66.69, Math.toRadians(-90.00)))
                                        .splineTo(new Vector2d(-47.53, 37.07), Math.toRadians(270.00))
                                        .splineTo(new Vector2d(-28.23, 35.59), Math.toRadians(7.83))
                                        .splineTo(new Vector2d(11.28, 34.42), Math.toRadians(-3.66))
                                        .splineTo(new Vector2d(48.86, 34.27), Math.toRadians(-0.97))
                                        .build()*/
                        // place pixel 2
                  /*      drive.trajectorySequenceBuilder(new Pose2d(-28.23, 66.10, Math.toRadians(-90.00)))
                                .splineTo(new Vector2d(-58.29, 35.45), Math.toRadians(270.00))
                                .splineTo(new Vector2d(-46.50, 25.57), Math.toRadians(0.00))
                                .splineTo(new Vector2d(-22.48, 35.15), Math.toRadians(-3.16))
                                .splineTo(new Vector2d(47.53, 36.33), Math.toRadians(-1.29))
                                .build()*/
                        // place pixel 3
                        drive.trajectorySequenceBuilder(new Pose2d(-28.81, 65.51, Math.toRadians(-90.00)))
                                .splineTo(new Vector2d(-47.53, 48.42), Math.toRadians(222.41))
                                .splineTo(new Vector2d(-35.74, 46.06), Math.toRadians(-11.31))
                                .splineTo(new Vector2d(-29.85, 40.61), Math.toRadians(-42.77))
                                .splineTo(new Vector2d(-24.25, 40.02), Math.toRadians(255.96))
                                .splineTo(new Vector2d(-9.65, 35.30), Math.toRadians(15.52))
                                .splineTo(new Vector2d(24.69, 34.42), Math.toRadians(3.66))
                                .splineTo(new Vector2d(49.01, 35.15), Math.toRadians(-1.83))
                                .build()
//
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
