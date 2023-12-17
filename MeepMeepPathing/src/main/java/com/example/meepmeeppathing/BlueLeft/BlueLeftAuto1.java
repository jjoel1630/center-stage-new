package com.example.meepmeeppathing.BlueLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeftAuto1 {
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
                                drive.trajectorySequenceBuilder(new Pose2d(-34.27, 64.04, Math.toRadians(-90.00)))
                                        .splineTo(new Vector2d(-48.12, 37.51), Math.toRadians(-87.52))
                                        .splineTo(new Vector2d(-38.40, 37.66), Math.toRadians(-5.04))
                                        .splineTo(new Vector2d(-49.74, 54.76), Math.toRadians(108.24))
                                        .splineTo(new Vector2d(-20.86, 59.32), Math.toRadians(0.83))
                                        .splineTo(new Vector2d(15.25, 58.88), Math.toRadians(1.55))
                                        .splineTo(new Vector2d(47.68, 36.33), Math.toRadians(0.00))
                                        .build()
                        // place pixel 2
                        /*drive.trajectorySequenceBuilder(new Pose2d(-30.14, 63.60, Math.toRadians(-90.00)))
                                .splineTo(new Vector2d(-60.36, 38.40), Math.toRadians(-88.07))
                                .splineTo(new Vector2d(-50.48, 24.25), Math.toRadians(3.73))
                                .splineTo(new Vector2d(-52.99, 52.10), Math.toRadians(91.30))
                                .splineTo(new Vector2d(-21.74, 59.77), Math.toRadians(-6.62))
                                .splineTo(new Vector2d(15.25, 59.91), Math.toRadians(-1.31))
                                .splineTo(new Vector2d(49.30, 37.36), Math.toRadians(-0.76))
                                .build()*/
                        // place pixel 3
      /*                  drive.trajectorySequenceBuilder(new Pose2d(-29.99, 65.66, Math.toRadians(-90.00)))
                                .splineTo(new Vector2d(-50.04, 49.30), Math.toRadians(213.27))
                                .splineTo(new Vector2d(-51.51, 26.90), Math.toRadians(-78.25))
                                .splineTo(new Vector2d(-43.26, 44.14), Math.toRadians(1.25))
                                .splineTo(new Vector2d(-30.14, 40.16), Math.toRadians(-11.14))
                                .splineTo(new Vector2d(-26.01, 34.56), Math.toRadians(-86.10))
                                .splineTo(new Vector2d(-51.22, 41.20), Math.toRadians(142.32))
                                .splineTo(new Vector2d(-23.36, 59.47), Math.toRadians(0.00))
                                .splineTo(new Vector2d(24.54, 59.32), Math.toRadians(-2.37))
                                .splineTo(new Vector2d(48.86, 35.15), Math.toRadians(1.82))
                                .build()*/
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