package com.example.meepmeeppathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedRightAuto {
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
//                                drive.trajectorySequenceBuilder(new Pose2d(8.28, -63.29, Math.toRadians(90.00)))
//                                        .splineTo(new Vector2d(17.42, -38.23), Math.toRadians(83.85))
//                                        .splineTo(new Vector2d(0.64, -35.26), Math.toRadians(167.16))
//                                        .splineTo(new Vector2d(24.64, -36.53), Math.toRadians(3.47))
//                                        .splineTo(new Vector2d(43.12, -36.11), Math.toRadians(0.95))
//                                        .splineTo(new Vector2d(50.76, -14.02), Math.toRadians(7.05))
//                                        .splineTo(new Vector2d(60.96, -13.17), Math.toRadians(-2.49))
//                                        .build()
                        // place pixel 2
//                        drive.trajectorySequenceBuilder(new Pose2d(9.56, -62.65, Math.toRadians(90.00)))
//                                .splineTo(new Vector2d(16.35, -25.06), Math.toRadians(83.85))
//                                .splineTo(new Vector2d(16.35, -9.13), Math.toRadians(-0.88))
//                                .splineTo(new Vector2d(40.14, -20.39), Math.toRadians(-74.20))
//                                .splineTo(new Vector2d(43.33, -31.65), Math.toRadians(-2.39))
//                                .splineTo(new Vector2d(44.81, -32.92), Math.toRadians(-1.40))
//                                .splineTo(new Vector2d(44.39, -11.89), Math.toRadians(3.47))
//                                .splineTo(new Vector2d(61.17, -11.26), Math.toRadians(0.62))
//                                .build()

                        // place pixel 3
                                drive.trajectorySequenceBuilder(new Pose2d(7.01, -61.59, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(23.36, -35.26), Math.toRadians(58.16))
                                        .splineTo(new Vector2d(44.39, -32.28), Math.toRadians(0.00))
                                        .splineTo(new Vector2d(52.25, -18.48), Math.toRadians(38.37))
                                        .splineTo(new Vector2d(62.23, -14.87), Math.toRadians(2.16))
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