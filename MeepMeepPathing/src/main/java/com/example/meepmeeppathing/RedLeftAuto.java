package com.example.meepmeeppathing;

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
//                                drive.trajectorySequenceBuilder(new Pose2d(-38.02, -60.11, Math.toRadians(90.00)))
//                                        .splineTo(new Vector2d(-50.76, -54.37), Math.toRadians(129.47))
//                                        .splineTo(new Vector2d(-58.41, -35.47), Math.toRadians(86.42))
//                                        .splineTo(new Vector2d(-53.73, -14.44), Math.toRadians(-87.06))
//                                        .splineTo(new Vector2d(-52.67, -33.35), Math.toRadians(-11.31))
//                                        .splineTo(new Vector2d(-47.36, -34.19), Math.toRadians(-9.09))
//                                        .splineTo(new Vector2d(1.06, -35.68), Math.toRadians(0.50))
//                                        .splineTo(new Vector2d(49.06, -36.11), Math.toRadians(0.00))
//                                        .build()
                        // place pixel 2
//                                drive.trajectorySequenceBuilder(new Pose2d(-38.65, -62.87, Math.toRadians(90.00)))
//                                        .splineTo(new Vector2d(-50.76, -54.37), Math.toRadians(129.47))
//                                        .splineTo(new Vector2d(-58.41, -35.47), Math.toRadians(86.42))
//                                        .splineTo(new Vector2d(-53.73, -14.44), Math.toRadians(-87.06))
//                                        .splineTo(new Vector2d(-52.67, -33.35), Math.toRadians(-11.31))
//                                        .splineTo(new Vector2d(-47.36, -34.19), Math.toRadians(-9.09))
//                                        .splineTo(new Vector2d(1.06, -35.68), Math.toRadians(0.50))
//                                        .splineTo(new Vector2d(49.06, -36.11), Math.toRadians(0.00))
//                                        .build()


                        // place pixel 3
                        drive.trajectorySequenceBuilder(new Pose2d(-38.44, -62.87, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-54.80, -34.41), Math.toRadians(115.04))
                                .splineTo(new Vector2d(-36.11, -34.83), Math.toRadians(1.29))
                                .splineTo(new Vector2d(-24.85, -35.26), Math.toRadians(-2.20))
                                .splineTo(new Vector2d(49.91, -35.89), Math.toRadians(0.73))
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