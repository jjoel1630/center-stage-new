package com.example.meepmeeppathing.RedLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeftAuto1 {
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
                                /*drive.trajectorySequenceBuilder(new Pose2d(-38.44, -63.29, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(-47.36, -34.41), Math.toRadians(91.47))
                                        .splineTo(new Vector2d(-48.00, -55.65), Math.toRadians(12.65))
                                        .splineTo(new Vector2d(-34.41, -56.92), Math.toRadians(-19.44))
                                        .splineTo(new Vector2d(-8.92, -59.04), Math.toRadians(-1.79))
                                        .splineTo(new Vector2d(19.75, -57.35), Math.toRadians(0.00))
                                        .splineTo(new Vector2d(48.64, -36.96), Math.toRadians(3.29))
                                        .build()
                            //place pixel 2
                                    drive.trajectorySequenceBuilder(new Pose2d(-38.65, -62.65, Math.toRadians(90.00)))
                                            .splineTo(new Vector2d(-52.67, -30.16), Math.toRadians(96.57))
                                            .splineTo(new Vector2d(-42.27, -24.64), Math.toRadians(1.42))
                                            .splineTo(new Vector2d(-56.28, -54.58), Math.toRadians(239.77))
                                            .splineTo(new Vector2d(-46.30, -58.83), Math.toRadians(-20.14))
                                            .splineTo(new Vector2d(-28.25, -60.53), Math.toRadians(-5.08))
                                            .splineTo(new Vector2d(-16.78, -60.32), Math.toRadians(5.10))
                                            .splineTo(new Vector2d(-0.64, -59.68), Math.toRadians(1.05))
                                            .splineTo(new Vector2d(50.12, -37.38), Math.toRadians(2.05))
                                            .build() */


                                //place pixel 3
                                drive.trajectorySequenceBuilder(new Pose2d(-38.44, -63.08, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(-46.30, -34.19), Math.toRadians(96.27))
                                        .splineTo(new Vector2d(-25.27, -35.04), Math.toRadians(-1.65))
                                        .splineTo(new Vector2d(-37.59, -39.93), Math.toRadians(176.71))
                                        .splineTo(new Vector2d(-57.98, -50.34), Math.toRadians(255.53))
                                        .splineTo(new Vector2d(-35.68, -59.04), Math.toRadians(-8.76))
                                        .splineTo(new Vector2d(-15.08, -59.89), Math.toRadians(5.56))
                                        .splineTo(new Vector2d(-0.42, -58.83), Math.toRadians(6.26))
                                        .splineTo(new Vector2d(49.91, -37.38), Math.toRadians(0.93))
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

