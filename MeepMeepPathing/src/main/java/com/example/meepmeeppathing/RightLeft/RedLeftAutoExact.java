package com.example.meepmeeppathing.RightLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

    public class RedLeftAutoExact {
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
                                        .build()
                            //place pixel 2
                                    drive.trajectorySequenceBuilder(new Pose2d(-38.65, -62.65, Math.toRadians(90.00)))
                                            .splineTo(new Vector2d(-52.67, -30.16), Math.toRadians(96.57))
                                            .build() */


                                 //place pixel 3
                                    drive.trajectorySequenceBuilder(new Pose2d(-38.44, -63.08, Math.toRadians(90.00)))
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

