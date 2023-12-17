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
            MeepMeep meepMeep = new MeepMeep(800);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                    // Option: Set theme. Default = ColorSchemeRedDark()
                    .followTrajectorySequence(drive ->

                            // place pixel 1
                               drive.trajectorySequenceBuilder(new Pose2d(-36.74, -60.32, Math.toRadians(90.00)))
                                     .splineTo(new Vector2d(-47.36, -34.62), Math.toRadians(112.45))
                                     .build()
/*
                             //place pixel 2
                               drive.trajectorySequenceBuilder(new Pose2d(-36.18, -59.77, Math.toRadians(90.00)))
                                     .splineTo(new Vector2d(-41.20, -24.84), Math.toRadians(98.16))
                                     .build()


                            //place pixel 3
                               drive.trajectorySequenceBuilder(new Pose2d(-36.63, -61.39, Math.toRadians(90.00)))
                                    .splineTo(new Vector2d(-24.84, -35.45), Math.toRadians(65.56))
                                    .build() */



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

