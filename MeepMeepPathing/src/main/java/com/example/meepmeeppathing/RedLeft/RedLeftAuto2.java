package com.example.meepmeeppathing.RedLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeftAuto2 {
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
                                /*drive.trajectorySequenceBuilder(new Pose2d(-38.44, -63.08, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(-47.36, -35.04), Math.toRadians(91.73))
                                        .splineTo(new Vector2d(-26.97, -34.83), Math.toRadians(-4.47))
                                        .splineTo(new Vector2d(50.34, -36.11), Math.toRadians(0.00))
                                        .build()

                                //place pixel 2
                                drive.trajectorySequenceBuilder(new Pose2d(-38.44, -63.08, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(-40.99, -24.85), Math.toRadians(91.73))
                                        .splineTo(new Vector2d(-23.36, -35.26), Math.toRadians(-4.47))
                                        .splineTo(new Vector2d(50.34, -36.11), Math.toRadians(0.00))
                                        .build() */


                                //place pixel 3
                                drive.trajectorySequenceBuilder(new Pose2d(-38.44, -63.08, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(-52.88, -26.34), Math.toRadians(91.73))
                                        .splineTo(new Vector2d(-25.91, -34.62), Math.toRadians(-20.03))
                                        .splineTo(new Vector2d(-14.44, -35.68), Math.toRadians(-4.47))
                                        .splineTo(new Vector2d(50.34, -36.11), Math.toRadians(0.00))
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

