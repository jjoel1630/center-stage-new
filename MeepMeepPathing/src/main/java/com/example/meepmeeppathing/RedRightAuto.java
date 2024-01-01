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
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .followTrajectorySequence(drive ->
                                // place pixel 1
                                drive.trajectorySequenceBuilder(new Pose2d(12.45, 62.86, Math.toRadians(-90.00)))
                                        .splineTo(new Vector2d(5.53, 32.20), Math.toRadians(257.27))
                                        .splineTo(new Vector2d(9.06, 26.16), Math.toRadians(-23.70))
                                        .splineTo(new Vector2d(51.22, 30.73), Math.toRadians(4.56))
                                        .splineTo(new Vector2d(49.74, 33.09), Math.toRadians(122.01))
                                        .splineTo(new Vector2d(46.35, 42.37), Math.toRadians(110.06))
                                        .splineTo(new Vector2d(51.07, 56.97), Math.toRadians(72.09))
                                        .splineTo(new Vector2d(63.30, 59.03), Math.toRadians(-8.97))
                                        .build()
                       /* // place pixel 2


/*                        // place pixel 3


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