package com.example.meepmeeppathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeftAutoExact {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36.53, -62.02, Math.toRadians(90.00)))
//                                .splineToConstantHeading(new Vector2d(-34.83, -41.84), Math.toRadians(94.59))
//                                .splineTo(new Vector2d(-2.97, -34.41), Math.toRadians(6.69))
//                                .build()
                                drive.trajectorySequenceBuilder(new Pose2d(-36.53, -62.02, Math.toRadians(90.00)))
                                        .splineToConstantHeading(new Vector2d(-34.83, -41.84), Math.toRadians(94.59))
                                        .splineTo(new Vector2d(46.51, -39.08), Math.toRadians(6.69))
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