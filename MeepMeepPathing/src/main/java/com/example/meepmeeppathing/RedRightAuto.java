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
                               /* drive.trajectorySequenceBuilder(new Pose2d(9.13, -62.44, Math.toRadians(90.00)))
                                        .lineToLinearHeading(new Pose2d(15.93, -35.89, Math.toRadians(90.00)))
                                        .lineToLinearHeading(new Pose2d(0.85, -35.47, Math.toRadians(90.00)))
                                        .lineToSplineHeading(new Pose2d(48.85, -36.53, Math.toRadians(0.00)))
                                        .lineToLinearHeading(new Pose2d(48.64, -16.99, Math.toRadians(0.00)))
                                        .build()
                        // place pixel 2
                                drive.trajectorySequenceBuilder(new Pose2d(9.13, -62.44, Math.toRadians(90.00)))
                                        .lineToLinearHeading(new Pose2d(17.20, -24.85, Math.toRadians(90.00)))
                                        .lineToSplineHeading(new Pose2d(48.85, -36.53, Math.toRadians(0.00)))
                                        .lineToLinearHeading(new Pose2d(48.64, -16.99, Math.toRadians(0.00)))
                                        .build() */

                        // place pixel 3
                        drive.trajectorySequenceBuilder(new Pose2d(9.13, -62.44, Math.toRadians(90.00)))
                                .lineToLinearHeading(new Pose2d(23.58, -35.89, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(48.85, -36.53, Math.toRadians(0.00)))
                                .lineToLinearHeading(new Pose2d(48.64, -16.99, Math.toRadians(0.00)))
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