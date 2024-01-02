package com.example.meepmeeppathing.BlueLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeftAutoYT {
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
//                        drive.trajectorySequenceBuilder(new Pose2d(-33.13, 62.65, Math.toRadians(270.00)))
//                                .lineToLinearHeading(new Pose2d(-47.36, 36.11, Math.toRadians(270.00)))
//                                .lineToSplineHeading(new Pose2d(-39.50, 9.98, Math.toRadians(0.00)))
//                                .lineToSplineHeading(new Pose2d(23.36, 9.98, Math.toRadians(0.00)))
//                                .lineToLinearHeading(new Pose2d(48.21, 35.26, Math.toRadians(0.00)))
//                                .build()



                        // place pixel 2
//                        drive.trajectorySequenceBuilder(new Pose2d(-33.13, 62.65, Math.toRadians(270.00)))
//                                .lineToLinearHeading(new Pose2d(-40.57, 24.64, Math.toRadians(270.00)))
//                                .lineToSplineHeading(new Pose2d(-39.50, 9.98, Math.toRadians(0.00)))
//                                .lineToSplineHeading(new Pose2d(23.36, 9.98, Math.toRadians(0.00)))
//                                .lineToLinearHeading(new Pose2d(48.21, 35.26, Math.toRadians(0.00)))
//                                .build()

                        // place pixel 3
                        drive.trajectorySequenceBuilder(new Pose2d(-33.56, 63.29, Math.toRadians(270.00)))
                                .lineToLinearHeading(new Pose2d(-33.13, 38.02, Math.toRadians(270.00)))
                                .lineToLinearHeading(new Pose2d(-24.85, 35.26, Math.toRadians(270.00)))
                                .lineToSplineHeading(new Pose2d(-49.06, 25.70, Math.toRadians(0.00)))
                                .lineToLinearHeading(new Pose2d(-32.71, 8.92, Math.toRadians(0.00)))
                                .lineToLinearHeading(new Pose2d(15.08, 8.92, Math.toRadians(0.00)))
                                .lineToLinearHeading(new Pose2d(49.06, 28.04, Math.toRadians(0.00)))
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