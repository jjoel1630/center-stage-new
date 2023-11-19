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
                                drive.trajectorySequenceBuilder(new Pose2d(11.63, -62.20, Math.toRadians(90.00)))
                                        .splineTo(new Vector2d(11.63, -44.09), Math.toRadians(90.00))
                                        .lineToLinearHeading(new Pose2d(5.79, -38.00, Math.toRadians(124.11)))
                                        .lineToLinearHeading(new Pose2d(18.85, -56.56, Math.toRadians(0.00)))
                                        .splineToConstantHeading(new Vector2d(51.22, -37.56), Math.toRadians(0.00))
                                        .build()
                        // place pixel 2
//                        drive.trajectorySequenceBuilder(new Pose2d(11.63, -62.20, Math.toRadians(90.00)))
//                                .splineTo(new Vector2d(11.88, -48.69), Math.toRadians(90.00))
//                                .lineToLinearHeading(new Pose2d(11.58, -33.40, Math.toRadians(90.00)))
//                                .lineToLinearHeading(new Pose2d(18.85, -56.56, Math.toRadians(0.00)))
//                                .splineToConstantHeading(new Vector2d(51.22, -37.56), Math.toRadians(0.00))
//                                .build()
                        // place pixel 3
//                                drive.trajectorySequenceBuilder(new Pose2d(11.63, -62.20, Math.toRadians(90.00)))
//                                        .splineTo(new Vector2d(11.88, -48.69), Math.toRadians(90.00))
//                                        .lineToLinearHeading(new Pose2d(16.63, -39.19, Math.toRadians(34.11)))
//                                        .lineToLinearHeading(new Pose2d(18.85, -56.56, Math.toRadians(0.00)))
//                                        .splineToConstantHeading(new Vector2d(51.22, -37.56), Math.toRadians(0.00))
//                                        .build()
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