package com.example.meepmeeppathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeftLobster {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d start = new Pose2d(-39.87, -65.50, Math.toRadians(90.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(start)
//                                .lineToConstantHeading(new Vector2d(-48.00, -42.00))
//                                .waitSeconds(0.8)
//                                .lineToLinearHeading(new Pose2d(-48.00, -60.00, Math.toRadians(0.00)))
//                                .lineToConstantHeading(new Vector2d(24.00, -60.00))
//                                .lineToLinearHeading(new Pose2d(38, -33, Math.toRadians(180.00)))
//                                .build()
//                        drive.trajectorySequenceBuilder(start)
//                                .lineToConstantHeading(new Vector2d(-39.00, -34.50))
//                                .waitSeconds(0.8)
//                                .lineToLinearHeading(new Pose2d(-48.00, -60.00, Math.toRadians(0.00)))
//                                .lineToConstantHeading(new Vector2d(24.00, -60.00))
//                                .lineToLinearHeading(new Pose2d(38, -36.00, Math.toRadians(180.00)))
//                                .build()
                        drive.trajectorySequenceBuilder(new Pose2d(-39.87, -65.50, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-35.00, -34.00), Math.toRadians(0.00))
                                .waitSeconds(0.8)
                                .lineToConstantHeading(new Vector2d(-48.00, -40.00))
                                .lineToConstantHeading(new Vector2d(-40.00, -60))
                                .lineToConstantHeading(new Vector2d(30.00, -60.00))
                                .lineToLinearHeading(new Pose2d(36, -44.00, Math.toRadians(180.00)))
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