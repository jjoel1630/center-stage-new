package com.example.meepmeeppathing.BlueLeft;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeftAuto {
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
                        drive.trajectorySequenceBuilder(new Pose2d(-36.37, -62.20, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-35.93, -48.84), Math.toRadians(90.00))
                                .lineToLinearHeading(new Pose2d(-43.94, -41.57, Math.toRadians(124.11)))
                                .lineToLinearHeading(new Pose2d(-51.66, -53.59, Math.toRadians(0.00)))
                                .splineToConstantHeading(new Vector2d(-16.63, -60.42), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(27.46, -56.26), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(50.77, -36.67), Math.toRadians(0.00))
                                .build()
                        // place pixel 2
//                        drive.trajectorySequenceBuilder(new Pose2d(-36.37, -62.20, Math.toRadians(90.00)))
//                                .splineTo(new Vector2d(-35.93, -48.84), Math.toRadians(90.00))
//                                .lineToLinearHeading(new Pose2d(-35.78, -32.66, Math.toRadians(90.00)))
//                                .lineToLinearHeading(new Pose2d(-51.66, -53.59, Math.toRadians(0.00)))
//                                .splineToConstantHeading(new Vector2d(-16.63, -60.42), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(27.46, -56.26), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(50.77, -36.67), Math.toRadians(0.00))
//                                .build()
                        // place pixel 3
//                        drive.trajectorySequenceBuilder(new Pose2d(-36.37, -62.20, Math.toRadians(90.00)))
//                                .splineTo(new Vector2d(-35.93, -48.84), Math.toRadians(90.00))
//                                .lineToLinearHeading(new Pose2d(-33.70, -35.93, Math.toRadians(34.11)))
//                                .lineToLinearHeading(new Pose2d(-51.66, -53.59, Math.toRadians(0.00)))
//                                .splineToConstantHeading(new Vector2d(-16.63, -60.42), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(27.46, -56.26), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(50.77, -36.67), Math.toRadians(0.00))
//                                .build()
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