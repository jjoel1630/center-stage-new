package com.example.meepmeeppathing.BlueRight;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRightAutoExact {
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
                        drive.trajectorySequenceBuilder(new Pose2d(11.42, -61.68, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(0.81, -35.45), Math.toRadians(112.02))
                                .build()

/*                         // place pixel 2
                        drive.trajectorySequenceBuilder(new Pose2d(11.42, -61.68, Math.toRadians(90.00)))
                            .splineTo(new Vector2d(0.81, -35.45), Math.toRadians(112.02))
                                .build()

                        //place pixel 3
                         drive.trajectorySequenceBuilder(new Pose2d(11.42, -61.68, Math.toRadians(90.00)))
                             .splineTo(new Vector2d(0.81, -35.45), Math.toRadians(112.02))
                                 .build()*/

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