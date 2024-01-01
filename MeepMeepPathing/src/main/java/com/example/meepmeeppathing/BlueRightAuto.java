package com.example.meepmeeppathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRightAuto {
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
                                drive.trajectorySequenceBuilder(new Pose2d(16.29, 66.25, Math.toRadians(-90.00)))
                                        .splineTo(new Vector2d(14.81, 46.94), Math.toRadians(265.63))
                                        .splineTo(new Vector2d(0.52, 33.68), Math.toRadians(-85.83))
                                        .splineTo(new Vector2d(2.43, 32.94), Math.toRadians(-21.04))
                                        .splineTo(new Vector2d(48.42, 36.92), Math.toRadians(-1.86))
                                        .splineTo(new Vector2d(49.74, 43.11), Math.toRadians(77.59))
                                        .splineTo(new Vector2d(48.71, 53.13), Math.toRadians(92.49))
                                        .splineTo(new Vector2d(61.39, 60.06), Math.toRadians(13.09))
                                        .build()
                       /* // place pixel 2
                       drive.trajectorySequenceBuilder(new Pose2d(12.45, 62.86, Math.toRadians(-90.00)))
                        .splineTo(new Vector2d(5.53, 32.20), Math.toRadians(257.27))
                        .splineTo(new Vector2d(9.06, 26.16), Math.toRadians(-23.70))
                        .splineTo(new Vector2d(51.22, 30.73), Math.toRadians(4.56))
                        .splineTo(new Vector2d(49.74, 33.09), Math.toRadians(122.01))
                        .splineTo(new Vector2d(46.35, 42.37), Math.toRadians(110.06))
                        .splineTo(new Vector2d(51.07, 56.97), Math.toRadians(72.09))
                         .splineTo(new Vector2d(63.30, 59.03), Math.toRadians(-8.97))
                        .build()
                        */

/*                        // place pixel 3
                        drive.trajectorySequenceBuilder(new Pose2d(8.03, 65.07, Math.toRadians(-90.00)))
                        .splineTo(new Vector2d(22.62, 36.63), Math.toRadians(-88.60))
                        .splineTo(new Vector2d(50.78, 35.74), Math.toRadians(0.62))
                        .splineTo(new Vector2d(50.63, 39.57), Math.toRadians(92.20))
                        .splineTo(new Vector2d(48.27, 53.43), Math.toRadians(104.22))
                        .splineTo(new Vector2d(62.57, 59.32), Math.toRadians(19.93))
                         .build()*/

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