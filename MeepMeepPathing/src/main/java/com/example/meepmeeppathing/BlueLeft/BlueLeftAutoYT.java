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
//                        drive.trajectorySequenceBuilder(new Pose2d(-36.37, -62.20, Math.toRadians(90.00)))
//                                .splineTo(new Vector2d(-35.93, -48.84), Math.toRadians(90.00))
//                                .lineToLinearHeading(new Pose2d(-43.94, -41.57, Math.toRadians(124.11)))
//                                .lineToLinearHeading(new Pose2d(-51.66, -53.59, Math.toRadians(0.00)))
//                                .splineToConstantHeading(new Vector2d(-16.63, -60.42), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(27.46, -56.26), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(50.77, -36.67), Math.toRadians(0.00))
//                                .build()
<<<<<<< HEAD
=======
<<<<<<< HEAD:MeepMeepPathing/src/main/java/com/example/meepmeeppathing/BlueLeftAuto.java
                        drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(204)))
                                .lineToLinearHeading(new Pose2d(13.7, 1.4, Math.toRadians(180)))
                                .build()
                        // place pixel 2
=======
>>>>>>> 9cc7fb20a6d8193db325cdb79e209640311f342a
                                drive.trajectorySequenceBuilder(new Pose2d(0.00, 0.00, Math.toRadians(0.00)))
                                        .splineToLinearHeading(new Pose2d(10, -25, Math.toRadians(-70)), Math.toRadians(-70))
                                        .splineToLinearHeading(new Pose2d(30.00, -30.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                                        .build()

        // place pixel 2
<<<<<<< HEAD
=======
>>>>>>> 43852f0cfd467e6a964fe9a4ae85efbcd3c4546a:MeepMeepPathing/src/main/java/com/example/meepmeeppathing/BlueLeft/BlueLeftAutoYT.java
>>>>>>> 9cc7fb20a6d8193db325cdb79e209640311f342a
//                        drive.trajectorySequenceBuilder(new Pose2d(-36.37, -62.20, Math.toRadians(90.00)))
//                                .splineTo(new Vector2d(-35.93, -48.84), Math.toRadians(90.00))
//                                .lineToLinearHeading(new Pose2d(-35.78, -32.66, Math.toRadians(90.00)))
//                                .lineToLinearHeading(new Pose2d(-51.66, -53.59, Math.toRadians(0.00)))
//                                .splineToConstantHeading(new Vector2d(-16.63, -60.42), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(27.46, -56.26), Math.toRadians(0.00))
//                                .splineToConstantHeading(new Vector2d(50.77, -36.67), Math.toRadians(0.00))
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
//                                .lineToLinearHeading(new Pose2d(48.21, 35.26, Math.toRadians(0.00)))//                                .build()

                        // place pixel 3
//                        drive.trajectorySequenceBuilder(new Pose2d(-33.56, 63.29, Math.toRadians(270.00)))
//                                .lineToLinearHeading(new Pose2d(-33.13, 38.02, Math.toRadians(270.00)))
//                                .lineToLinearHeading(new Pose2d(-24.85, 35.26, Math.toRadians(270.00)))
//                                .lineToSplineHeading(new Pose2d(-49.06, 25.70, Math.toRadians(0.00)))
//                                .lineToLinearHeading(new Pose2d(-32.71, 8.92, Math.toRadians(0.00)))
//                                .lineToLinearHeading(new Pose2d(15.08, 8.92, Math.toRadians(0.00)))
//                                .lineToLinearHeading(new Pose2d(49.06, 28.04, Math.toRadians(0.00)))
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