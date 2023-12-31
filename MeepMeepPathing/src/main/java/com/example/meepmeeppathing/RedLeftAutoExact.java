//package com.example.meepmeeppathing;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//    public class RedLeftAutoExact {
//        public static void main(String[] args) {
//            // Declare a MeepMeep instance
//            // With a field size of 800 pixels
//            MeepMeep meepMeep = new MeepMeep(800);
//
//            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                    // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
//                    // Option: Set theme. Default = ColorSchemeRedDark()
//                    .followTrajectorySequence(drive ->
//
//                            // place pixel 1
//                               drive.trajectorySequenceBuilder(new Pose2d(-36.74, -60.32, Math.toRadians(90.00)))
//                                     .splineTo(new Vector2d(-47.36, -34.62), Math.toRadians(112.45))
//                                     .build
//
//                             //place pixel 2
//                               drive.trajectorySequenceBuilder(new Pose2d(-36.32, -60.96, Math.toRadians(90.00)))
//                                    .splineTo(new Vector2d(-40.14, -24.85), Math.toRadians(96.04))
//                                    .build()
//
//                            //place pixel 3
//                               drive.trajectorySequenceBuilder(new Pose2d(-37.59, -59.68, Math.toRadians(90.00)))
//                                    .splineTo(new Vector2d(-24.85, -34.19), Math.toRadians(63.43))
//                                    .build()
//
//
//                    );
//
//
//// Set field image
//            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
//                    .setDarkMode(true)
//// Background opacity from 0-1
//                    .setBackgroundAlpha(0.95f)
//                    .addEntity(myBot)
//                    .start();
//        }
//    }
//
