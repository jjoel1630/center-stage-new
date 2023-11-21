package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagLocalization extends LinearOpMode {
    public static boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "BlizzardNewCam"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    private AprilTagDetection telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.update();

        return currentDetections.get(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initAprilTag();

        Pose2d start = new Pose2d(0,0, Math.toRadians(90.00));
        drive.setPoseEstimate(start);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(-36.22, -36.15))
                .build();

        waitForStart();

        while(opModeIsActive()) {
            drive.followTrajectorySequence(traj1);

            AprilTagDetection tag = telemetryAprilTag();

            telemetry.update();

            visionPortal.resumeStreaming();
            sleep(20);

            drive.update();
            Pose2d currentPosition = drive.getPoseEstimate();

            TrajectorySequence tagPose = drive.trajectorySequenceBuilder(currentPosition)
                    .lineToConstantHeading(new Vector2d(tag.ftcPose.x, tag.ftcPose.y))
                    .build();

            drive.followTrajectorySequence(tagPose);
        }
    }
}
