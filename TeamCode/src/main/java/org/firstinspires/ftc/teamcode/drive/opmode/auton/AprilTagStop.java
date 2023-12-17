package org.firstinspires.ftc.teamcode.drive.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagStop extends LinearOpMode {
    public static boolean USE_WEBCAM = true;

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    public static int numberAprilTag = 0;

    public void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
//            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//            FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    private AprilTagDetection telemetryAprilTag(int number) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addData("ID " + detection.id, String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addData("Pose " + detection.id, String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addData("Pitch " + detection.id, String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addData("Range " + detection.id, String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addData("ID " + detection.id, String.format("\n==== (ID %d) Unknown", detection.id), "none");
                telemetry.addData("Center " + detection.id, String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y), "none");
            }
        }   // end for() loop

        telemetry.update();

        // Add "key" information to telemetry
        return currentDetections.isEmpty() ? null : currentDetections.get(number);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.update();

        initAprilTag();

        Pose2d start = new Pose2d(0,0, Math.toRadians(90.00));
        drive.setPoseEstimate(start);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(10, 0))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            AprilTagDetection tag = telemetryAprilTag(numberAprilTag);

            telemetry.update();

            visionPortal.resumeStreaming();
            sleep(20);

            drive.update();
            Pose2d currentPosition = drive.getPoseEstimate();

            if(tag != null && !drive.isBusy()) {
                TrajectorySequence tagPose = drive.trajectorySequenceBuilder(currentPosition)
                        .lineToConstantHeading(new Vector2d(tag.ftcPose.x, tag.ftcPose.y))
                        .build();

                drive.followTrajectorySequence(tagPose);
            }
        }
        visionPortal.close();
    }
}
