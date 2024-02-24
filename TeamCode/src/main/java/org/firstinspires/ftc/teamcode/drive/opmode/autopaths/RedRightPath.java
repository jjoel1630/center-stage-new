package org.firstinspires.ftc.teamcode.drive.opmode.autopaths;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.drive.opmode.teleop.TeleOpFiniteStates;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.CustomElementPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.TeamElementPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.drive.opmode.Constants.*;


import java.util.List;

@Autonomous(group = "autonomous")
@Config
public class RedRightPath extends LinearOpMode {
    public enum DriverState {
        PRELOADED,
        TAGS,
        WHITE,
        PARK,
        DONE,
    }

    // Camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static int zone = 1;
    public static int width = 2304, height = 1536;

    public static double aprilTagGap = -1*atGap+2;
    public static double aprilTagOffset = -1*atOff;

    // Linear Constants
    public static double pixelDropHeight = 1000;
    public static double p = 3, i = 0, d = 0, f = 0.09;
    public static double error = 15;

    //

    private static class LinearSlides extends Thread {
        Telemetry t;
        OuttakeSlides slides;

        public LinearSlides(Telemetry tele, OuttakeSlides slides) {
            this.t = tele;
            this.slides = slides;
        }

        @Override
        public void run() {
            t.addLine("in thread");
            t.update();
            try {
                slides.moveToPosition(100, 50);
            } catch(Exception err) {}

            this.interrupt();
        }
    }

    DriverState driverState = DriverState.PRELOADED;



    public static Pose2d start = new Pose2d(15.875, -65.50, Math.toRadians(90));

    public OuttakeSlides slides;
    public OuttakeArm arm;
    public IntakeClaw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

//        slides = new OuttakeSlides();
//        arm = new OuttakeArm();
//        claw = new IntakeClaw();

        Servo arm = hardwareMap.servo.get("arm");
        arm.setPosition(0.5);

        // Paths
        TrajectorySequence path1preloaded = drive.trajectorySequenceBuilder(start)
                .splineTo(new Vector2d(16.00, -32.00), Math.toRadians(90.00))
                .addDisplacementMarker(() -> {
//                    claw.singleOpen();
                    telemetry.addLine("drop purple pixel");
                    telemetry.update();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(41.50, -36.00, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {
                    telemetry.addLine("drop yellow pixel");
                    telemetry.update();
                })
//                .build();
//        TrajectorySequence path1white = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(22.00, -14.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(-35.00, -14.00, Math.toRadians(180.00)))
                .addSpatialMarker(new Vector2d(-55, -36), () -> {
                    telemetry.addLine("drop arm to pick white pixel");
                    telemetry.update();
                })
                .lineToLinearHeading(new Pose2d(-60.00, -36.00, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {
                    telemetry.addLine("pick white pixels");
                    telemetry.update();
                })
                .lineToLinearHeading(new Pose2d(-34.00, -14.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(15.00, -14.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(50.00, -38.00, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {
                    telemetry.addLine("drop white pixels");
                    telemetry.update();
                })
                .build();

        TrajectorySequence path1 = drive.trajectorySequenceBuilder(start)
                .splineTo(new Vector2d(8.00, -34.00), Math.toRadians(135.00))
                .lineToLinearHeading(new Pose2d(39, -32, Math.toRadians(180.00)))
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(12.00, -32.50))
                .lineToLinearHeading(new Pose2d(41.50, -36.00, Math.toRadians(180.00)))
                .build();
        TrajectorySequence path3 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(27.00, -42.00))
                .lineToLinearHeading(new Pose2d(41.50, -36.00, Math.toRadians(180.00)))
                .build();

        initAprilTag();
        initCamera();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            List<AprilTagDetection> tags = telemetryAprilTag();
            switch (driverState) {
                case PRELOADED:
//                    drive.followTrajectorySequence(path1preloaded);

                    driverState = DriverState.TAGS;
                    break;
                case TAGS:
                    if(tags != null) {
                        AprilTagDetection tag = tags.get(0);
                        for(AprilTagDetection t : tags) {
                            if(zone == 1 && t.id == 4) tag = t;
                            else if(zone == 2 && t.id == 5) tag = t;
                            else if(zone == 3 && t.id == 6) tag = t;
                        }
                        Pose2d current = drive.getPoseEstimate();
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.addData("tag", tag.id);
                        telemetry.addData("current heading", current.getHeading());
                        telemetry.addData("total", current.getHeading()+tag.ftcPose.yaw);
                        telemetry.update();
//                        Trajectory tagPose = drive.trajectoryBuilder(new Pose2d(current.getX(), current.getY(), Math.toRadians(Math.toDegrees(current.getHeading())+tag.ftcPose.yaw)))
//                                .lineToLinearHeading(new Pose2d(current.getX() + tag.ftcPose.y + aprilTagGap,
//                                        current.getY() - tag.ftcPose.x + aprilTagOffset, Math.toRadians(180)))
//                                .build();
//
//                        drive.followTrajectory(tagPose);
                    }
                    break;
                case WHITE:
                    break;
                case PARK:
                    Pose2d current = drive.getPoseEstimate();

                    TrajectorySequence park = drive.trajectorySequenceBuilder(current)
                            .lineToConstantHeading(new Vector2d(50.00, -13.00))
                            .lineToConstantHeading(new Vector2d(65.50, -13.00))
                            .build();

                    drive.followTrajectorySequenceAsync(park);

                    driverState = DriverState.DONE;

                    break;
                case DONE:
                    break;
            }
        }
    }

    public void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addLine("april id" + cameraMonitorViewId);
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
//                .setLiveViewContainerId(cameraMonitorViewId)
                .setCameraResolution(new Size(1280, 960))
                .build();
    }
    public List<AprilTagDetection> telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        return currentDetections.size() > 0 ? currentDetections : null;
    }
    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addLine("other id" + cameraMonitorViewId);
        telemetry.update();
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        TeamElementPipeline elementPipeTeam = new TeamElementPipeline();
        camera.setPipeline(elementPipeTeam);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
                while(!opModeIsActive() && !isStopRequested()) {
                    telemetry.addLine("streaming");
                    zone = elementPipeTeam.get_element_zone();
                    telemetry.addLine("Element Zone" + zone);
                    telemetry.update();
                }
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });
    }
}
