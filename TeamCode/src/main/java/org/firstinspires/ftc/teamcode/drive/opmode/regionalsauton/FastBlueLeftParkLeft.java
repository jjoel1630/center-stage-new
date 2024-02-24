package org.firstinspires.ftc.teamcode.drive.opmode.regionalsauton;

import static org.firstinspires.ftc.teamcode.drive.opmode.Constants.atGap;
import static org.firstinspires.ftc.teamcode.drive.opmode.Constants.atOff;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.autoncomp.RedRight;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.BlueLeftPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.RedRightPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Config
@Autonomous(group="fast")
public class FastBlueLeftParkLeft extends LinearOpMode {
    public enum DriverState {
        AUTOMATIC,
        TAGS,
        PARK,
        DONE,
        LIFT_SCORE,
    }

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static int zone = 1;
    public static int width = 2304, height = 1536;
    public void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addLine("april id" + cameraMonitorViewId);
//        telemetry.update();
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

        BlueLeftPipeline elementPipeTeam = new BlueLeftPipeline();
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

    // Slide Subsystem
    public OuttakeSlides slides;
    public static double p = 3, i = 0, d = 0, f = 0.01;
    public static String slideName = "linearSlide";
    public static int armPreventionThreshold = 500, slidePositionMax = 2000, linearFThreshold = 1000, slidePositionScore = 1500;
    public static int linearLow = 0, linearError = 50;

    // Arm Subsystem
    public OuttakeArm arm;
    public static double high = 0, raised = 0.5, ground = 0.6, drop = 0.95;
    public static String armName = "arm";

    // Claw Subsystem
    public IntakeClaw claw;
    public static double openSingle = 0, closeSingle = 0.9, openStacked = 0.7, openOneStacked = 0.29, closeStacked = 0;
    public static String clawNameSingle = "singleClaw", clawNameStacked = "stackedClaw";
    public static double clawTime = 0.5, armTime = 0.5;

    public static Pose2d start = new Pose2d(15.875, 65.50, Math.toRadians(270));
    public String parkLocation = "right";
    public static double aprilTagGap = -7;
    public static double aprilTagOffset = -6;

    DriverState driverState = DriverState.AUTOMATIC;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        slides = new OuttakeSlides(0, this, true, p, i, d, f, slideName);
        arm = new OuttakeArm(ground, high, raised, ground, drop, this, armName);
        claw = new IntakeClaw(0, 0, openSingle, closeSingle, openStacked, openOneStacked, closeStacked, this, clawNameStacked, clawNameSingle);

        TrajectorySequence path1 = drive.trajectorySequenceBuilder(start)
                .splineTo(new Vector2d(12, 36), Math.toRadians(180.00))
                .addDisplacementMarker(() -> {
                    claw.singleOpen();
                })
                .waitSeconds(0.8)
                .addDisplacementMarker(() -> {
                    arm.raise();
                })
                .lineToLinearHeading(new Pose2d(38.00, 40, Math.toRadians(180.00)))
                .build();
        TrajectorySequence path2 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(12.00, 36))
                .addDisplacementMarker(() -> {
                    claw.singleOpen();
                })
                .waitSeconds(0.8)
                .addDisplacementMarker(() -> {
                    arm.raise();
                })
                .lineToConstantHeading(new Vector2d(12.00, 45.00))
                .lineToLinearHeading(new Pose2d(37, 39, Math.toRadians(180.00)))
                .build();
        TrajectorySequence path3 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(25.00, 42.00))
                .addDisplacementMarker(() -> {
                    claw.singleOpen();
                })
                .waitSeconds(0.8)
                .addDisplacementMarker(() -> {
                    arm.raise();
                })
                .lineToConstantHeading(new Vector2d(20, 52))
                .lineToLinearHeading(new Pose2d(38, 36, Math.toRadians(180.00)))
                .build();

        initAprilTag();
        initCamera();

        claw.closeBoth();
        waitSeconds(0.8);
        arm.moveArm(0);

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            List<AprilTagDetection> tags = telemetryAprilTag();
            switch (driverState) {
                case AUTOMATIC:
                    visionPortal.resumeStreaming();
                    sleep(20);

                    arm.moveArm(0.55);

                    if (zone == 1) drive.followTrajectorySequence(path3);
                    else if (zone == 2) drive.followTrajectorySequence(path2);
                    else if (zone == 3) drive.followTrajectorySequence(path1);

                    driverState = DriverState.TAGS;
                    break;
                case TAGS:
                    if (tags != null) {
                        AprilTagDetection tag = tags.get(0);
                        for (AprilTagDetection t : tags) {
                            if (zone == 1 && t.id == 3) tag = t;
                            else if (zone == 2 && t.id == 2) tag = t;
                            else if (zone == 3 && t.id == 1) tag = t;
                        }
                        Pose2d current = drive.getPoseEstimate();
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.addData("tag", tag.id);
                        telemetry.addData("current heading", current.getHeading());
                        telemetry.addData("total", current.getHeading() + tag.ftcPose.yaw);
                        telemetry.update();
                        Trajectory tagPose = drive.trajectoryBuilder(new Pose2d(current.getX(), current.getY(), Math.toRadians(Math.toDegrees(current.getHeading()) + tag.ftcPose.yaw)))
                                .lineToLinearHeading(new Pose2d(current.getX() + tag.ftcPose.y + aprilTagGap,
                                        current.getY() - tag.ftcPose.x + aprilTagOffset, Math.toRadians(180)))
                                .build();

                        drive.followTrajectory(tagPose);

                        driverState = DriverState.LIFT_SCORE;
                    }
                    break;
                case LIFT_SCORE:
                    arm.raise();
                    claw.closeBoth();
                    slides.moveToPosition(slidePositionScore, linearError);
                    slides.powerSlideRaw(f);
                    waitSeconds(0.8);
                    arm.drop();
                    waitSeconds(0.8);
                    claw.openBoth();
                    waitSeconds(0.5);
                    claw.closeBoth();
                    waitSeconds(0.5);
                    arm.raise();
                    waitSeconds(0.8);
                    slides.moveToPosition(0, linearError);

                    driverState = DriverState.DONE;

                    break;
                case PARK:
                    Pose2d current = drive.getPoseEstimate();

                    TrajectorySequence parkRight = drive.trajectorySequenceBuilder(current)
                            .lineToConstantHeading(new Vector2d(55, 12))
                            .build();
                    TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(current)
                            .lineToConstantHeading(new Vector2d(48, 62))
                            .build();

                    drive.followTrajectorySequenceAsync(parkLeft);

                    driverState = DriverState.DONE;
                    break;
                case DONE:
                    break;
            }
        }
    }
    public void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.seconds() <= seconds) {};

        timer.reset();
    }
}
