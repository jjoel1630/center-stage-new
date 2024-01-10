package org.firstinspires.ftc.teamcode.drive.opmode.autoncomp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;
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
public class RedLeft extends LinearOpMode {
    public enum DriverState {
        AUTOMATIC,
        TAGS,
        PARK,
        DONE
    }

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static int zone = 1;
    public static int width = 2304, height = 1536;
    public void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addLine("april id" + cameraMonitorViewId);
        telemetry.update();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        TeamElementPipeline elementPipeTeam = new TeamElementPipeline();
        camera.setPipeline(elementPipeTeam);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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

    private DcMotorEx linearSlide = null;
    private Servo claw, arm;
    private ElapsedTime timer;

    double clawPos = CLAW_MIN, armPos = ARM_MIN;
    public static double linearCurPos, pid;
    PIDControllerCustom linearController = new PIDControllerCustom(linearKp, linearKi, linearKd);

    DriverState driverState = DriverState.AUTOMATIC;

    public static double aprilTagGap = -1*atGap;
    public static double aprilTagOffset = -1*atOff;

    public Pose2d start = new Pose2d(-39.87, -65.50, Math.toRadians(90.00));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(start);

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");

//        claw.setPosition(clawPos);
//        arm.setPosition(armPos);

        timer = new ElapsedTime();

        // Paths
        TrajectorySequence path1 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(-47.00, -42.00))
                .lineToLinearHeading(new Pose2d(-48.00, -60.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(24.00, -60.00))
                .lineToLinearHeading(new Pose2d(41.50, -36.00, Math.toRadians(180.00)))
                .build();
        TrajectorySequence path2 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(-36.00, -30.50))
                .lineToLinearHeading(new Pose2d(-48.00, -60.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(24.00, -60.00))
                .lineToLinearHeading(new Pose2d(41.50, -36.00, Math.toRadians(180.00)))
                .build();
        TrajectorySequence path3 = drive.trajectorySequenceBuilder(new Pose2d(-39.87, -65.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-30.00, -38.00), Math.toRadians(45.00))
                .lineToLinearHeading(new Pose2d(-48.00, -60.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(24.00, -60.00))
                .lineToLinearHeading(new Pose2d(41.50, -36.00, Math.toRadians(180.00)))
                .build();

        initAprilTag();
//        initCamera();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            linearController.setPID(linearKp, linearKi, linearKd);

            List<AprilTagDetection> tags = telemetryAprilTag();
            switch (driverState) {
                case AUTOMATIC:
                    visionPortal.resumeStreaming();
                    sleep(20);

                    if(zone == 1) drive.followTrajectorySequence(path1);
                    else if(zone == 2) drive.followTrajectorySequence(path2);
                    else if(zone == 3) drive.followTrajectorySequence(path3);

                    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    while(Math.abs(linearHigh - linearCurPos) >= linearError) {
                        linearCurPos = linearSlide.getCurrentPosition();
                        pid = linearController.update(linearHigh, linearCurPos);
                        linearSlide.setVelocity(pid);
                    }

                    driverState = DriverState.TAGS;
                    break;
                case TAGS:
                    linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    linearSlide.setPower(linearF);

                    if(tags != null) {
                        AprilTagDetection tag = tags.get(0);
                        Pose2d current = drive.getPoseEstimate();
                        telemetry.addData("current heading", current.getHeading());
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.addData("total", current.getHeading()+tag.ftcPose.yaw);
                        telemetry.update();
                        Trajectory tagPose = drive.trajectoryBuilder(new Pose2d(current.getX(), current.getY(), Math.toRadians(Math.toDegrees(current.getHeading())+tag.ftcPose.yaw)))
                                .lineToLinearHeading(new Pose2d(current.getX() + tag.ftcPose.y + aprilTagGap,
                                        current.getY() - tag.ftcPose.x + aprilTagOffset, Math.toRadians(180)))
                                .build();

                        drive.followTrajectory(tagPose);

                        driverState = DriverState.PARK;
                    }
                    break;
                case PARK:
                    break;
                case DONE:
                    break;
            }
        }
    }
}
