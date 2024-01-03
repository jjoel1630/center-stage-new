package org.firstinspires.ftc.teamcode.drive.opmode.auton;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous
public class MarkersTesting extends LinearOpMode {
    public enum DriverState {
        AUTOMATIC,
        TAGS,
        LIFT_PICKUP,
        LIFT_RAISE,
        LIFT_DROP,
        LIFT_RETRACT,
        DONE
    }
    DriverState driverState = DriverState.AUTOMATIC;

    public static boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public void initAprilTag() {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        else visionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, aprilTag);
    }
    public List<AprilTagDetection> telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        return currentDetections.size() > 0 ? currentDetections : null;
    }

    // Variables
    public static double spline1x = 10, spline1y = -25, spline1heading = -90, spline1splineheading = -90;
    public static double spline2x = 30, spline2y = -30, spline2heading = 180, spline2splineheading = 90;
    public static double line1x = 40, line1y = -30;
    public static double aprilTagGap = -6;
    public static double aprilTagOffset = -1;

    // Creating motors + servos
    private DcMotorEx linearSlide = null;
    private Servo claw, arm;
    private ElapsedTime timer;

    // outtake constants
    public static double CLAW_MAX = 0.65, CLAW_MIN = 0.5;
    public static double ARM_MAX = 0.58, ARM_MIN = 0.0;
    public static double clawTime = 0.5, armTime = 0.7;
    double clawPos = CLAW_MIN, armPos = ARM_MIN;

    // linearslide constants: black black, red red for both
    public static double slideCoeff = 1;
    public static double linearF = 0.05;
    public static double linearLow = 0, linearHigh = 2500, linearError = 50;
    public static double linearKp = 0.011, linearKi = 0, linearKd = 0.00018;
    PIDControllerCustom linearController = new PIDControllerCustom(linearKp, linearKi, linearKd);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(0,0, Math.toRadians(0));
        drive.setPoseEstimate(start);

        TrajectorySequence path1 = drive.trajectorySequenceBuilder(start)
                .splineToLinearHeading(new Pose2d(spline1x, spline1y, Math.toRadians(spline1heading)), Math.toRadians(spline1splineheading))
                .splineToLinearHeading(new Pose2d(spline2x, spline2y, Math.toRadians(spline2heading)), Math.toRadians(spline2splineheading))
                .lineTo(new Vector2d(line1x, line1y))
                .build();

        // linearSlide
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");

        initAprilTag();

        timer = new ElapsedTime();

        waitForStart();

        while(opModeIsActive()) {
            List<AprilTagDetection> tags = telemetryAprilTag();
            double linearCurPos, pid;
            switch (driverState) {
                case AUTOMATIC:
                    visionPortal.resumeStreaming();
                    drive.update();
                    sleep(20);

                    drive.followTrajectorySequence(path1);

                    driverState = DriverState.TAGS;
                    break;
                case TAGS:
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

                        driverState = DriverState.DONE;
                    }
                    break;
                case LIFT_PICKUP:
                    clawPos = CLAW_MAX;
                    claw.setPosition(clawPos);
                    if(timer.seconds() >= clawTime && armPos != ARM_MIN) {
                        armPos = ARM_MIN;
                        arm.setPosition(armPos);
                        timer.reset();
                    }

                    if(timer.seconds() >= armTime) driverState = DriverState.LIFT_RAISE;
                    break;
                case LIFT_RAISE:
                    linearCurPos = linearSlide.getCurrentPosition();
                    pid = linearController.update(linearHigh, linearCurPos);
                    linearSlide.setPower(pid * slideCoeff);

                    if(Math.abs(linearHigh - linearCurPos) <= linearError) {
                        driverState = DriverState.LIFT_DROP;
                        timer.reset();
                    }
                    break;
                case LIFT_DROP:
                    linearSlide.setPower(linearF);

                    armPos = ARM_MAX;
                    arm.setPosition(armPos);
                    if(timer.seconds() >= armTime && clawPos != CLAW_MIN) {
                        clawPos = CLAW_MIN;
                        claw.setPosition(clawPos);
                    }

                    if(timer.seconds() >= clawTime + armTime) driverState = DriverState.LIFT_RETRACT;
                    break;
                case LIFT_RETRACT:
                    clawPos = CLAW_MAX;
                    claw.setPosition(clawPos);
                    if(timer.seconds() >= clawTime && armPos != ARM_MIN) {
                        armPos = ARM_MIN;
                        arm.setPosition(armPos);
                        timer.reset();
                    }

                    linearCurPos = linearSlide.getCurrentPosition();
                    if(timer.seconds() >= armTime) {
                        pid = linearController.update(linearLow, linearCurPos);
                        linearSlide.setPower(pid * slideCoeff);
                    }

                    if(Math.abs(linearLow - linearCurPos) <= linearError) {
                        driverState = DriverState.DONE;
                        timer.reset();
                    }

                    break;
                case DONE:
                    break;
            }

//            if(tags != null) {
//                for (AprilTagDetection detection : tags) {
//                    if (detection.metadata != null) {
//                        telemetry.addData("ID " + detection.id, String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                        telemetry.addData("Pose " + detection.id, String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                    } else {
//                        telemetry.addData("ID " + detection.id, String.format("\n==== (ID %d) Unknown", detection.id), "none");
//                        telemetry.addData("Center " + detection.id, String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y), "none");
//                    }
//                }
//            }
//            telemetry.update();
        }
    }
}
