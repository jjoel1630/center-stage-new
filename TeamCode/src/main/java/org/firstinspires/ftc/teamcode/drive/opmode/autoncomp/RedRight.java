package org.firstinspires.ftc.teamcode.drive.opmode.autoncomp;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(group = "autonomous")
@Config
public class RedRight extends LinearOpMode {
    public enum DriverState {
        AUTOMATIC,
        TAGS,
        LIFT_PICKUP,
        LIFT_RAISE,
        LIFT_DROP,
        LIFT_RETRACT,
        DONE
    }

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public void initAprilTag() {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }
    public List<AprilTagDetection> telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        return currentDetections.size() > 0 ? currentDetections : null;
    }

    private DcMotorEx linearSlide = null;
    private Servo claw, arm;
    private ElapsedTime timer;
    public static double CLAW_MAX = 1, CLAW_MIN = 0.12;
    public static double ARM_GROUND = 0.2, ARM_MAX = 0.50, ARM_MIN = 0.0;
    public static double clawTime = 0.5, armTime = 0.7;
    double clawPos = CLAW_MIN, armPos = ARM_MIN;

    public static double slideCoeff = 1;
    public static double linearF = 0.05, linearFThreshold = 500;
    public static double armPreventionThreshold = 500, slidePositionMax = 3000;
    public static double linearLow = 0, linearHigh = 2500, linearError = 50;
    public static double linearKp = 0.011, linearKi = 0, linearKd = 0.00018;
    double linearCurPos, pid;
    PIDControllerCustom linearController = new PIDControllerCustom(linearKp, linearKi, linearKd);
    DriverState driverState = DriverState.AUTOMATIC;

    public static double aprilTagGap = -4;
    public static double aprilTagOffset = -5;


    public static Pose2d start = new Pose2d(15.875, -65.50, Math.toRadians(90));

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

        claw.setPosition(clawPos);
        arm.setPosition(armPos);

        initAprilTag();

        timer = new ElapsedTime();

        // Paths
        TrajectorySequence path1 = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(23.00, -42.50))
                .lineToLinearHeading(new Pose2d(48.00, -36.00, Math.toRadians(180)))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            linearController.setPID(linearKp, linearKi, linearKd);

            List<AprilTagDetection> tags = telemetryAprilTag();
            switch (driverState) {
                case AUTOMATIC:
                    visionPortal.resumeStreaming();
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

                        driverState = DriverState.LIFT_PICKUP;
                    }
                    break;
                case LIFT_PICKUP:
                    clawPos = CLAW_MIN;
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
                    if(timer.seconds() >= armTime && clawPos != CLAW_MAX) {
                        clawPos = CLAW_MAX;
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
        }
    }
}
