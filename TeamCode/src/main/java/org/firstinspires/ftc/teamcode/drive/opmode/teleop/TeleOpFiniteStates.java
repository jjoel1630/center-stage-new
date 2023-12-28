package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp
public class TeleOpFiniteStates extends LinearOpMode {
    public enum OuttakeState {
        LIFT_PICKUP,
        LIFT_RAISE,
        LIFT_DROP,
        LIFT_RETRACT,
        LIFT_MANUAL
    };

    public enum DriverState {
        AUTOMATIC,
        DRIVER
    }

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


    // Creating motors + servos
    private DcMotorEx frontLeft = null, frontRight = null, rearLeft = null, rearRight = null;
    private DcMotorEx linearSlide = null;
    private Servo airplane, claw, arm;
    private ElapsedTime timer;

    // drivetrain constants
    public static double axialCoefficient = 1, yawCoefficient = 1, lateralCoefficient = 1.1;
    public static double slowModePower = 0.3, regularPower = 0.8;

    // outtake constants
    public static double CLAW_MAX = 0.65, CLAW_MIN = 0.5;
    public static double ARM_GROUND = 0.2, ARM_MAX = 0.58, ARM_MIN = 0.0;
    public static double clawTime = 0.5, armTime = 0.7;
    double clawPos = CLAW_MIN, armPos = ARM_MIN;

    // airplane constants
    public static double AIRPLANE_MAX = 1.0, AIRPLANE_MIN = 0.0;

    // linearslide constants: black black, red red for both
    public static double slideCoeff = 1;
    public static double linearF = 0.05, linearFThreshold = 500;
    public static double armPreventionThreshold = 500, slidePositionMax = 3000;
    public static double linearLow = 0, linearHigh = 2500, linearError = 50;
    public static double linearKp = 0.011, linearKi = 0, linearKd = 0.00018;
    PIDControllerCustom linearController = new PIDControllerCustom(linearKp, linearKi, linearKd);
    OuttakeState outState = OuttakeState.LIFT_MANUAL;

    DriverState driverState = DriverState.DRIVER;
    public static double aprilTagGap = 6;
    public static double aprilTagOffset = 2;
    public static double currentHeading = 270;

    @Override
    public void runOpMode() throws InterruptedException {
        /* ----------- HW MAP ----------- */
        // DT
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        rearLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        // reverse dt motor
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(currentHeading)));

        // linearSlide
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");
        airplane = hardwareMap.servo.get("airplane");

        initAprilTag();

        timer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            /* ------- DRIVETRAIN ------- */
            drive.update();
            currentHeading = Math.toDegrees(drive.getPoseEstimate().getHeading());
            List<AprilTagDetection> tags = telemetryAprilTag();
            switch(driverState) {
                case DRIVER:
                    // gamepad input
                    double axial = -gamepad1.left_stick_y * axialCoefficient;  // forward, back
                    double lateral = gamepad1.right_stick_x * lateralCoefficient; // side to side
                    double yaw = gamepad1.left_stick_x * yawCoefficient; // turning

                    double dtDenominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

                    // slowmode
                    boolean slowModeOn = false;
                    if (gamepad1.left_bumper) slowModeOn = true;

                    double powerModifier = slowModeOn ? slowModePower : regularPower;

                    //making the equations in order to have the directions synchronous
                    double leftFrontPower = ((axial + lateral + yaw) / dtDenominator) * powerModifier;
                    double rightFrontPower = ((axial - lateral - yaw) / dtDenominator) * powerModifier;
                    double leftBackPower = ((axial - lateral + yaw) / dtDenominator) * powerModifier;
                    double rightBackPower = ((axial + lateral - yaw) / dtDenominator) * powerModifier;

                    // set powers
                    frontLeft.setPower(leftFrontPower);
                    frontRight.setPower(rightFrontPower);
                    rearLeft.setPower(leftBackPower);
                    rearRight.setPower(rightBackPower);

                    visionPortal.resumeStreaming();
                    sleep(20);
                    if(tags != null && gamepad1.dpad_up) {
                        AprilTagDetection tag = tags.get(0);
                        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(270-tag.ftcPose.yaw)));
                        Trajectory tagPose = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(270-tag.ftcPose.yaw)))
                                .lineToLinearHeading(new Pose2d(tag.ftcPose.x+aprilTagOffset,tag.ftcPose.y-aprilTagGap, 270))
                                .build();

                        drive.followTrajectoryAsync(tagPose);

                        driverState = DriverState.AUTOMATIC;
                    }
                    break;
                case AUTOMATIC:
                    if (!drive.isBusy()) {
                        driverState = DriverState.DRIVER;
                    }
                    break;
            }

            /*--------OUTTAKE---------*/
            // hot buttons
            // pick up pixel + raise
            // pick drop pixel + reduce
            linearController.setPID(linearKp, linearKi, linearKd);

            if(gamepad2.x) outState = OuttakeState.LIFT_MANUAL;
            if(gamepad2.b) {
                timer.reset();
                outState = OuttakeState.LIFT_PICKUP;
            }

            double linearCurPos, pid;
            switch (outState) {
                case LIFT_PICKUP:
                    clawPos = CLAW_MAX;
                    claw.setPosition(clawPos);
                    if(timer.seconds() >= clawTime && armPos != ARM_MIN) {
                        armPos = ARM_MIN;
                        arm.setPosition(armPos);
                        timer.reset();
                    }

                    if(timer.seconds() >= armTime) outState = OuttakeState.LIFT_RAISE;
                    break;
                case LIFT_RAISE:
                    linearCurPos = linearSlide.getCurrentPosition();
                    pid = linearController.update(linearHigh, linearCurPos);
                    linearSlide.setPower(pid * slideCoeff);

                    if(Math.abs(linearHigh - linearCurPos) <= linearError) {
                        outState = OuttakeState.LIFT_DROP;
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

                    if(timer.seconds() >= clawTime + armTime) outState = OuttakeState.LIFT_RETRACT;
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
                        outState = OuttakeState.LIFT_MANUAL;
                        timer.reset();
                    }

                    break;
                case LIFT_MANUAL:
                    // if right bumper pressed first servo releases
                    if(gamepad2.left_bumper) clawPos = CLAW_MAX;
                    if(gamepad2.right_bumper) clawPos = CLAW_MIN;
                    if(gamepad2.left_trigger == 1) armPos = ARM_MAX;
                    if(gamepad2.right_trigger == 1 && linearSlide.getCurrentPosition() >= armPreventionThreshold) armPos = ARM_MIN;
                    if(gamepad2.dpad_up) armPos = ARM_GROUND;

                    claw.setPosition(clawPos);
                    arm.setPosition(armPos);

                    // linear slide
                    double axialLS = -gamepad2.left_stick_y;  // forward, back

                    if(Math.abs(linearSlide.getCurrentPosition()) >= Math.abs(slidePositionMax) && !gamepad2.a) axialLS = 0;
                    else if(linearSlide.getCurrentPosition() >= armPreventionThreshold && armPos == ARM_MAX) axialLS = 0;
                    else axialLS = axialLS;

                    if(linearSlide.getCurrentPosition() >= linearFThreshold) axialLS += linearF;

                    linearSlide.setPower(axialLS * slideCoeff);

                    break;
                default:
                    // should never be reached, as liftState should never be null
                    outState = OuttakeState.LIFT_MANUAL;
            }

            /*-----------AIRPLANE LAUNCHER-----------*/
            double airplanePos = AIRPLANE_MAX;

            if(gamepad2.dpad_left) airplanePos = AIRPLANE_MAX;
            if(gamepad2.dpad_right) airplanePos = AIRPLANE_MIN;

            airplane.setPosition(airplanePos);

            if(tags != null) {
                for (AprilTagDetection detection : tags) {
                    if (detection.metadata != null) {
                        telemetry.addData("ID " + detection.id, String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addData("Pose " + detection.id, String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    } else {
                        telemetry.addData("ID " + detection.id, String.format("\n==== (ID %d) Unknown", detection.id), "none");
                        telemetry.addData("Center " + detection.id, String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y), "none");
                    }
                }
            }
            telemetry.addData("slide position", linearSlide.getCurrentPosition());
            telemetry.addData("lift state", outState);
            telemetry.addData("timer", timer.seconds());
            telemetry.addData("heading", currentHeading);
            telemetry.update();
        }
    }
}