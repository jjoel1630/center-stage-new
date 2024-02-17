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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(group="teleop")
public class DebugVoltage extends LinearOpMode {
    // Creating motors + servos
    private DcMotorEx frontLeft = null, frontRight = null, rearLeft = null, rearRight = null;
    private DcMotorEx linearSlide = null;
    private Servo airplane, claw, arm;
    private ElapsedTime timer;

    // drivetrain constants
    public static double axialCoefficient = 1, yawCoefficient = 1, lateralCoefficient = 1.1;
    public static double slowModePower = 0.5, regularPower = 0.7;

    // outtake constants
    public static double CLAW_MAX = 1, CLAW_MIN = 0.75;
    public static double ARM_GROUND = 0.29, ARM_MAX = 0.63, ARM_MIN = 0.0;
    public static double clawTime = 0.7, armTime = 0.9;
    double clawPos = CLAW_MIN, armPos = ARM_MIN;

    // airplane constants
    public static double AIRPLANE_MAX = 0.3, AIRPLANE_MIN = 0.0;
    double airplanePos = AIRPLANE_MIN;

    // linearslide constants: black black, red red for both
    public static double slideCoeff = 1;
    public static double linearF = 0.09, linearFThreshold = 1500;
    public static double armPreventionThreshold = 500, slidePositionMax = 2200;
    public static double linearLow = 0, linearHigh = 2400, linearError = 50;
    public static double linearKp = 1.5, linearKi = 0, linearKd = 0.1; // 4.8, 0.5

    @Override
    public void runOpMode() throws InterruptedException {
        /* ----------- HW MAP ----------- */
        // DT
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        rearLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // reverse dt motor
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // linearSlide
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");
        airplane = hardwareMap.servo.get("airplane");

        timer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y * axialCoefficient;  // forward, back
            double lateral = gamepad1.left_stick_x * lateralCoefficient; // side to side
            double yaw = gamepad1.right_stick_x * yawCoefficient; // turning

            double dtDenominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
//            double dtDenominator = 1;

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


            // if right bumper pressed first servo releases
//            if(gamepad2.left_bumper) clawPos = CLAW_MAX;
//            if(gamepad2.right_bumper) clawPos = CLAW_MIN;
//            if(gamepad2.left_trigger == 1 && linearSlide.getCurrentPosition() >= armPreventionThreshold) armPos = ARM_MAX;
//            if(gamepad2.right_trigger == 1) armPos = ARM_MIN;
//            if(gamepad2.dpad_up) armPos = ARM_GROUND;
//
//            claw.setPosition(clawPos);
//            arm.setPosition(armPos);

            // linear slide
//            double axialLS = -gamepad2.left_stick_y;  // forward, back
//
//            if(Math.abs(linearSlide.getCurrentPosition()) >= Math.abs(slidePositionMax) && !gamepad2.dpad_down) axialLS = 0;
//            else if(linearSlide.getCurrentPosition() >= armPreventionThreshold && armPos == ARM_MAX) axialLS = 0;
//            else axialLS = axialLS;
//
//            if(linearSlide.getCurrentPosition() >= linearFThreshold) axialLS += linearF;
//
//            linearSlide.setPower(axialLS * slideCoeff);
//
//            if(gamepad1.y) {
//                airplanePos = AIRPLANE_MAX;
//                airplane.setPosition(airplanePos);
//            }
//            if(gamepad1.x) {
//                airplanePos = AIRPLANE_MIN;
//                airplane.setPosition(airplanePos);
//            }

            telemetry.addData("voltage current", voltageSensor.getVoltage());
            telemetry.addData("lf power", frontLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lr power", rearLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rf power", frontRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rr power", rearRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }
    }
}