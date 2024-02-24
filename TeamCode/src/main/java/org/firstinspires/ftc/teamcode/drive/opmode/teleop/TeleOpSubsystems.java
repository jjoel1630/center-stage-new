package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.OuttakeSlides;

@TeleOp
@Config
public class TeleOpSubsystems extends LinearOpMode {
    public enum OuttakeState {
        LIFT_PICKUP,
        LIFT_RAISE,
        LIFT_DROP,
        LIFT_RETRACT,
        LIFT_MANUAL
    };

    // Drivetrain Motors & Config
    private DcMotorEx frontLeft = null, frontRight = null, rearLeft = null, rearRight = null;

    public static double axialCoefficient = 1, yawCoefficient = 1, lateralCoefficient = 1.1;
    public static double slowModePower = 0.5, regularPower = 0.85;

    // Slide Subsystem
    public OuttakeSlides slides;
    public static double p = 3, i = 0, d = 0, f = 0.09;
    public static String slideName = "linearSlide";
    public static int armPreventionThreshold = 500, slidePositionMax = 2000, linearFThreshold = 1000;
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

    // States
    OuttakeState outState = OuttakeState.LIFT_MANUAL;

    // Timer
    public ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize DT
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        rearLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearRight = hardwareMap.get(DcMotorEx.class, "rightRear");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Subsystem Initialize
        slides = new OuttakeSlides(0, this, true, p, i, d, f, slideName);
        arm = new OuttakeArm(0, high, raised, ground, drop, this, armName);
        claw = new IntakeClaw(0, 0, openSingle, closeSingle, openStacked, openOneStacked, closeStacked, this, clawNameStacked, clawNameSingle);

        // Voltage System
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        timer = new ElapsedTime();

        while(!isStopRequested() && opModeIsActive()) {
            /*  Drivetrain Commands -------- */
            // input
            double axial = -gamepad1.left_stick_y * axialCoefficient;  // forward, back
            double lateral = gamepad1.left_stick_x * lateralCoefficient; // side to side
            double yaw = gamepad1.right_stick_x * yawCoefficient; // turning

            double dtDenominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

            // slowmode
            boolean slowModeOn = false;
            if (gamepad1.left_bumper) slowModeOn = true;

            double powerModifier = slowModeOn ? slowModePower : regularPower;

            // making the equations in order to have the directions synchronous
            double leftFrontPower = ((axial + lateral + yaw) / dtDenominator) * powerModifier;
            double rightFrontPower = ((axial - lateral - yaw) / dtDenominator) * powerModifier;
            double leftBackPower = ((axial - lateral + yaw) / dtDenominator) * powerModifier;
            double rightBackPower = ((axial + lateral - yaw) / dtDenominator) * powerModifier;

            // set powers
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            rearLeft.setPower(leftBackPower);
            rearRight.setPower(rightBackPower);

            /* -------- Outtake Commands -------- */
            switch (outState) {
                case LIFT_RETRACT:
                    claw.closeBoth();
                    if(timer.seconds() >= clawTime) {
                        arm.raise();
                        timer.reset();
                    }

                    slides.setMode("with");
                    slides.setCurrentPosition();
                    double linearCurPos = slides.getCurrentPosition();

                    if(timer.seconds() >= armTime) {
                        slides.teleopPID(linearLow, linearError);
                    }

                    if(Math.abs(linearLow - linearCurPos) <= linearError) {
                        outState = OuttakeState.LIFT_MANUAL;
                        slides.setMode("without");
                        timer.reset();
                    }

                    break;
                case LIFT_MANUAL:
                    slides.setMode("without");
                    slides.setCurrentPosition();

                    // if right bumper pressed first servo releases
                    if(gamepad2.left_bumper) claw.singleOpen();
                    if(gamepad2.right_bumper) claw.stackedOpen();
                    if(gamepad2.y) claw.singleClose();
                    if(gamepad2.b) claw.stackedClose();
                    if(gamepad2.left_stick_button) claw.stackedOpenOne();
                    if(gamepad2.right_stick_button) claw.openBoth();

                    if(gamepad2.left_trigger == 1 && slides.getCurrentPosition() >= armPreventionThreshold) arm.raise();
                    if(gamepad2.right_trigger == 1) arm.drop();
                    if(gamepad2.dpad_up) arm.ground();


                    // linear slide
                    double axialLS = -gamepad2.left_stick_y;  // forward, back

                    if(Math.abs(slides.getCurrentPosition()) >= Math.abs(slidePositionMax) && slides.getCurrentPower() >= 0) axialLS = 0;
                    else if(Math.abs(slides.getCurrentPosition()) >= armPreventionThreshold && arm.getCurrentPosition() == arm.DROP) axialLS = 0;
                    else axialLS = axialLS;

                    if(slides.getCurrentPosition() >= linearFThreshold) axialLS += f;

                    slides.powerSlideRaw(axialLS);

                    slides.LOG_STATS();

                    break;
                default:
                    outState = OuttakeState.LIFT_MANUAL;
            }
        }
    }
}
