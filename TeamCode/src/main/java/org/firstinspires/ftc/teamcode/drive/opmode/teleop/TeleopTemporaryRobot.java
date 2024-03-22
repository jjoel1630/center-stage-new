package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class TeleopTemporaryRobot extends LinearOpMode {
    //Defining the 4 drivetrain motors as null
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx rearRight = null;

    private DcMotorEx linearSlide = null; //setting intake motor variable

    private Servo airplane;

    private Servo claw;
    private Servo arm;

    // outtake constants
    public static double CLAW_MAX = 1.0;
    public static double CLAW_MIN = 0.5;
    public static double ARM_GROUND = 0.2;
    public static double ARM_MAX = 0.58;
    public static double ARM_MIN = 0.0;
    double clawPos = CLAW_MIN;
    double armPos = ARM_MIN;

    // airplane constants
    public static double AIRPLANE_MAX = 1.0;
    public static double AIRPLANE_MIN = 0.0;

    // linearslide constants
    // black black, red red for both
    public static double slideDir = 1;
    public static double slideCoeff = 1;
    public static double linearF = 0.05;
    public static double linearFThreshold = 500;
    public static double armPreventionThreshold = 500;
    public static int slidePositionMax = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        /* ----------- HW MAP ----------- */
        // DT
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rightFront");
        rearRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        // reverse dt motor
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // linearSlide
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");
        airplane = hardwareMap.servo.get("airplane");

        waitForStart();

        while (opModeIsActive()) {
            /* ------- DRIVETRAIN ------- */
            // gamepad input
            double axial = -gamepad1.left_stick_y;  // forward, back
            double lateral = gamepad1.right_stick_x; // side to side
            double yaw = gamepad1.left_stick_x; // turning

            // coefficient modifiers
            double axialCoefficient = 1;
            double yawCoefficient = 1;
            double lateralCoefficient = 1;

            // changing power factor by coef
            yaw = yaw * yawCoefficient;
            axial = axial * axialCoefficient;
            lateral = lateral * lateralCoefficient;

            // slowmode
            boolean slowModeOn = false;
            if (gamepad1.left_bumper) slowModeOn = true;

            double powerModifier = slowModeOn ? 0.3 : 0.8;

            //making the equations in order to have the directions synchronous
            double leftFrontPower = (axial + lateral + yaw) * powerModifier;
            double rightFrontPower = (axial - lateral - yaw) * powerModifier;
            double leftBackPower = (axial - lateral + yaw) * powerModifier;
            double rightBackPower = (axial + lateral - yaw) * powerModifier;

            // set powers
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            rearLeft.setPower(leftBackPower);
            rearRight.setPower(rightBackPower);

            /*--------OUTTAKE---------*/
            // hot buttons
            // pick up pixel + raise
            // pick drop pixel + reduce

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

            if(Math.abs(linearSlide.getCurrentPosition()) >= Math.abs(slidePositionMax) && !gamepad2.a) {
                axialLS = 0;
            } else if(linearSlide.getCurrentPosition() >= armPreventionThreshold && armPos == ARM_MAX) {
                axialLS = 0;
            } else {
                axialLS = axialLS * slideCoeff;
            }

            if(linearSlide.getCurrentPosition() >= linearFThreshold) {
                axialLS += linearF;
            }

            linearSlide.setPower(axialLS * slideDir);

            /*-----------AIRPLANE LAUNCHER-----------*/
            double airplanePos = AIRPLANE_MAX;

            if(gamepad2.b) airplanePos = AIRPLANE_MAX;
            if(gamepad2.a) airplanePos = AIRPLANE_MIN;

            airplane.setPosition(airplanePos);

            telemetry.addData("slide position", linearSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}