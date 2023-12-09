package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
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
    public static double CLAW_MIN = 0.0;
    public static double ARM_MAX = 1.0;
    public static double ARM_MIN = 0.0;

    // airplane constants
    public static double AIRPLANE_MAX = 1.0;
    public static double AIRPLANE_MIN = 0.0;

    // linearslide constants
    // black black, red red for both
    public static double slideDir = 1;
    public static double slideCoeff = 1;

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

        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");

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
            double clawPos = 1.0;
            double armPos = 1.0;

            // if right bumper pressed first servo releases
            if(gamepad2.left_bumper) clawPos = CLAW_MAX;
            if(gamepad2.right_bumper) clawPos = CLAW_MIN;
            if(gamepad2.left_trigger == 1) armPos = ARM_MAX;
            if(gamepad2.right_trigger == 1) armPos = ARM_MIN;

            claw.setPosition(clawPos);
            arm.setPosition(armPos);

            // linear slide
            double axialLS = -gamepad2.left_stick_y;  // forward, back
            axialLS = axialLS * slideCoeff;

            linearSlide.setPower(axialLS * slideDir);

            /*-----------AIRPLANE LAUNCHER-----------*/
            double airplanePos = AIRPLANE_MAX;

            if(gamepad2.b) airplanePos = AIRPLANE_MAX;
            if(gamepad2.a) airplanePos = AIRPLANE_MIN;

            airplane.setPosition(airplanePos);
        }
    }
}