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
public class TeleopCompetition extends LinearOpMode {
    //Defining the 4 drivetrain motors as null
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx rearRight = null;

    private DcMotorEx intakeMotor = null; //setting intake motor variable
    private Servo rightServo, leftServo;

    private Servo airplane;

    private DcMotorEx leftLS = null;
    private DcMotorEx rightLS = null;

    // outtake constants
    public static double SERVO_MAX_RIGHT = 1.0;
    public static double SERVO_MIN_RIGHT = 0.0;
    public static double SERVO_MAX_LEFT = 1.0;
    public static double SERVO_MIN_LEFT = 0.0;

    // airplane constants
    public static double AIRPLANE_MAX = 1.0;
    public static double AIRPLANE_MIN = 0.0;

    // linearslide constants
    // black black, red red for both
    public static int leftDir = 1;
    public static int rightDir = 1;
    public static double axialLSCoefficient = 0.05;

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
        leftLS = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");
        rightLS = hardwareMap.get(DcMotorEx.class, "linearSlideRight");

        // intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        // reverse intake
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightServo = hardwareMap.servo.get("rightServo");
        leftServo = hardwareMap.servo.get("leftServo");

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

            double powerModifier = slowModeOn ? 0.3 : 1.0;

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

            /* ------- INTAKE ------- */
            double power = gamepad2.right_stick_y;
            intakeMotor.setPower(power);

            /*--------OUTTAKE---------*/
            double servoRight = 1.0;
            double servoLeft = 1.0;

            // if right bumper pressed first servo releases
            if(gamepad2.left_bumper) servoRight = SERVO_MIN_RIGHT;
            if(gamepad2.right_bumper) servoRight = SERVO_MAX_RIGHT;
            if(gamepad2.left_trigger == 1) servoLeft = SERVO_MIN_LEFT;
            if(gamepad2.right_trigger == 1) servoLeft = SERVO_MAX_LEFT;

//            rightServo.setPosition(servoRight);
//            leftServo.setPosition(servoLeft);

            // linear slide
            double axialLS = -gamepad2.left_stick_y;  // forward, back
            axialLS = axialLS * axialLSCoefficient;

            rightLS.setPower(axialLS * rightDir);
            leftLS.setPower(-axialLS * leftDir);

            /*-----------AIRPLANE LAUNCHER-----------*/
//            double airplanePos = AIRPLANE_MAX;
//
//            if(gamepad2.b) airplanePos = AIRPLANE_MAX;
//            if(gamepad2.a) airplanePos = AIRPLANE_MIN;
//
//            airplane.setPosition(airplanePos);
        }
    }
}