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

    private DcMotorEx intakeMotor1 = null; //setting intake motor variable
    private DcMotorEx intakeMotor2 = null; //setting 2nd intake motor variable
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
    public static double axialLSCoefficient = 1;

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
        //reverse ls motors
        leftLS.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLS.setDirection(DcMotorSimple.Direction.REVERSE);

        // intake
        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake2");
        // reverse intake
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

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

            /* ------- INTAKE ------- */
            double power = gamepad2.right_stick_y;
            intakeMotor1.setPower(power);
            intakeMotor2.setPower(power);

            /*--------OUTTAKE---------*/
            double servoOuttake = 1.0;

            // if right bumper pressed first servo releases
            if(gamepad2.left_bumper) servoOuttake = SERVO_MIN_RIGHT;
            if(gamepad2.right_trigger == 1) servoOuttake = SERVO_MAX_LEFT;

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