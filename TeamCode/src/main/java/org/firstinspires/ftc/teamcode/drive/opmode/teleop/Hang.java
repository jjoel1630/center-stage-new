package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class Hang extends LinearOpMode {
    DcMotorEx left, right;
    public static boolean leftReversed = true, rightReversed = true;
    public static double leftPower = 1, rightPower = 1;
    public static double speed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotorEx.class, "hangLeft");
        right = hardwareMap.get(DcMotorEx.class, "handRight");

        if(leftReversed) left.setDirection(DcMotorSimple.Direction.REVERSE);
        if(rightReversed) right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double power = gamepad1.right_stick_y * speed; // turning
            left.setPower(power * leftPower);
            right.setPower(power * rightPower);
        }
    }
}
