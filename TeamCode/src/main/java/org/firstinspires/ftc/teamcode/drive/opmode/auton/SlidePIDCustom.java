package org.firstinspires.ftc.teamcode.drive.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;

@Config
@Autonomous
public class SlidePIDCustom extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();

    public static double p = 0.1, i = 0, d = 0.001, f = 0.3; // long belt
    public PIDControllerCustom pidController = new PIDControllerCustom(p, i, d);

    public static double target = 500;
    public static double divisor = 1;
    public static double powerMultiplier = 1; // short belt = 1;
    public static boolean reversed = true;

    public DcMotorEx slide;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(reversed) slide.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double mx = 0;

        while(opModeIsActive()) {
            pidController.setPID(p, i, d);

            double curPos = slide.getCurrentPosition();
            double pid = pidController.update(target / divisor, curPos / divisor);

            if(f >= 0) {
                pid = pid + f;
            }

            slide.setPower(powerMultiplier * pid);

            mx = Math.max(mx, Math.abs(powerMultiplier * pid));

            telemetry.addData("curPos", curPos);
            telemetry.addData("power", pid * powerMultiplier);
            telemetry.addData("pid", pid);
            telemetry.addData("targetPos", target);
            telemetry.addData("max power", mx);
            telemetry.update();
        }
    }
}
