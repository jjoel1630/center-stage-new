package org.firstinspires.ftc.teamcode.drive.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;

@Config
@Autonomous
public class SlidePIDInternal extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();

    public static double p = 0.08, i = 0, d = 0.0008, f = 0;

    public double lastError = 0;
    public double integralSum = 0;

    public static double target = 500;
    public static double powerMultiplier = 1;
    public static double thresholdPower = -1.0;

    public DcMotorEx slide;

    public double calcPower(double ref, double state) {
        double err = ref - state;
        integralSum += err * timer.seconds();
        double derivative = (err - lastError) / timer.seconds();
        lastError = err;

        timer.reset();

        double output = (err * p) + (derivative * d) + (integralSum * i) + (ref * f);
        return output;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double mx = 0;

        while(opModeIsActive()) {
            int curPos = slide.getCurrentPosition();
            double pid = calcPower(target, curPos);
            if(thresholdPower != -1) {
                pid = Math.abs(pid) <= thresholdPower ? 0 : pid;
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
