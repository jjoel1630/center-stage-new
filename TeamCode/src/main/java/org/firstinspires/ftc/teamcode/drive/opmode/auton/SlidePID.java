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
public class SlidePID extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();

    public static double pl = 0.08, il = 0, dl = 0.0008, fl = 0;
//    public static double pr = 0.000789, ir = 0, dr = 0.00023, fr = 0;
    public PIDControllerCustom pidController = new PIDControllerCustom(pl,il,dl);

    public double lastError = 0;
    public double integralSum = 0;

    public static double target = 500;
    public static double powerMultiplierLeft = 1;
    public static double powerMultiplierRight = 1;
    public static long timeout = 3000;
    public static double thresholdPower = 0.1;

    public DcMotorEx slideLeft, slideRight;

    public double calcPowerLeft(double ref, double state) {
        double err = ref - state;
        integralSum += err * timer.seconds();
        double derivative = (err - lastError) / timer.seconds();
        lastError = err;

        timer.reset();

        double output = (err * pl) + (derivative * dl) + (integralSum * il) + (ref * fl);
        return output;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        slideLeft = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight = hardwareMap.get(DcMotorEx.class, "linearSlideRight");
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double mx = 0;

        while(opModeIsActive()) {
            int curPosLeft = slideLeft.getCurrentPosition();
            double pidLeft = pidController.update(target, curPosLeft);
            pidLeft = Math.abs(pidLeft) <= thresholdPower ? 0 : pidLeft;
//            double pidLeft = calcPowerLeft(curPosLeft, target);
//            int curPosRight = slideRight.getCurrentPosition();
//            double pidRight = calcPowerRight(curPosRight, target);

            slideLeft.setPower(powerMultiplierLeft * pidLeft);

            mx = Math.max(mx, Math.abs(powerMultiplierLeft * pidLeft));

            telemetry.addData("curPosLeft", curPosLeft);
            telemetry.addData("power left", pidLeft);
            telemetry.addData("targetPos", target);
            telemetry.addData("max power", mx);
            telemetry.update();
        }
    }
}
