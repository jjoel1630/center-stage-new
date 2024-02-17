package org.firstinspires.ftc.teamcode.drive.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;

//@Disabled
@Config
@Autonomous
public class SlidePIDCustom extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();

    public static double p = 0, i = 0, d = 0, f = 0; // long belt
    public static double vel = 500, accel = 500;
    public PIDControllerCustom pidController = new PIDControllerCustom(p, i, d);

    public static double target = 1000;
    public double divisor = 1;
    public static double powerMultiplier = 1; // short belt = 1;
    public static boolean reversed = true;

    public static double armPos = 0.0;

    public DcMotorEx slide;
    public Servo arm;

    public double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt;
        if (elapsed_time > entire_dt) return distance;

        // if we're accelerating
        if (elapsed_time < acceleration_dt) return 0.5 * max_acceleration * Math.pow(elapsed_time, 2);
        // if we're cruising
        else if (elapsed_time < entire_dt) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;
            return acceleration_distance + max_velocity * cruise_current_dt;
        } else return distance;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(reversed) slide.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.servo.get("arm");
        arm.setPosition(armPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double mx = 0;
        timer.startTime();

        while(opModeIsActive()) {
            pidController.setPID(p, i, d);

            double curPos = slide.getCurrentPosition();
            double instantTargetPos = motion_profile(accel, vel, target, timer.seconds());

//            double pid = pidController.update(target / divisor, curPos / divisor) + f;
            double pid = (instantTargetPos - curPos) * p;

            slide.setVelocity(pid);

            mx = Math.max(mx, Math.abs(powerMultiplier * pid));
            telemetry.addData("curPos", curPos);
            telemetry.addData("targetPos", target);
            telemetry.addData("instTargetPos", instantTargetPos);
            telemetry.addData("pid", pid);
            telemetry.addData("max power", mx);
            telemetry.update();
        }
    }
}
