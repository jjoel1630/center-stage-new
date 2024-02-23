package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;
import org.firstinspires.ftc.teamcode.drive.opmode.teleop.TeleOPReg;

public class OuttakeSlides {
    // raise
    // lower
    // reset
    // hold position

    public int currentPosition;
    public double power, velo;
    public boolean reversed = false;

    public double p, i, d, f;
    public PIDControllerCustom pidController;

    public DcMotorEx slide;

    LinearOpMode curOpMode;

    ElapsedTime timer = new ElapsedTime();

    public OuttakeSlides(int startPosition, HardwareMap hardwareMap, LinearOpMode op, boolean rev, double p, double i, double d, double f) {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(rev) slide.setDirection(DcMotorSimple.Direction.REVERSE);

        this.currentPosition = startPosition;
        this.power = 0;
        this.velo = 0;

        this.p = p; this.i = i; this.d = d;
        this.f = f;
        pidController = new PIDControllerCustom(p, i, d);

        this.curOpMode = op;
        curOpMode.telemetry = new MultipleTelemetry(curOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void moveToPosition(int height, int error) {
        this.setRunMode("with");
        this.setCurrentPosition();
        this.resetTimer();
        this.startTimer();

        while(Math.abs(height - this.currentPosition) <= error) {
            double pid = pidController.update(height, this.currentPosition);
            this.velo = pid;
            this.powerSlideVel(pid);
        }

        this.resetTimer();
    }

    public void setRunMode(String mode) {
        if(mode.equals("without")) this.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        else if(mode.equals("with")) this.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getCurrentPower() {
        return this.power;
    }

    public double getCurrentVelo() {
        return this.velo;
    }

    public void powerSlideRaw(double pwr) {
        this.slide.setPower(pwr);
    }

    public void powerSlideVel(double vel) {
        this.slide.setVelocity(vel);
    }

    public void startTimer() {
        this.timer.startTime();
    }

    public void resetTimer() {
        this.timer.reset();
    }

    public double getTime() {
        return this.timer.seconds();
    }

    public int setCurrentPosition() {
        this.currentPosition = slide.getCurrentPosition();
        return this.currentPosition;
    }

    public void LOG_STATS() {
        String stats = "Current Power: " + this.power + "\nCurrent Velo: " + this.velo + "\nCurrent Position: " + this.currentPosition + "\nCurrent Time: " + this.timer.seconds();
        curOpMode.telemetry.addData("OuttakeSlides", stats);
    }
}
