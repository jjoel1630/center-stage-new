package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.auton.PIDControllerCustom;
import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util.SlideConstraints;

public class OuttakeSlides {
    private int currentPosition;
    private double power, velo;
    private boolean reversed = false;
    private String mode;

    private double p, i, d, f;
    private PIDControllerCustom pidController;

    private DcMotorEx slide;

    LinearOpMode curOpMode;

    ElapsedTime timer = new ElapsedTime();

    public OuttakeSlides(LinearOpMode op, String name, SlideConstraints constraints) {
        this.curOpMode = op;
        curOpMode.telemetry = new MultipleTelemetry(curOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        this.slide = this.curOpMode.hardwareMap.get(DcMotorEx.class, name);
        this.setMode("without");
        this.resetEncoder();
        if(constraints.isRev()) slide.setDirection(DcMotorSimple.Direction.REVERSE);

        this.currentPosition = constraints.getStartPosition();
        this.power = 0;
        this.velo = 0;

        this.p = constraints.getP(); this.i = constraints.getI(); this.d = constraints.getD();
        this.f = constraints.getF();
        this.pidController = new PIDControllerCustom(this.p, this.i, this.d);
    }

    public void initialize() {
        this.setMode("without");
        this.resetEncoder();
        this.currentPosition = 0;
        this.pidController.setPID(this.p, this.i, this.d);
    }

    public void moveToPosition(int height, int error) {
        this.setMode("with");
        this.setCurrentPosition();
        this.resetTimer();
        this.startTimer();

        while(Math.abs(height - this.currentPosition) >= error && !this.curOpMode.isStopRequested()) {
            this.setCurrentPosition();
            double pid = pidController.update(height, this.currentPosition);
            this.velo = pid;

            this.powerSlideVel(pid);

            this.LOG_STATS();
        }

        this.powerSlideVel(0);

        this.resetTimer();
    }

    public void teleopPID(int height, int error) {
        this.setCurrentPosition();
        double pid = pidController.update(height, this.currentPosition);
        this.velo = pid;

        this.powerSlideVel(pid);

        this.LOG_STATS();
    }

    public void setRunMode() {
        if(mode.equals("without")) this.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        else if(mode.equals("with")) this.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMode(String mode) {
        this.mode = mode;
        this.setRunMode();
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

    public void updatePIDConst(double p, double i, double d) {
        this.p = p; this.i = i; this.d = d;
        pidController.setPID(this.p, this.i, this.d);
    }

    public int setCurrentPosition() {
        this.currentPosition = slide.getCurrentPosition();
        return this.currentPosition;
    }

    public int getCurrentPosition() {
        return this.currentPosition;
    }

    public void resetEncoder() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setRunMode();
    }

    public void LOG_STATS() {
        String stats = "Current Power: " + this.power + "\nCurrent Velo: " + this.velo + "\nCurrent Position: " + this.currentPosition + "\nCurrent Time: " + this.timer.seconds();
        curOpMode.telemetry.addData("OuttakeSlides", stats);
        curOpMode.telemetry.update();
    }
}
