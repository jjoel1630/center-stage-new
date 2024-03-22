package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util.ClawConstraints;

public class IntakeClaw {
    private double currentPositionStacked, currentPositionSingle;
    private double OPEN_STACKED, OPEN_ONE_STACKED, CLOSE_STACKED;
    private double OPEN_SINGLE, CLOSE_SINGLE;

    private Servo stacked, single;

    LinearOpMode curOpMode;

    ElapsedTime timer = new ElapsedTime();

    public IntakeClaw(LinearOpMode op, String stackedName, String singleName, ClawConstraints constraints) {
        this.curOpMode = op;
        this.stacked = op.hardwareMap.servo.get(stackedName);
        this.single = op.hardwareMap.servo.get(singleName);

        this.currentPositionSingle = constraints.getCurrentPositionSingle();
        this.currentPositionStacked = constraints.getCurrentPositionStacked();

        this.OPEN_STACKED = constraints.getOpenStacked();
        this.OPEN_ONE_STACKED = constraints.getOpenOneStacked();
        this.CLOSE_STACKED = constraints.getCloseStacked();
        this.OPEN_SINGLE = constraints.getOpenSingle();
        this.CLOSE_SINGLE = constraints.getCloseSingle();
    }

    public void initialize() {
        this.stacked.setPosition(this.currentPositionStacked);
        this.single.setPosition(this.currentPositionSingle);
    }

    public void setCurrentPositionStacked(double pos) {
        this.currentPositionStacked = pos;
    }

    public void setCurrentPositionSingle(double pos) {
        this.currentPositionSingle = pos;
    }

    public void moveStacked(double pos) {
        this.setCurrentPositionStacked(pos);
        stacked.setPosition(pos);
    }

    public void moveSingle(double pos) {
        this.setCurrentPositionSingle(pos);
        single.setPosition(pos);
    }

    public void stackedOpen() {
        this.setCurrentPositionStacked(this.OPEN_STACKED);
        this.moveStacked(this.OPEN_STACKED);
    }

    public void stackedOpenOne() {
        this.setCurrentPositionStacked(this.OPEN_ONE_STACKED);
        this.moveStacked(this.OPEN_ONE_STACKED);
    }

    public void stackedClose() {
        this.setCurrentPositionStacked(this.CLOSE_STACKED);
        this.moveStacked(this.CLOSE_STACKED);
    }

    public void singleOpen() {
        this.setCurrentPositionStacked(this.OPEN_SINGLE);
        this.moveSingle(this.OPEN_SINGLE);
    }

    public void singleClose() {
        this.setCurrentPositionStacked(this.CLOSE_SINGLE);
        this.moveSingle(this.CLOSE_SINGLE);
    }

    public void openBoth() {
        this.singleOpen();
        this.stackedOpen();
    }

    public void closeBoth() {
        this.singleClose();
        this.stackedClose();
    }

    public void LOG_STATS() {
        String stats = "Current Position Stacked: " + this.currentPositionStacked + "\nCurrent Position Single: " + this.currentPositionSingle;
        curOpMode.telemetry.addData("IntakeClaw", stats);
        curOpMode.telemetry.update();
    }
}
