package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeClaw {
    public double currentPositionStacked, currentPositionSingle;
    public double OPEN_STACKED, OPEN_ONE_STACKED, CLOSE_STACKED;
    public double OPEN_SINGLE, CLOSE_SINGLE;

    public Servo stacked, single;

    LinearOpMode curOpMode;

    ElapsedTime timer = new ElapsedTime();

    public IntakeClaw(double currentPositionStacked, double currentPositionSingle, double openSingle, double closeSingle, double openStacked, double openOneStacked, double closeStacked, LinearOpMode op, String stackedName, String singleName) {
        this.curOpMode = op;
        stacked = op.hardwareMap.servo.get(stackedName);
        single = op.hardwareMap.servo.get(singleName);

        this.currentPositionSingle = currentPositionSingle;
        this.currentPositionStacked = currentPositionStacked;

        this.OPEN_STACKED = openStacked;
        this.OPEN_ONE_STACKED = openOneStacked;
        this.CLOSE_STACKED = closeStacked;
        this.OPEN_SINGLE = openSingle;
        this.CLOSE_SINGLE = closeSingle;

        stacked.setPosition(this.currentPositionStacked);
        single.setPosition(this.currentPositionSingle);
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
