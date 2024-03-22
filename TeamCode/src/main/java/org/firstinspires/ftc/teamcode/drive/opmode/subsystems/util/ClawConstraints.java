package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util;

public class ClawConstraints extends Constraints {
    private double currentPositionStacked;
    private double currentPositionSingle;
    private double openSingle;
    private double closeSingle;
    private double openStacked;
    private double openOneStacked;
    private double closeStacked;

    public ClawConstraints(double currentPositionStacked, double currentPositionSingle, double openSingle, double closeSingle, double openStacked, double openOneStacked, double closeStacked) {
        this.currentPositionStacked = currentPositionStacked;
        this.currentPositionSingle = currentPositionSingle;
        this.openSingle = openSingle;
        this.closeSingle = closeSingle;
        this.openStacked = openStacked;
        this.openOneStacked = openOneStacked;
        this.closeStacked = closeStacked;
    }

    public double getCurrentPositionStacked() {
        return currentPositionStacked;
    }

    public void setCurrentPositionStacked(double currentPositionStacked) {
        this.currentPositionStacked = currentPositionStacked;
    }

    public double getCurrentPositionSingle() {
        return currentPositionSingle;
    }

    public void setCurrentPositionSingle(double currentPositionSingle) {
        this.currentPositionSingle = currentPositionSingle;
    }

    public double getOpenSingle() {
        return openSingle;
    }

    public void setOpenSingle(double openSingle) {
        this.openSingle = openSingle;
    }

    public double getCloseSingle() {
        return closeSingle;
    }

    public void setCloseSingle(double closeSingle) {
        this.closeSingle = closeSingle;
    }

    public double getOpenStacked() {
        return openStacked;
    }

    public void setOpenStacked(double openStacked) {
        this.openStacked = openStacked;
    }

    public double getOpenOneStacked() {
        return openOneStacked;
    }

    public void setOpenOneStacked(double openOneStacked) {
        this.openOneStacked = openOneStacked;
    }

    public double getCloseStacked() {
        return closeStacked;
    }

    public void setCloseStacked(double closeStacked) {
        this.closeStacked = closeStacked;
    }
}
