package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util;

public class ArmConstraints extends Constraints {
    private double currentPosition;
    private double high;
    private double raised;
    private double ground;
    private double drop;

    public ArmConstraints(double currentPosition, double high, double raised, double ground, double drop) {
        this.currentPosition = currentPosition;
        this.high = high;
        this.raised = raised;
        this.ground = ground;
        this.drop = drop;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }

    public double getHigh() {
        return high;
    }

    public void setHigh(double high) {
        this.high = high;
    }

    public double getRaised() {
        return raised;
    }

    public void setRaised(double raised) {
        this.raised = raised;
    }

    public double getGround() {
        return ground;
    }

    public void setGround(double ground) {
        this.ground = ground;
    }

    public double getDrop() {
        return drop;
    }

    public void setDrop(double drop) {
        this.drop = drop;
    }
}
