package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util;

public class SlideConstraints extends Constraints {
    private int startPosition;
    private boolean rev;
    private double p;
    private double i;
    private double d;
    private double f;

    public SlideConstraints(int startPosition, boolean rev, double p, double i, double d, double f) {
        this.startPosition = startPosition;
        this.rev = rev;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public int getStartPosition() {
        return startPosition;
    }

    public void setStartPosition(int startPosition) {
        this.startPosition = startPosition;
    }

    public boolean isRev() {
        return rev;
    }

    public void setRev(boolean rev) {
        this.rev = rev;
    }

    public double getP() {
        return p;
    }

    public void setP(double p) {
        this.p = p;
    }

    public double getI() {
        return i;
    }

    public void setI(double i) {
        this.i = i;
    }

    public double getD() {
        return d;
    }

    public void setD(double d) {
        this.d = d;
    }

    public double getF() {
        return f;
    }

    public void setF(double f) {
        this.f = f;
    }
}
