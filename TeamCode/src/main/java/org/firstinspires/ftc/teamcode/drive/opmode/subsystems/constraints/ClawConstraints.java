package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.constraints;

public class ClawConstraints extends Constraints {
    private double currentPosition;
    private double clawClose;
    private double clawOpen;

    public ClawConstraints(double currentPosition, double clawClose, double clawOpen) {
        this.currentPosition = currentPosition;
        this.clawClose = clawClose;
        this.clawOpen = clawOpen;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }

    public double getClawClose() {
        return clawClose;
    }

    public void setClawClose(double clawClose) {
        this.clawClose = clawClose;
    }

    public double getClawOpen() {
        return clawOpen;
    }

    public void setClawOpen(double clawOpen) {
        this.clawOpen = clawOpen;
    }
}