package org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util;

public class VisionConstraints {
    private int frontCameraHeight;
    private int frontCameraWidth;
    private int backCameraHeight;
    private int backCameraWidth;

    public VisionConstraints(int frontCameraHeight, int frontCameraWidth, int backCameraHeight, int backCameraWidth) {
        this.frontCameraHeight = frontCameraHeight;
        this.frontCameraWidth = frontCameraWidth;
        this.backCameraHeight = backCameraHeight;
        this.backCameraWidth = backCameraWidth;
    }

    public int getFrontCameraHeight() {
        return frontCameraHeight;
    }

    public void setFrontCameraHeight(int frontCameraHeight) {
        this.frontCameraHeight = frontCameraHeight;
    }

    public int getFrontCameraWidth() {
        return frontCameraWidth;
    }

    public void setFrontCameraWidth(int frontCameraWidth) {
        this.frontCameraWidth = frontCameraWidth;
    }

    public int getBackCameraHeight() {
        return backCameraHeight;
    }

    public void setBackCameraHeight(int backCameraHeight) {
        this.backCameraHeight = backCameraHeight;
    }

    public int getBackCameraWidth() {
        return backCameraWidth;
    }

    public void setBackCameraWidth(int backCameraWidth) {
        this.backCameraWidth = backCameraWidth;
    }
}
