package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControlSystem {
    private Gamepad gamepad1, gamepad2;
    private Bot robot;

    public ControlSystem(Gamepad gamepad1, Gamepad gamepad2, Bot robot) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robot = robot;
    }

    public void triggerSlides() {
        double axialLinearSlides = -gamepad2.left_stick_y;
        if(Math.abs(slides.getCurrentPosition()) >= Math.abs(slidePositionMax) && slides.getCurrentPower() >= 0) axialLS = 0;
        else if(Math.abs(slides.getCurrentPosition()) >= armPreventionThreshold && arm.getCurrentPosition() == arm.DROP) axialLS = 0;
        else axialLS = axialLS;
    }
}
