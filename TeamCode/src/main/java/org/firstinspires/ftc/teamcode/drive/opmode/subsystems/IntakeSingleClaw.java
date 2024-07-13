package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSingleClaw {
    public double currentPosition;
    public double OPEN, CLOSE;

    public Servo claw;

    LinearOpMode curOpMode;

    ElapsedTime timer = new ElapsedTime();

    public IntakeSingleClaw(double currentPosition, double open, double close, LinearOpMode op, String clawName) {
        this.curOpMode = op;
        claw = op.hardwareMap.servo.get(clawName);

        this.currentPosition = currentPosition;

        this.OPEN = open;
        this.CLOSE = close;

        claw.setPosition(currentPosition);
    }

    public void setCurrentPosition(double pos) {
        this.currentPosition = pos;
    }

    public void moveClaw(double pos) {
        this.setCurrentPosition(pos);
        claw.setPosition(pos);
    }

    public void clawOpen() {
        this.setCurrentPosition(this.OPEN);
        this.moveClaw(this.OPEN);
    }

    public void clawClose() {
        this.setCurrentPosition(this.CLOSE);
        this.moveClaw(this.CLOSE);
    }

    public void LOG_STATS() {
        String stats = "Current Position: " + this.currentPosition;
        curOpMode.telemetry.addData("IntakeClaw", stats);
        curOpMode.telemetry.update();
    }
}
