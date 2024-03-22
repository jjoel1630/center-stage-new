package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.subsystems.util.ArmConstraints;

public class OuttakeArm {
    private double currentPosition;
    private double HIGH, RAISED, GROUND, DROP;

    private Servo arm;

    LinearOpMode curOpMode;

    ElapsedTime timer = new ElapsedTime();

    public OuttakeArm(LinearOpMode op, String name, ArmConstraints constraints) {
        this.curOpMode = op;
        this.arm = op.hardwareMap.servo.get(name);

        this.currentPosition = constraints.getCurrentPosition();
        this.HIGH = constraints.getHigh();
        this.RAISED = constraints.getRaised();
        this.GROUND = constraints.getGround();
        this.DROP = constraints.getDrop();
    }

    public void initialize() {
        this.moveArm(this.currentPosition);
    }

    public void setCurrentPosition(double pos) {
        this.currentPosition = pos;
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public void moveArm(double pos) {
        this.setCurrentPosition(pos);
        arm.setPosition(pos);
    }

    public void high() {
        this.setCurrentPosition(this.HIGH);
        this.moveArm(this.HIGH);
    }

    public void raise() {
        this.setCurrentPosition(this.RAISED);
        this.moveArm(this.RAISED);
    }

    public void ground() {
        this.setCurrentPosition(this.GROUND);
        this.moveArm(this.GROUND);
    }

    public void drop() {
        this.setCurrentPosition(this.DROP);
        this.moveArm(this.DROP);
    }

    public void LOG_STATS() {
        String stats = "Current Position: " + this.currentPosition;
        curOpMode.telemetry.addData("OuttakeArm", stats);
        curOpMode.telemetry.update();
    }
}