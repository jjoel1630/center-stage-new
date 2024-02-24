package org.firstinspires.ftc.teamcode.drive.opmode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeArm {
    public double currentPosition;
    public double HIGH, RAISED, GROUND, DROP;

    public Servo arm;

    LinearOpMode curOpMode;

    ElapsedTime timer = new ElapsedTime();

    public OuttakeArm(double currentPosition, double high, double raised, double ground, double drop, LinearOpMode op, String name) {
        this.curOpMode = op;
        arm = op.hardwareMap.servo.get(name);

        this.currentPosition = currentPosition;
        this.HIGH = high;
        this.RAISED = raised;
        this.GROUND = ground;
        this.DROP = drop;

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

    public void initialize() {
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