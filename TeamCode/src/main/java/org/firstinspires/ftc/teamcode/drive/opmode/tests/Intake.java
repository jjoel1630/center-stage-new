package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake extends LinearOpMode {
    private DcMotorEx intakeMotor = null; //setting intake motor variable

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        waitForStart();

        while (opModeIsActive()) {
            intakeMotor.setPower(1);
        }
    }
}
